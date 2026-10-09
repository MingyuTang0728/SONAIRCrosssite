"""
twin.py -- a live digital twin: the benchmark's simulator, running beside
the real arm.

Every RTDE packet carries the controller's commanded joints, target_q. The
twin feeds exactly that to the same MuJoCo model the benchmark replays
offline (sim_mujoco.Arm, the S0 baseline), one packet at a time, at the
packet's own controller time. So at every instant there are two arms: the
real one, and what the simulator says the real one should be doing. Their
difference is the sim-to-real gap, live -- while the job runs, for the
operator watching, and not only after the offline replay.

What it is for, and what it is not:

  * for SEEING the gap as it happens: which joint, how big, when. The
    console draws the twin as a ghost over the real arm and shows the error;
  * for catching a setup fault early: a twin that disagrees by centimetres
    at rest means a wrong payload, tool offset or model -- found during
    the first run, not after the campaign;
  * NOT the scored simulation. The scored sim side is the offline replay of
    each recorded run (sim_mujoco.py), because it is reproducible and does
    not depend on whether this PC kept up. The twin runs the same model with
    the same input, so the two agree, but only the replay is the record.

A CANDIDATE model can run beside S0 -- a team's submitted MuJoCo model, or
one of ours -- driven by the same packets. Each one's tool point is compared
with the real arm's, and live_bench.py turns those comparisons into the
benchmark's own score (GCR against S0), live and per recorded run.

The cost is small: four 2 ms MuJoCo steps per 8 ms packet, per model.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

log = logging.getLogger("twin")

HERE = Path(__file__).resolve().parent
WINDOW_S = 5.0          # rolling window for the RMS figures
TCP_EVERY = 4           # packets between tool-point comparisons (~31 Hz)


def menagerie_dirs():
    out = []
    if os.environ.get("MENAGERIE"):
        out.append(Path(os.environ["MENAGERIE"]))
    out.append(HERE / "mujoco_menagerie")
    if os.environ.get("LOCALAPPDATA"):
        out.append(Path(os.environ["LOCALAPPDATA"]) / "SONAIR" / "mujoco_menagerie")
    out.append(Path.home() / ".sonair" / "mujoco_menagerie")
    return out


class Twin:
    def __init__(self):
        self.arm = None              # S0: always the reference, always first
        self.candidate = None        # (name, Arm, path) or None
        self.enabled = False
        self.why = ""
        self.model_words = ""
        self.carrier_mass_kg = 0.0
        self.run_fn = None           # -> the recorder's current run, or None
        self._lock = threading.Lock()
        self._svc = None
        self._last_ts = None
        self._n = 0
        self._real_q = None
        self._sim_q = None
        self._cand_q = None
        self._err = deque()          # (t, [6 joint errors, rad])
        self._tcp = deque()          # (t, mm)
        self._tcp_now = None
        self._cand_tcp_now = None
        self._offsets = {}           # arm name -> tool point in its site frame
        self.resets = 0

    # -- lifecycle --------------------------------------------------------
    def load(self, carrier_mass_kg: float = 0.0) -> bool:
        """Load S0 -- and a candidate if SONAIR_TWIN_MODEL names one."""
        try:
            import sim_mujoco as sm
            ok, why = sm.available()
            if not ok:
                raise RuntimeError(why)
            for d in menagerie_dirs():
                try:
                    path = sm.ensure_model(d)
                    break
                except FileNotFoundError:
                    pass
            else:
                raise RuntimeError(
                    "the MuJoCo model library is not installed. Run: "
                    "python install_sim.py")
            self.carrier_mass_kg = float(carrier_mass_kg)
            self.arm = sm.Arm(path, carrier_mass_kg=carrier_mass_kg)
            import mujoco
            self.model_words = (f"MuJoCo {mujoco.__version__}, menagerie UR5e "
                                f"(the benchmark's S0), payload "
                                f"{carrier_mass_kg:.2f} kg")
            self.why = ""
            submitted = os.environ.get("SONAIR_TWIN_MODEL", "").strip()
            if submitted and self.candidate is None:
                res = self.set_candidate(submitted)
                if not res.get("ok"):
                    log.warning("candidate model not loaded: %s", res.get("error"))
            return True
        except Exception as e:      # noqa: BLE001
            self.arm, self.why = None, str(e)
            return False

    def set_candidate(self, path: str, name: str = "") -> dict:
        """
        Run a second model beside S0: a Track A submission, or any MuJoCo
        model that keeps the benchmark's contract (UR joint names in UR order,
        six position actuators, an attachment_site). Our sensor block is
        wrapped round it, so it is measured at the same point as S0.
        """
        try:
            import sim_mujoco as sm
            p = Path(str(path).strip().strip('"'))
            if not p.exists():
                return {"ok": False, "error": f"no such file: {p}"}
            arm = sm.Arm(sm.wrap_model(p), carrier_mass_kg=self.carrier_mass_kg)
        except Exception as e:      # noqa: BLE001
            return {"ok": False, "error": f"the model could not be used: {e}"}
        name = name or p.stem
        if name == "S0":
            name = "candidate"
        with self._lock:
            self.candidate = (name, arm, str(p))
            self._last_ts = None          # both start again from the real arm
        try:
            import live_bench
            live_bench.LIVE.models[name] = str(p)
        except Exception:                   # noqa: BLE001
            pass
        log.info("candidate model running beside S0: %s (%s)", name, p)
        return {"ok": True, "name": name, "path": str(p)}

    def clear_candidate(self) -> dict:
        with self._lock:
            gone = self.candidate[0] if self.candidate else None
            self.candidate = None
            self._cand_q = self._cand_tcp_now = None
        try:
            import live_bench
            live_bench.LIVE.models.pop(gone, None)
        except Exception:                   # noqa: BLE001
            pass
        return {"ok": True}

    def start(self, telemetry, carrier_mass_kg: float = 0.0) -> dict:
        if self.arm is None and not self.load(carrier_mass_kg):
            return {"ok": False, "error": self.why}
        if telemetry is None or not hasattr(telemetry, "subscribe"):
            return {"ok": False, "error": "the robot link is not running"}
        self.stop()
        with self._lock:
            self._svc = telemetry
            self._last_ts = None
            self._err.clear()
            self._tcp.clear()
        telemetry.subscribe(self._on_packet)
        self.enabled = True
        log.info("digital twin running: %s", self.model_words)
        return {"ok": True, "model": self.model_words}

    def stop(self) -> None:
        svc, self._svc = self._svc, None
        if svc is not None:
            try:
                svc.unsubscribe(self._on_packet)
            except Exception:       # noqa: BLE001
                pass
        self.enabled = False

    # -- one packet ---------------------------------------------------------
    def _arms(self):
        out = [("S0", self.arm)]
        if self.candidate is not None:
            out.append((self.candidate[0], self.candidate[1]))
        return out

    def _on_packet(self, st: dict) -> None:
        tq, aq = st.get("target_q"), st.get("actual_q")
        ts = st.get("timestamp")
        if not tq or not aq or not isinstance(ts, (int, float)):
            return
        with self._lock:
            arms = self._arms()
            dt = None if self._last_ts is None else float(ts) - self._last_ts
            self._last_ts = float(ts)
            if dt is None or not (0.0 < dt < 0.25):
                # first packet, or the stream jumped: the twin starts again
                # from where the real arm is
                for _, a in arms:
                    a.reset(aq)
                self._offsets.clear()
                self.resets += 1
                self._err.clear()
            else:
                for _, a in arms:
                    a.drive(tq, dt)
            sq = arms[0][1].joints()
            self._real_q, self._sim_q = list(aq), sq
            self._cand_q = arms[1][1].joints() if len(arms) > 1 else None
            now = float(ts)
            self._err.append((now, [a - b for a, b in zip(aq, sq)]))
            while self._err and now - self._err[0][0] > WINDOW_S:
                self._err.popleft()
            self._n += 1
            errs = (self._tcp_points(st, arms, now)
                    if self._n % TCP_EVERY == 0 else None)
        if errs:
            try:
                import live_bench
                run = self.run_fn() if self.run_fn else None
                live_bench.LIVE.add(now, errs, run)
            except Exception as e:      # noqa: BLE001
                log.debug("live bench: %s", e)

    def _tcp_points(self, st, arms, now):
        """Each model's tool point against the real one, in mm."""
        tcp = st.get("actual_TCP_pose")
        if not tcp:
            return None
        out = {}
        for name, a in arms:
            try:
                pos, R = _site_pose(a)
                off = self._offsets.get(name)
                if off is None:
                    # Taken where the model was just set to the real arm's
                    # joints: the tool point in this model's own flange
                    # frame. It is fixed to the flange and turns with it.
                    off = [sum(R[k][i] * (tcp[k] - pos[k]) for k in range(3))
                           for i in range(3)]
                    self._offsets[name] = off
                p = [pos[i] + sum(R[i][k] * off[k] for k in range(3))
                     for i in range(3)]
                out[name] = 1000.0 * math.dist(p, tcp[:3])
            except Exception:       # noqa: BLE001
                pass
        if "S0" in out:
            self._tcp_now = out["S0"]
            self._tcp.append((now, out["S0"]))
            while self._tcp and now - self._tcp[0][0] > WINDOW_S:
                self._tcp.popleft()
        if len(arms) > 1:
            self._cand_tcp_now = out.get(arms[1][0])
        return out

    # -- what the console shows -------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            if not self.enabled or self._sim_q is None:
                return {"enabled": self.enabled, "why": self.why,
                        "model": self.model_words}
            errs = [e for _, e in self._err]
            rms = [math.sqrt(sum(e[i] ** 2 for e in errs) / len(errs))
                   for i in range(6)] if errs else [0.0] * 6
            peak = [max(abs(e[i]) for e in errs) for i in range(6)] if errs else [0.0] * 6
            tcp = [d for _, d in self._tcp]
            return {
                "enabled": True, "model": self.model_words,
                "q": list(self._sim_q),
                "err_deg": [math.degrees(a - b)
                            for a, b in zip(self._real_q, self._sim_q)],
                "rms_deg": [math.degrees(v) for v in rms],
                "peak_deg": [math.degrees(v) for v in peak],
                "tcp_mm": self._tcp_now,
                "tcp_rms_mm": (math.sqrt(sum(d * d for d in tcp) / len(tcp))
                               if tcp else None),
                "window_s": WINDOW_S, "resets": self.resets,
                "candidate": ({"name": self.candidate[0],
                               "path": self.candidate[2],
                               "q": self._cand_q,
                               "tcp_mm": self._cand_tcp_now}
                              if self.candidate is not None else None),
            }


def _site_pose(arm):
    """attachment_site's position and rotation matrix, from the model's sensors."""
    pos = arm.read("tcp_pos")
    w, x, y, z = arm.read("tcp_quat")
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return pos, R


TWIN = Twin()
