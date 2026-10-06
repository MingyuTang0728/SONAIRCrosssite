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

The cost is small: four 2 ms MuJoCo steps per 8 ms packet.
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
        self.arm = None
        self.enabled = False
        self.why = ""
        self.model_words = ""
        self._lock = threading.Lock()
        self._svc = None
        self._last_ts = None
        self._n = 0
        self._real_q = None
        self._sim_q = None
        self._err = deque()          # (t, [6 joint errors, rad])
        self._tcp = deque()          # (t, mm)
        self._tcp_now = None
        self._tool_off = None
        self.resets = 0

    # -- lifecycle --------------------------------------------------------
    def load(self, carrier_mass_kg: float = 0.0) -> bool:
        try:
            import sim_mujoco as sm
            ok, why = sm.available()
            if not ok:
                raise RuntimeError(why)
            err = None
            # A Track A submission can be the twin instead of S0: the model a
            # team submitted, running beside the real arm, live.
            submitted = os.environ.get("SONAIR_TWIN_MODEL", "").strip()
            if submitted:
                path = sm.wrap_model(Path(submitted))
            else:
                for d in menagerie_dirs():
                    try:
                        path = sm.ensure_model(d)
                        break
                    except FileNotFoundError as e:
                        err = e
                else:
                    raise RuntimeError(
                        "the MuJoCo model library is not installed. Run: "
                        "python install_sim.py")
            self.arm = sm.Arm(path, carrier_mass_kg=carrier_mass_kg)
            import mujoco
            which = (f"submitted model {Path(submitted).name}" if submitted
                     else "menagerie UR5e (the benchmark's S0)")
            self.model_words = (f"MuJoCo {mujoco.__version__}, {which}, payload "
                                f"{carrier_mass_kg:.2f} kg")
            self.why = ""
            del err
            return True
        except Exception as e:      # noqa: BLE001
            self.arm, self.why = None, str(e)
            return False

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
    def _on_packet(self, st: dict) -> None:
        tq, aq = st.get("target_q"), st.get("actual_q")
        ts = st.get("timestamp")
        if not tq or not aq or not isinstance(ts, (int, float)):
            return
        arm = self.arm
        with self._lock:
            dt = None if self._last_ts is None else float(ts) - self._last_ts
            self._last_ts = float(ts)
            if dt is None or not (0.0 < dt < 0.25):
                # first packet, or the stream jumped: the twin starts again
                # from where the real arm is
                arm.reset(aq)
                self.resets += 1
                self._err.clear()
            else:
                arm.drive(tq, dt)
            sq = arm.joints()
            self._real_q, self._sim_q = list(aq), sq
            now = float(ts)
            self._err.append((now, [a - b for a, b in zip(aq, sq)]))
            while self._err and now - self._err[0][0] > WINDOW_S:
                self._err.popleft()
            self._n += 1
            if self._n % TCP_EVERY == 0:
                self._tcp_point(st, sq, now)

    def _tcp_point(self, st, sq, now):
        tcp = st.get("actual_TCP_pose")
        if not tcp:
            return
        try:
            import campaign_runner as cr
            if self._tool_off is None:
                self._tool_off = cr._tool_offset(st["actual_q"], tcp)
            p = cr._tool_at(sq, self._tool_off)
            d = 1000.0 * math.dist(p, tcp[:3])
            self._tcp_now = d
            self._tcp.append((now, d))
            while self._tcp and now - self._tcp[0][0] > WINDOW_S:
                self._tcp.popleft()
        except Exception:       # noqa: BLE001
            pass

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
            }


TWIN = Twin()
