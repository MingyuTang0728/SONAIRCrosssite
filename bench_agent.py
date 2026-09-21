"""
bench_agent.py — SONAIR benchmark acquisition, host side.

Sits beside multimodal_bridge.py on the workstation wired to the UR5e and owns
the three things the bridge should not have to know about:

  1. INERTIAL INGESTION from up to three tiers of unit at once —
     the industrial unit via FusionHub, a consumer module, and the D435i's own
     BMI055. All three land in one canonical record shape.

  2. THE TIME MASTER. Every channel is stamped against one clock. The robot
     state, the IMUs and the camera frames each arrive on their own clock, and
     the offsets between them are MEASURED here (tap verification) rather than
     assumed to be zero. A temporal misalignment that is assumed away reappears
     downstream as a position error and gets attributed to the sim-to-real gap.

  3. RUN RECORDING in the canonical schema, so what the arm produces is
     already in the form sonair_benchmark.metrics expects — no conversion
     step, and therefore no conversion step to get wrong halfway through a
     four-week campaign.

Everything here is import-tolerant: no RealSense, no FusionHub, no benchmark
package on the path and the module still loads, with the corresponding source
reporting itself as unavailable. The bridge must boot on a developer laptop.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from collections import deque
from pathlib import Path

log = logging.getLogger("bench")

try:
    import pyrealsense2 as rs
    _HAS_RS = True
except ImportError:
    _HAS_RS = False

try:
    from sonair_benchmark.imu import FusionHubUdpSource, make_record, quat_normalise
    from sonair_benchmark.clock import detect_tap, fit_offset, tap_alignment
    from sonair_benchmark.schema import RunManifest, RunWriter, Sample
    _HAS_BENCH = True
except ImportError:  # pragma: no cover - the package travels with this file
    _HAS_BENCH = False
    log.warning("sonair_benchmark package not importable — recording disabled")


# ============================================================
# Time master
# ============================================================

class TimeMaster:
    """
    The host's monotonic clock stands in for the Teensy until the Teensy is
    wired in Phase 1. The interface does not change when it is: callers ask
    for `now()` and register per-channel offsets, and swapping the reference
    is one method.

    Using a MONOTONIC clock rather than wall time is not a detail. Wall time
    can step backwards mid-run under NTP correction, and a run containing a
    backwards time step is silently unusable.
    """

    def __init__(self):
        self._t0 = time.monotonic()
        self._wall0 = time.time()
        self.offsets: dict[str, float] = {}
        self.residuals: dict[str, float] = {}
        self.source = "host-monotonic"

    def now(self) -> float:
        return time.monotonic() - self._t0

    def wall_of(self, t: float) -> float:
        return self._wall0 + t

    def set_offset(self, channel: str, offset_s: float, residual_s: float = 0.0) -> None:
        self.offsets[channel] = float(offset_s)
        self.residuals[channel] = float(residual_s)

    def to_master(self, channel: str, t_src: float) -> float:
        return float(t_src) + self.offsets.get(channel, 0.0)

    def measured_channels(self) -> list[str]:
        return sorted(self.offsets)

    def status(self) -> dict:
        return {
            "source": self.source,
            "t": round(self.now(), 4),
            "offsets_ms": {k: round(v * 1000.0, 3) for k, v in self.offsets.items()},
            "residuals_ms": {k: round(v * 1000.0, 3) for k, v in self.residuals.items()},
            "worst_residual_ms": round(max(self.residuals.values(), default=0.0) * 1000.0, 3),
        }


MASTER = TimeMaster()


# ============================================================
# IMU hub — every unit, one shape
# ============================================================

class ImuHub:
    """
    Holds the latest sample from each inertial unit plus a short ring buffer
    per unit, which is what the tap verification reads.

    `latest()` is what the browser polls; it is deliberately a snapshot rather
    than a stream subscription, so a slow browser can never back-pressure
    acquisition.
    """

    RING = 4096

    def __init__(self):
        self._lock = threading.Lock()
        self._latest: dict[str, tuple[float, dict]] = {}
        self._rings: dict[str, deque] = {}
        self._counts: dict[str, int] = {}
        self._rates: dict[str, float] = {}
        self._last_rate_calc: dict[str, tuple[float, int]] = {}

    def push(self, unit: str, t_master: float, rec: dict) -> None:
        with self._lock:
            self._latest[unit] = (t_master, rec)
            ring = self._rings.get(unit)
            if ring is None:
                ring = self._rings[unit] = deque(maxlen=self.RING)
            ring.append((t_master, rec))
            self._counts[unit] = self._counts.get(unit, 0) + 1
            # rolling rate estimate, recomputed once a second
            last = self._last_rate_calc.get(unit)
            if last is None:
                self._last_rate_calc[unit] = (t_master, self._counts[unit])
            elif t_master - last[0] >= 1.0:
                dn = self._counts[unit] - last[1]
                self._rates[unit] = dn / (t_master - last[0])
                self._last_rate_calc[unit] = (t_master, self._counts[unit])

    def latest(self) -> dict:
        with self._lock:
            return {u: {"t": round(t, 5), **rec} for u, (t, rec) in self._latest.items()}

    def snapshot(self) -> dict:
        """The per-unit block that goes into one recorded Sample."""
        with self._lock:
            return {u: dict(rec) for u, (_, rec) in self._latest.items()}

    def ring(self, unit: str) -> list:
        with self._lock:
            return list(self._rings.get(unit, ()))

    def status(self) -> dict:
        with self._lock:
            return {
                u: {"samples": self._counts.get(u, 0),
                    "rate_hz": round(self._rates.get(u, 0.0), 1),
                    "age_s": round(MASTER.now() - self._latest[u][0], 3)}
                for u in self._latest
            }


HUB = ImuHub()


# ============================================================
# Source: the D435i's own BMI055
# ============================================================

class D435iImuSource:
    """
    The camera you already own contains an IMU. It is a consumer-grade part
    and it is not a substitute for the industrial unit, but it costs nothing,
    it is rigidly coupled to the camera whose extrinsics you will calibrate
    anyway, and it is available the moment the USB cable is in.

    It runs on its OWN pipeline, separate from the depth/colour pipeline in
    multimodal_bridge.camera_thread. That is deliberate: the motion streams
    run at 200/63 Hz against the image streams' 30 Hz, and forcing them into
    one wait_for_frames() throttles the IMU to the frame rate, which destroys
    the only property that made it worth logging.
    """

    UNIT = "d435i"

    def __init__(self, accel_hz: int = 63, gyro_hz: int = 200):
        self.accel_hz = accel_hz
        self.gyro_hz = gyro_hz
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.available = _HAS_RS
        self.error = "" if _HAS_RS else "pyrealsense2 not installed"
        self.n = 0

    def start(self) -> bool:
        if not self.available:
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="d435i-imu")
        self._thread.start()
        return True

    def _loop(self) -> None:
        while not self._stop.is_set():
            pipe = rs.pipeline()
            cfg = rs.config()
            try:
                cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, self.accel_hz)
                cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.gyro_hz)
                pipe.start(cfg)
                self.error = ""
                log.info("D435i IMU started (accel %d Hz, gyro %d Hz)",
                         self.accel_hz, self.gyro_hz)
            except Exception as e:
                self.error = str(e)
                log.warning("D435i IMU start failed: %s — retrying in 5 s", e)
                time.sleep(5)
                continue

            accel = [0.0, 0.0, 0.0]
            gyro = [0.0, 0.0, 0.0]
            while not self._stop.is_set():
                try:
                    frames = pipe.wait_for_frames(timeout_ms=2000)
                except Exception:
                    break
                got = False
                for f in frames:
                    mf = f.as_motion_frame()
                    if not mf:
                        continue
                    d = mf.get_motion_data()
                    prof = mf.get_profile().stream_type()
                    if prof == rs.stream.accel:
                        accel = [d.x, d.y, d.z]
                        got = True
                    elif prof == rs.stream.gyro:
                        gyro = [d.x, d.y, d.z]
                        got = True
                if got:
                    # The camera stamps in its own clock; the offset to the
                    # master is measured by tap verification, not assumed.
                    t = MASTER.to_master(self.UNIT, MASTER.now())
                    HUB.push(self.UNIT, t, {"accel": list(accel), "gyro": list(gyro)})
                    self.n += 1
            try:
                pipe.stop()
            except Exception:
                pass

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def status(self) -> dict:
        return {"unit": self.UNIT, "available": self.available,
                "error": self.error, "samples": self.n,
                "running": bool(self._thread and self._thread.is_alive())}


# ============================================================
# Source: FusionHub (the industrial unit)
# ============================================================

class FusionHubBridge:
    """
    Wraps sonair_benchmark.imu.FusionHubUdpSource and pushes into the hub.

    FusionHub is configured to stream JSON over UDP to this port. That is the
    only integration point — the benchmark never asks FusionHub for anything,
    it only listens. If the live stream is not available yet, record in
    FusionHub and use the replay path instead; the schema is identical, so
    nothing downstream changes when you switch over.
    """

    def __init__(self, port: int = 5005, unit: str = "ind0"):
        self.port = port
        self.unit = unit
        self.src = None
        self.error = "" if _HAS_BENCH else "sonair_benchmark not importable"

    def start(self) -> bool:
        if not _HAS_BENCH:
            return False
        try:
            self.src = FusionHubUdpSource(
                port=self.port, unit_id=self.unit,
                on_sample=lambda t_src, rec: HUB.push(
                    self.unit, MASTER.to_master(self.unit, t_src), rec))
            self.src.start()
            log.info("FusionHub listener on UDP %d as unit %s", self.port, self.unit)
            return True
        except Exception as e:
            self.error = str(e)
            log.warning("FusionHub listener failed: %s", e)
            return False

    def stop(self) -> None:
        if self.src:
            self.src.stop()

    def status(self) -> dict:
        base = {"unit": self.unit, "port": self.port, "error": self.error}
        if self.src:
            base.update(self.src.health())
        return base


# ============================================================
# Tap verification — the Phase 1 exit check
# ============================================================

def verify_tap(window_s: float = 5.0) -> dict:
    """
    One sharp mechanical event on the carrier, seen by every inertial channel.

    Finds the tap in each unit's ring buffer and reports the spread of arrival
    times. That spread is the temporal row of the error budget, and it is
    quoted in every later result. Anything above a couple of milliseconds
    means the channels are not on one clock yet.
    """
    if not _HAS_BENCH:
        return {"ok": False, "error": "sonair_benchmark not importable"}
    now = MASTER.now()
    found: dict[str, float] = {}
    for unit in list(HUB.status()):
        ring = [(t, r) for t, r in HUB.ring(unit) if now - t <= window_s]
        if len(ring) < 20:
            continue
        ts = [t for t, _ in ring]
        mags = []
        for _, rec in ring:
            a = rec.get("accel")
            mags.append(math.sqrt(sum(v * v for v in a)) if a else 0.0)
        t_tap = detect_tap(ts, mags, k=6.0)
        if t_tap is not None:
            found[unit] = t_tap
    if len(found) < 2:
        return {"ok": False, "n_channels": len(found), "per_channel": found,
                "error": "need a detectable tap on at least two channels — "
                         "tap the carrier once, firmly, then re-run"}
    res = tap_alignment(found)
    res["ok"] = res["spread_s"] < 0.005
    res["spread_ms"] = res["spread_s"] * 1000.0
    res["advice"] = ("channels are aligned to within 5 ms"
                     if res["ok"] else
                     "spread above 5 ms — measure the per-channel offset and "
                     "apply it before recording, or this shows up later as a "
                     "position error blamed on the sim-to-real gap")
    return res


# ============================================================
# Run recorder
# ============================================================

class BenchRecorder:
    """
    Writes one canonical run file per recording, sampling the shared robot
    state and the IMU hub at a fixed rate against the master clock.

    Sampling on a fixed grid rather than on each channel's arrival is what
    makes the real and simulated runs directly comparable: Phase 4 generates
    at the same declared rate, so the two sides need no resampling to be
    differenced, and resampling that is not needed is error that is not added.
    """

    def __init__(self, out_dir: str | Path = "./bench_runs"):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._writer = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.current: dict | None = None
        self.last: dict | None = None
        self.state_fn = None  # set by the bridge: () -> (q, tcp_pose)

    def is_recording(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def start(self, *, run_id: str, joint_vel: float, arm_config: str,
              traj_type: str, repeat_idx: int, calib_version: str,
              rate_hz: float = 125.0, carrier_mass_kg: float = 0.0,
              carrier_id: str = "carrier-v1", operator: str = "",
              notes: str = "") -> dict:
        if not _HAS_BENCH:
            return {"ok": False, "error": "sonair_benchmark package not importable"}
        if self.is_recording():
            return {"ok": False, "error": "already recording; stop the current run first"}

        manifest = RunManifest(
            run_id=run_id, side="real", calib_version=calib_version,
            joint_vel=float(joint_vel), arm_config=arm_config,
            traj_type=traj_type, repeat_idx=int(repeat_idx),
            carrier_id=carrier_id, carrier_mass_kg=float(carrier_mass_kg),
            sample_rate_hz=float(rate_hz),
            started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            operator=operator, notes=notes,
        )
        problems = manifest.validate()
        if problems:
            return {"ok": False, "error": "; ".join(problems)}

        path = self.out_dir / f"{run_id}.jsonl"
        try:
            self._writer = RunWriter(path, manifest)
        except Exception as e:
            return {"ok": False, "error": str(e)}

        self._stop.clear()
        self.current = {"run_id": run_id, "path": str(path),
                        "started": MASTER.now(), "rate_hz": rate_hz, "n": 0}
        self._thread = threading.Thread(target=self._loop, args=(rate_hz,),
                                        daemon=True, name=f"bench-rec-{run_id}")
        self._thread.start()
        log.info("recording run %s -> %s", run_id, path)
        return {"ok": True, "run_id": run_id, "path": str(path)}

    def _loop(self, rate_hz: float) -> None:
        period = 1.0 / max(1.0, rate_hz)
        next_t = MASTER.now()
        n = 0
        while not self._stop.is_set():
            now = MASTER.now()
            if now < next_t:
                time.sleep(min(period, max(0.0, next_t - now)))
                continue
            next_t += period
            # If we fall far behind (a GC pause, a disk hiccup), resynchronise
            # rather than sprinting to catch up — a burst of samples all
            # stamped microseconds apart is worse than a visible gap.
            if MASTER.now() - next_t > 0.25:
                next_t = MASTER.now()

            q = tcp = None
            if self.state_fn:
                try:
                    q, tcp = self.state_fn()
                except Exception:
                    pass
            sample = Sample(
                t=now,
                q=list(q) if q else None,
                tcp_pos=list(tcp[:3]) if tcp else None,
                tcp_rot=list(tcp[3:6]) if tcp and len(tcp) >= 6 else None,
                imu=HUB.snapshot(),
            )
            with self._lock:
                if self._writer:
                    try:
                        self._writer.write(sample)
                        n += 1
                        if self.current:
                            self.current["n"] = n
                    except Exception as e:
                        log.warning("sample write failed: %s", e)
                        break

    def stop(self) -> dict:
        if not self.is_recording():
            return {"ok": False, "error": "not recording"}
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        with self._lock:
            n = self._writer.n if self._writer else 0
            if self._writer:
                self._writer.close()
            self._writer = None
        cur = self.current or {}
        self.last = {**cur, "n": n, "stopped": MASTER.now()}
        self.current = None
        log.info("run %s finished: %d samples", self.last.get("run_id"), n)
        return {"ok": True, **self.last}

    def status(self) -> dict:
        return {
            "recording": self.is_recording(),
            "current": self.current,
            "last": self.last,
            "out_dir": str(self.out_dir.resolve()),
            "available": _HAS_BENCH,
        }


RECORDER = BenchRecorder()
D435I = D435iImuSource()
FUSIONHUB = FusionHubBridge()


def start_sources(*, d435i: bool = True, fusionhub: bool = True,
                  fusionhub_port: int = 5005) -> dict:
    """Called once from the bridge's main(). Never raises."""
    out = {}
    if d435i:
        out["d435i"] = D435I.start()
    if fusionhub:
        FUSIONHUB.port = fusionhub_port
        out["fusionhub"] = FUSIONHUB.start()
    return out


def status() -> dict:
    """One blob the browser polls to render the acquisition panel."""
    return {
        "clock": MASTER.status(),
        "units": HUB.status(),
        "sources": {"d435i": D435I.status(), "fusionhub": FUSIONHUB.status()},
        "recorder": RECORDER.status(),
        "bench_available": _HAS_BENCH,
    }


def handle_message(data: dict) -> dict | None:
    """
    Benchmark control messages from the browser. Returns a reply dict, or None
    if the message is not ours — so the bridge can chain this into its existing
    dispatch without a second dispatch table to keep in step.
    """
    mtype = data.get("type")
    if mtype == "bench_status":
        return {"type": "bench_status", **status()}
    if mtype == "bench_start":
        res = RECORDER.start(
            run_id=data.get("run_id") or f"run_{int(time.time())}",
            joint_vel=data.get("joint_vel", 0.4),
            arm_config=data.get("arm_config", "mid_workspace"),
            traj_type=data.get("traj_type", "contour"),
            repeat_idx=data.get("repeat_idx", 0),
            calib_version=data.get("calib_version", "calib-0"),
            rate_hz=data.get("rate_hz", 125.0),
            carrier_mass_kg=data.get("carrier_mass_kg", 0.0),
            carrier_id=data.get("carrier_id", "carrier-v1"),
            operator=data.get("operator", ""),
            notes=data.get("notes", ""),
        )
        return {"type": "bench_start_res", **res}
    if mtype == "bench_stop":
        return {"type": "bench_stop_res", **RECORDER.stop()}
    if mtype == "bench_tap":
        return {"type": "bench_tap_res", **verify_tap(data.get("window_s", 5.0))}
    if mtype == "bench_offset":
        MASTER.set_offset(data.get("channel", ""), data.get("offset_s", 0.0),
                          data.get("residual_s", 0.0))
        return {"type": "bench_offset_res", "ok": True, **MASTER.status()}
    return None
