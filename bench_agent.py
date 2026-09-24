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
    from sonair_benchmark.attitude import AttitudeTracker
    _HAS_BENCH = True
except ImportError:  # pragma: no cover - the package travels with this file
    _HAS_BENCH = False
    AttitudeTracker = None
    log.warning("sonair_benchmark package not importable — recording disabled")

try:
    import sensor_hub
    _HAS_SENSORS = True
except Exception:       # noqa: BLE001
    sensor_hub = None
    _HAS_SENSORS = False

try:
    import imu_link
    _HAS_LINK = True
    _LINK_ERR = ""
except Exception as e:      # noqa: BLE001
    imu_link = None
    _HAS_LINK = False
    _LINK_ERR = str(e)


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
        self._trackers: dict = {}
        self._tlock = threading.Lock()
        # Anything that wants every sample as it arrives, rather than the
        # latest one when it happens to look. The continuous logger is the
        # only subscriber today; the point of the hook is that a logger does
        # not have to poll, because a poller loses samples between polls and
        # a benchmark capture that quietly loses samples is worthless.
        self._sinks: list = []

    def subscribe(self, fn) -> None:
        with self._lock:
            if fn not in self._sinks:
                self._sinks.append(fn)

    def unsubscribe(self, fn) -> None:
        with self._lock:
            if fn in self._sinks:
                self._sinks.remove(fn)

    def tracker(self, unit: str):
        """
        One attitude tracker per unit, created on first sight of that unit.

        Kept in the hub rather than in each source so that a unit which
        changes transport mid-campaign — FusionHub over UDP on Monday, the
        same unit over serial on Tuesday — keeps one continuous orientation
        estimate and one gyro-bias history instead of silently restarting.
        """
        if AttitudeTracker is None:
            return None
        with self._tlock:
            tr = self._trackers.get(unit)
            if tr is None:
                tr = self._trackers[unit] = AttitudeTracker(unit)
            return tr

    def reset_tracker(self, unit: str) -> bool:
        if AttitudeTracker is None:
            return False
        with self._tlock:
            self._trackers[unit] = AttitudeTracker(unit)
        return True

    def tracker_status(self) -> dict:
        with self._tlock:
            return {u: t.status() for u, t in self._trackers.items()}

    def push(self, unit: str, t_master: float, rec: dict) -> None:
        # Derive orientation BEFORE storing, so the recorded sample and the
        # sample the browser renders are the same object. Deriving it in the
        # display path only would mean the run file silently lacks the very
        # modality the benchmark is scored on.
        tr = self.tracker(unit)
        if tr is not None:
            try:
                rec = {**rec, **tr.update(t_master, rec)}
            except Exception as e:      # noqa: BLE001
                log.debug("attitude update failed for %s: %s", unit, e)
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
            sinks = list(self._sinks)
        # Sinks run OUTSIDE the lock. A logger that blocks on a disk write
        # while holding the hub lock stalls every inertial link feeding it,
        # and a stalled link drops packets at the socket.
        for fn in sinks:
            try:
                fn(unit, t_master, rec)
            except Exception as e:      # noqa: BLE001
                log.debug("imu sink failed: %s", e)

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
# Getting the inertial data OUT
# ============================================================

# One row per sample, one column per number, in a fixed order. A fixed order
# matters more than it looks: these files are read months later by a script
# nobody has opened since, and a column set that varies with whichever
# channels the sensor happened to be publishing that day is a file that has
# to be re-discovered every time it is read. Columns a unit does not provide
# are present and empty, which is a statement ("this unit has no
# magnetometer"), where a missing column is a question.
IMU_COLUMNS = [
    ("t_s", lambda r: None),                    # filled by the writer
    ("unit", lambda r: None),                   # filled by the writer
    ("quat_w", lambda r: _at(r, "quat", 0)),
    ("quat_x", lambda r: _at(r, "quat", 1)),
    ("quat_y", lambda r: _at(r, "quat", 2)),
    ("quat_z", lambda r: _at(r, "quat", 3)),
    ("roll_deg", lambda r: _at(r, "euler_deg", 0)),
    ("pitch_deg", lambda r: _at(r, "euler_deg", 1)),
    ("yaw_deg", lambda r: _at(r, "euler_deg", 2)),
    ("gyro_x_rad_s", lambda r: _at(r, "gyro", 0)),
    ("gyro_y_rad_s", lambda r: _at(r, "gyro", 1)),
    ("gyro_z_rad_s", lambda r: _at(r, "gyro", 2)),
    ("accel_x_m_s2", lambda r: _at(r, "accel", 0)),
    ("accel_y_m_s2", lambda r: _at(r, "accel", 1)),
    ("accel_z_m_s2", lambda r: _at(r, "accel", 2)),
    ("lin_accel_x_m_s2", lambda r: _at(r, "linear_accel", 0)),
    ("lin_accel_y_m_s2", lambda r: _at(r, "linear_accel", 1)),
    ("lin_accel_z_m_s2", lambda r: _at(r, "linear_accel", 2)),
    ("mag_x", lambda r: _at(r, "mag", 0)),
    ("mag_y", lambda r: _at(r, "mag", 1)),
    ("mag_z", lambda r: _at(r, "mag", 2)),
    ("accel_norm_m_s2", lambda r: r.get("accel_norm")),
    ("gyro_norm_deg_s", lambda r: r.get("gyro_norm_deg_s")),
    ("tilt_roll_deg", lambda r: _at(r, "tilt_deg", 0)),
    ("tilt_pitch_deg", lambda r: _at(r, "tilt_deg", 1)),
    ("quat_source", lambda r: r.get("quat_source")),
    ("still", lambda r: int(bool(r.get("still"))) if "still" in r else None),
    ("rate_hz", lambda r: r.get("rate_hz")),
    ("bias_x_deg_s", lambda r: _at(r, "gyro_bias_deg_s", 0)),
    ("bias_y_deg_s", lambda r: _at(r, "gyro_bias_deg_s", 1)),
    ("bias_z_deg_s", lambda r: _at(r, "gyro_bias_deg_s", 2)),
    ("filter_disagreement_deg", lambda r: r.get("filter_disagreement_deg")),
    ("device_vs_estimate_tilt_deg", lambda r: r.get("device_vs_estimate_tilt_deg")),
    ("temp_c", lambda r: r.get("temp_c")),
    ("pressure_hpa", lambda r: r.get("pressure_hpa")),
    ("humidity_pct", lambda r: r.get("humidity_pct")),
]

IMU_HEADER = [c[0] for c in IMU_COLUMNS]


def _at(rec, key, i):
    v = rec.get(key)
    try:
        return v[i]
    except Exception:
        return None


def _cell(v) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def imu_row(unit: str, t: float, rec: dict) -> list[str]:
    row = [f"{t:.6f}", unit]
    for name, get in IMU_COLUMNS[2:]:
        row.append(_cell(get(rec)))
    return row


class ImuLogger:
    """
    Writes every inertial sample to a CSV as it arrives, for as long as it is
    running.

    This exists because the hub's ring buffer holds 4096 samples per unit --
    forty seconds at 100 Hz, twelve at 350 -- and "export the IMU data" means
    the whole capture, not the tail of it. Subscribing to the hub rather than
    polling it is the whole point: a poller at any rate loses whatever arrived
    between two polls, and a gap in an inertial record is not recoverable and
    not always visible.

    Buffered and flushed on a timer rather than per row: at 350 Hz a flush per
    sample is 350 syscalls a second competing with the camera for the same
    disk, and an unflushed buffer costs at most one second of data if the
    process is killed, against a capture that stutters the whole time it runs.
    """

    FLUSH_EVERY_S = 1.0

    def __init__(self):
        self._lock = threading.Lock()
        self._fh = None
        self.path: Path | None = None
        self.units: set[str] | None = None
        self.rows = 0
        self.dropped = 0
        self.started_at: float | None = None
        self._last_flush = 0.0
        self.error = ""

    def running(self) -> bool:
        return self._fh is not None

    def start(self, path=None, units=None) -> dict:
        self.stop()
        folder = Path("imu_logs")
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except Exception as e:      # noqa: BLE001
            return {"ok": False, "error": f"could not create {folder}: {e}"}
        name = path or f"imu_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        target = Path(name)
        if not target.is_absolute() and target.parent == Path("."):
            target = folder / target
        try:
            fh = open(target, "w", newline="", encoding="utf-8")
            fh.write(",".join(IMU_HEADER) + "\n")
        except Exception as e:      # noqa: BLE001
            return {"ok": False, "error": f"could not open {target}: {e}"}
        with self._lock:
            self._fh = fh
            self.path = target
            self.units = set(units) if units else None
            self.rows = 0
            self.dropped = 0
            self.started_at = time.monotonic()
            self._last_flush = self.started_at
            self.error = ""
        HUB.subscribe(self._on_sample)
        log.info("imu logging to %s", target)
        return {"ok": True, **self.status()}

    def _on_sample(self, unit: str, t: float, rec: dict) -> None:
        with self._lock:
            fh = self._fh
            if fh is None:
                return
            if self.units is not None and unit not in self.units:
                return
            try:
                fh.write(",".join(imu_row(unit, t, rec)) + "\n")
                self.rows += 1
            except Exception as e:      # noqa: BLE001
                self.dropped += 1
                self.error = str(e)
                return
            now = time.monotonic()
            if now - self._last_flush >= self.FLUSH_EVERY_S:
                self._last_flush = now
                try:
                    fh.flush()
                except Exception:
                    pass

    def stop(self) -> dict:
        HUB.unsubscribe(self._on_sample)
        with self._lock:
            fh, path, rows = self._fh, self.path, self.rows
            dur = (time.monotonic() - self.started_at) if self.started_at else 0.0
            self._fh = None
        if fh is None:
            return {"ok": False, "error": "nothing was being logged",
                    "running": False}
        try:
            fh.flush()
            fh.close()
        except Exception:
            pass
        size = path.stat().st_size if path and path.exists() else 0
        log.info("imu log closed: %s rows=%d", path, rows)
        return {"ok": True, "running": False, "path": str(path.resolve()),
                "rows": rows, "bytes": size, "seconds": round(dur, 1),
                "dropped": self.dropped,
                "note": (f"{rows} samples written to {path}. This is the "
                         "complete record for the period it was running, not "
                         "a sample of it.")}

    def status(self) -> dict:
        with self._lock:
            dur = (time.monotonic() - self.started_at) if self.started_at else 0.0
            return {"running": self._fh is not None,
                    "path": str(self.path.resolve()) if self.path else None,
                    "rows": self.rows, "dropped": self.dropped,
                    "seconds": round(dur, 1),
                    "units": sorted(self.units) if self.units else "all",
                    "error": self.error}


LOGGER = ImuLogger()


def export_ring(units=None, path=None) -> dict:
    """
    Everything still in memory, written out now.

    The companion to the logger, for the operator who has just seen something
    happen and wants that, without having remembered to start a log first. It
    is bounded by the ring -- a few thousand samples per unit -- and it says
    so, because an export that silently holds the last forty seconds of a ten
    minute run is a trap.
    """
    names = list(units) if units else sorted(HUB._rings)     # noqa: SLF001
    rows = []
    for unit in names:
        for t, rec in HUB.ring(unit):
            rows.append((t, unit, rec))
    if not rows:
        return {"ok": False, "error": "no inertial samples are in memory yet "
                                      "— connect a sensor first"}
    rows.sort(key=lambda r: r[0])
    folder = Path("imu_logs")
    try:
        folder.mkdir(parents=True, exist_ok=True)
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not create {folder}: {e}"}
    target = Path(path) if path else folder / f"imu_snapshot_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    if not target.is_absolute() and target.parent == Path("."):
        target = folder / target
    lines = [",".join(IMU_HEADER)]
    for t, unit, rec in rows:
        lines.append(",".join(imu_row(unit, t, rec)))
    text = "\n".join(lines) + "\n"
    try:
        target.write_text(text, encoding="utf-8")
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not write {target}: {e}"}
    span = rows[-1][0] - rows[0][0]
    return {"ok": True, "path": str(target.resolve()), "rows": len(rows),
            "units": names, "seconds": round(span, 2),
            "csv": text if len(text) < 4_000_000 else None,
            "note": (f"{len(rows)} samples covering {span:.1f} s — everything "
                     "held in memory. Memory holds a few thousand samples per "
                     "sensor, so for a longer capture start the continuous "
                     "log instead.")}



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
    One managed inertial link, of any transport imu_link supports.

    The name is historical — it began as a UDP-only FusionHub listener — but a
    "FusionHub bridge" that can only do UDP JSON is the thing that failed in
    practice, so what it actually holds now is a transport plus its config, and
    the transport is chosen at run time from the console.

    Restart semantics matter: `start()` stops any existing link first. Two
    listeners bound to one port is not an error either of them reports, and the
    symptom — half the packets, silently — looks exactly like a flaky sensor.
    """

    def __init__(self, port: int = 5005, unit: str = "ind0"):
        self.unit = unit
        self.kind = "udp-listen"
        self.config = {"port": int(port)}
        self.gyro_units = "auto"
        self.link = None
        self.error = "" if _HAS_LINK else _LINK_ERR

    # `port` stays a property so existing callers (start_sources, the CLI
    # flag) keep working against the new config dict.
    @property
    def port(self) -> int:
        return int(self.config.get("port", 5005))

    @port.setter
    def port(self, value) -> None:
        self.config["port"] = int(value)

    def start(self, kind: str | None = None, config: dict | None = None,
              gyro_units: str | None = None) -> bool:
        if not _HAS_LINK:
            self.error = _LINK_ERR or "imu_link not importable"
            return False
        self.stop()
        if kind:
            self.kind = kind
        if config:
            self.config = dict(config)
        if gyro_units:
            self.gyro_units = gyro_units
        try:
            self.link = imu_link.make_link(
                self.kind, self.unit, gyro_units=self.gyro_units,
                on_sample=lambda t_src, rec: HUB.push(
                    self.unit, MASTER.to_master(self.unit, t_src), rec),
                **self.config)
        except Exception as e:      # noqa: BLE001
            self.error = str(e)
            log.warning("inertial link %s could not be built: %s", self.kind, e)
            return False
        res = self.link.start()
        self.error = res.get("error", "")
        if res.get("ok"):
            log.info("inertial link up: unit=%s transport=%s %s",
                     self.unit, self.kind, self.config)
        else:
            log.warning("inertial link failed: %s", self.error)
        return bool(res.get("ok"))

    def stop(self) -> None:
        if self.link:
            try:
                self.link.stop()
            except Exception:
                pass
            self.link = None

    def status(self) -> dict:
        base = {"unit": self.unit, "kind": self.kind, "config": dict(self.config),
                "gyro_units": self.gyro_units, "error": self.error,
                "port": self.port, "transport_available": _HAS_LINK}
        if self.link:
            base.update(self.link.health())
        else:
            base.update({"running": False, "samples": 0, "rate_hz": 0.0})
        return base


class LinkRegistry:
    """
    Every inertial link the console has configured, keyed by unit id.

    A registry rather than one hardcoded FusionHub slot, because the benchmark
    explicitly wants three tiers at once — the industrial unit, an accessible
    consumer module, and the camera's own part — and because the next sensor
    to arrive should need a config entry, not a code change.
    """

    def __init__(self):
        self.links: dict[str, FusionHubBridge] = {}

    def get(self, unit: str) -> FusionHubBridge:
        link = self.links.get(unit)
        if link is None:
            link = self.links[unit] = FusionHubBridge(unit=unit)
        return link

    def start(self, unit: str, kind: str, config: dict,
              gyro_units: str = "auto") -> dict:
        link = self.get(unit)
        ok = link.start(kind, config, gyro_units)
        return {"ok": ok, "unit": unit, **link.status()}

    def stop(self, unit: str) -> dict:
        link = self.links.get(unit)
        if link is None:
            return {"ok": False, "error": f"no link configured for {unit!r}"}
        link.stop()
        return {"ok": True, "unit": unit, **link.status()}

    def stop_all(self) -> None:
        for link in self.links.values():
            link.stop()

    def status(self) -> dict:
        return {u: l.status() for u, l in self.links.items()}


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
            # Every registered modality goes into the same row. A sensor that
            # arrives next month is recorded from the day it is attached with
            # no change here — which is the point of the registry.
            extra = {}
            if _HAS_SENSORS:
                try:
                    extra = sensor_hub.HUB.snapshot()
                except Exception:
                    extra = {}
            sample = Sample(
                t=now,
                q=list(q) if q else None,
                tcp_pos=list(tcp[:3]) if tcp else None,
                tcp_rot=list(tcp[3:6]) if tcp and len(tcp) >= 6 else None,
                imu=HUB.snapshot(),
                sensors=extra,
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
LINKS = LinkRegistry()
FUSIONHUB = LINKS.get("ind0")


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
        "links": LINKS.status(),
        "attitude": HUB.tracker_status(),
        "recorder": RECORDER.status(),
        "imu_log": LOGGER.status(),
        "bench_available": _HAS_BENCH,
        "transports": sorted(imu_link.TRANSPORTS) if _HAS_LINK else [],
        "transport_error": "" if _HAS_LINK else _LINK_ERR,
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
    if mtype == "sensors_report":
        if not _HAS_SENSORS:
            return {"type": "sensors_report_res", "ok": False,
                    "error": "sensor_hub not importable"}
        return {"type": "sensors_report_res", "ok": True,
                **sensor_hub.HUB.report()}

    if mtype == "bench_tap":
        return {"type": "bench_tap_res", **verify_tap(data.get("window_s", 5.0))}
    # ---- inertial link management -------------------------------------
    if mtype == "imu_transports":
        return {"type": "imu_transports_res",
                "available": _HAS_LINK, "error": "" if _HAS_LINK else _LINK_ERR,
                "transports": sorted(imu_link.TRANSPORTS) if _HAS_LINK else [],
                "serial": imu_link.list_serial_ports() if _HAS_LINK
                else {"available": False, "ports": []},
                "links": LINKS.status()}
    if mtype == "imu_discover":
        if not _HAS_LINK:
            return {"type": "imu_discover_res", "ok": False, "error": _LINK_ERR}
        res = imu_link.discover_udp(data.get("ports"),
                                    float(data.get("seconds", 6.0)))
        return {"type": "imu_discover_res", "ok": True, **res}
    if mtype == "imu_tcp_probe":
        if not _HAS_LINK:
            return {"type": "imu_tcp_probe_res", "ok": False, "error": _LINK_ERR}
        return {"type": "imu_tcp_probe_res", "ok": True,
                **imu_link.probe_tcp(data.get("host", "127.0.0.1"),
                                     data.get("ports"))}
    if mtype == "imu_link_start":
        if not _HAS_LINK:
            return {"type": "imu_link_res", "ok": False, "error": _LINK_ERR}
        return {"type": "imu_link_res", "cmd": "start",
                **LINKS.start(data.get("unit", "ind0"),
                              data.get("kind", "udp-listen"),
                              data.get("config") or {},
                              data.get("gyro_units", "auto"))}
    if mtype == "imu_link_stop":
        return {"type": "imu_link_res", "cmd": "stop",
                **LINKS.stop(data.get("unit", "ind0"))}
    if mtype == "imu_sniff":
        # The raw bytes of the most recent packet on a link, classified. This
        # is what turns "no data" from a guess into a reading.
        unit = data.get("unit", "ind0")
        link = LINKS.links.get(unit)
        if link is None or link.link is None:
            return {"type": "imu_sniff_res", "ok": False,
                    "error": f"no link running for {unit!r}"}
        raw = link.link.last_raw
        if not raw:
            return {"type": "imu_sniff_res", "ok": False,
                    "error": "the link is up but nothing has arrived on it yet"}
        return {"type": "imu_sniff_res", "ok": True, "unit": unit,
                **imu_link.sniff(raw)}
    if mtype == "imu_zero":
        # Re-seed one unit's attitude estimate and clear its learned gyro bias.
        # Done with the unit held still; the console says so.
        unit = data.get("unit", "ind0")
        ok = HUB.reset_tracker(unit)
        return {"type": "imu_zero_res", "ok": ok, "unit": unit,
                "note": "hold the unit still for two seconds while the bias "
                        "re-learns" if ok else "attitude tracking unavailable"}
    if mtype == "imu_d435i":
        want = bool(data.get("on", True))
        if want:
            D435I.accel_hz = int(data.get("accel_hz", D435I.accel_hz))
            D435I.gyro_hz = int(data.get("gyro_hz", D435I.gyro_hz))
            ok = D435I.start()
        else:
            D435I.stop()
            ok = True
        return {"type": "imu_d435i_res", "ok": ok, **D435I.status()}

    # ---- getting the data out -----------------------------------------
    if mtype == "imu_export":
        return {"type": "imu_export_res",
                **export_ring(data.get("units"), data.get("path"))}
    if mtype == "imu_log_start":
        return {"type": "imu_log_res", "cmd": "start",
                **LOGGER.start(data.get("path"), data.get("units"))}
    if mtype == "imu_log_stop":
        return {"type": "imu_log_res", "cmd": "stop", **LOGGER.stop()}
    if mtype == "imu_log_status":
        return {"type": "imu_log_res", "cmd": "status", "ok": True,
                **LOGGER.status()}

    if mtype == "bench_offset":
        MASTER.set_offset(data.get("channel", ""), data.get("offset_s", 0.0),
                          data.get("residual_s", 0.0))
        return {"type": "bench_offset_res", "ok": True, **MASTER.status()}
    return None
