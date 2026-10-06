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
    import ur_telemetry
    _HAS_URT = True
except Exception:       # noqa: BLE001
    ur_telemetry = None
    _HAS_URT = False

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

class SourceClock:
    """
    What one channel's own timestamps are worth, decided from the stream.

    A source clock is a claim, not a fact, and this rig has a channel whose
    claim is false: an LPMS-B2 through FusionHub publishes `timecode` and
    `timestamp` fields that are BIT-IDENTICAL on every packet -- verified
    against FusionHub's own MCAP recording, 4280 messages, one distinct
    timestamp between them. Taken at face value that produced an export whose
    1425 rows all carried the same instant, an integration step of exactly
    zero for every orientation filter, and a channel age computed against a
    clock that never moved.

    So each channel is classified as it arrives:

      ok        its timestamps advance by plausible amounts
      stalled   they do not advance at all
      jumpy     they advance by implausible amounts
      unfitted  they advance fine, but no measured offset maps them onto the
                master clock, so their zero is somewhere else entirely

    Only `ok` AND fitted earns a channel the right to place its own samples on
    the master timeline. Everything else is placed by arrival time at the host,
    which is jittery by a packet or two but monotonic, shared with every other
    channel, and never a lie.

    The classification is made once, from the first few hundred samples, and
    then held. Switching timebases part-way through a recording would put two
    incompatible timelines in one file, which is worse than either.
    """

    DECIDE_AFTER = 200
    MIN_STEP = 1e-5             # 100 kHz; below this it is not advancing
    MAX_STEP = 2.0              # a bigger step than this is a fault, not a gap

    def __init__(self, channel: str):
        self.channel = channel
        self.verdict = ""               # "" until decided
        self.n = 0
        self.n_stalled = 0
        self.n_jumpy = 0
        self.n_ok = 0
        self.fitted = False
        self._last: float | None = None

    def observe(self, t_src: float) -> None:
        if self._last is not None:
            step = t_src - self._last
            if abs(step) < self.MIN_STEP:
                self.n_stalled += 1
            elif step < 0 or step > self.MAX_STEP:
                self.n_jumpy += 1
            else:
                self.n_ok += 1
        self._last = t_src
        self.n += 1
        if self.verdict or self.n < self.DECIDE_AFTER:
            return
        total = max(1, self.n_ok + self.n_stalled + self.n_jumpy)
        if self.n_stalled / total > 0.5:
            self.verdict = "stalled"
        elif self.n_jumpy / total > 0.1:
            self.verdict = "jumpy"
        else:
            self.verdict = "ok"

    def usable(self) -> bool:
        return self.verdict == "ok" and self.fitted

    def why(self) -> str:
        if self.usable():
            return "the channel's own clock, mapped onto the master by a measured offset"
        if not self.verdict:
            return "arrival time at the host while the channel's own clock is still being assessed"
        if self.verdict == "stalled":
            return "arrival time at the host -- the channel's own timestamp does not advance"
        if self.verdict == "jumpy":
            return (f"arrival time at the host -- the channel's own timestamp "
                    f"stepped implausibly on {self.n_jumpy} of {self.n} packets")
        return ("arrival time at the host -- the channel's clock advances, but no "
                "offset onto the master clock has been measured for it yet")

    def status(self) -> dict:
        return {"clock": self.verdict or "assessing", "fitted": self.fitted,
                "timebase": "own" if self.usable() else "arrival",
                "why": self.why(), "samples": self.n,
                "stalled": self.n_stalled, "jumpy": self.n_jumpy}


class TimeMaster:
    """
    The host's monotonic clock stands in for the Teensy until the Teensy is
    wired in Phase 1. The interface does not change when it is: callers ask
    for `now()` and register per-channel offsets, and swapping the reference
    is one method.

    Using a MONOTONIC clock rather than wall time is not a detail. Wall time
    can step backwards mid-run under NTP correction, and a run containing a
    backwards time step is silently unusable.

    `to_master` used to be `t_src + offset`, with the offset defaulting to
    zero. That default is the mistake this class exists to avoid: it takes a
    number from someone else's clock, adds nothing to it, and returns it as
    master time. Two things then go wrong at once, and both were live. An
    epoch-scale sensor timestamp came back as master time, so a channel's age
    -- master now, minus that -- read minus 1.79 billion seconds and no
    staleness check could ever fire. And a sensor whose clock does not advance
    at all put every sample in a recording at the same instant.

    A channel's own clock is now used only once it has been shown to advance
    AND an offset onto the master has actually been fitted from a shared
    event. Until then, arrival time at the host -- which is what `now()`
    returns -- stands in, and which channels are on which basis is reported
    rather than assumed.
    """

    def __init__(self):
        self._t0 = time.perf_counter()
        self._wall0 = time.time()
        self.offsets: dict[str, float] = {}
        self.residuals: dict[str, float] = {}
        self.clocks: dict[str, SourceClock] = {}
        self.source = "host-monotonic"
        self._lock = threading.Lock()

    def now(self) -> float:
        # perf_counter, NOT monotonic. On Windows time.monotonic() is
        # GetTickCount64, whose resolution is the 15.6 ms scheduler tick, and
        # this is the clock every recorded sample is stamped with. A smoke-test
        # run showed it plainly: every interval in the file was a multiple of
        # 15.6 ms, 108 of 303 samples shared a timestamp with a neighbour, a
        # loop asked for 125 Hz delivered 43, and the file reported time
        # standing still between consecutive samples. perf_counter is
        # QueryPerformanceCounter, sub-microsecond, and monotonic in practice
        # on every platform this runs on. A 190 Hz sensor has a 5.3 ms period,
        # so a 15.6 ms clock cannot even order its samples correctly, let alone
        # measure a sim-to-real gap with them.
        return time.perf_counter() - self._t0

    def wall_of(self, t: float) -> float:
        return self._wall0 + t

    def set_offset(self, channel: str, offset_s: float, residual_s: float = 0.0) -> None:
        self.offsets[channel] = float(offset_s)
        self.residuals[channel] = float(residual_s)
        with self._lock:
            ch = self.clocks.get(channel)
            if ch is None:
                ch = self.clocks[channel] = SourceClock(channel)
            ch.fitted = True

    def reset_channel(self, channel: str, offset_s: float | None = None) -> None:
        """
        Forget everything learned about a channel's clock -- a new link on it
        is a new clock -- and optionally declare its offset straight away.

        A declared offset is for a link whose timestamps are ALREADY on this
        PC's clock (the OpenZen link maps the sensor clock onto perf_counter
        itself); it is not a way to skip measuring an unknown one.
        """
        with self._lock:
            self.clocks[channel] = SourceClock(channel)
        self.offsets.pop(channel, None)
        self.residuals.pop(channel, None)
        if offset_s is not None:
            self.set_offset(channel, offset_s)

    def clock_of(self, channel: str) -> SourceClock:
        with self._lock:
            ch = self.clocks.get(channel)
            if ch is None:
                ch = self.clocks[channel] = SourceClock(channel)
            return ch

    def to_master(self, channel: str, t_src: float) -> float:
        """
        Place one sample on the master timeline.

        Returns arrival time unless this channel has earned the right to place
        its own samples -- see SourceClock. Callers do not need to know which
        happened; `clock_report()` says, for the run manifest and the console.
        """
        ch = self.clock_of(channel)
        try:
            ch.observe(float(t_src))
        except (TypeError, ValueError):
            return self.now()
        if ch.usable():
            return float(t_src) + self.offsets.get(channel, 0.0)
        return self.now()

    def measured_channels(self) -> list[str]:
        return sorted(self.offsets)

    def arrival_timed_channels(self) -> list[str]:
        with self._lock:
            return sorted(c for c, ch in self.clocks.items() if not ch.usable())

    def clock_report(self) -> dict:
        with self._lock:
            return {c: ch.status() for c, ch in self.clocks.items()}

    def status(self) -> dict:
        return {
            "source": self.source,
            "t": round(self.now(), 4),
            "offsets_ms": {k: round(v * 1000.0, 3) for k, v in self.offsets.items()},
            "residuals_ms": {k: round(v * 1000.0, 3) for k, v in self.residuals.items()},
            "worst_residual_ms": round(max(self.residuals.values(), default=0.0) * 1000.0, 3),
            "channels": self.clock_report(),
            "arrival_timed": self.arrival_timed_channels(),
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
                # The six-face correction for THIS unit, if one has been made.
                # Loaded once, when the unit is first seen, rather than on
                # every sample: a calibration made mid-run must not change the
                # scale half way through a recording.
                try:
                    import accel_cal
                    cal = accel_cal.load(accel_cal.DEFAULT_PATH
                                         if unit == "ind0"
                                         else f"accel_cal_{unit}.json")
                    if cal.get("ok"):
                        tr.accel_cal = cal
                        log.info("accelerometer calibration loaded for %s: %s",
                                 unit, accel_cal.describe(cal))
                except Exception as e:      # noqa: BLE001
                    log.debug("no accelerometer calibration for %s: %s", unit, e)
            return tr

    # Which way round each sensor publishes its quaternion, per connection
    # kind, as measured in earlier sessions. It is a property of the sensor's
    # firmware and the software in between (FusionHub and OpenZen differ),
    # not of how it is mounted -- so it is remembered, and the operator does
    # not have to move the arm to re-learn it after every restart. It is still
    # re-checked from the data each session.
    CONV_PATH = Path("calib") / "quat_convention.json"

    def _conv_book(self) -> dict:
        try:
            return json.loads(self.CONV_PATH.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def set_source(self, unit: str, kind: str) -> None:
        """
        A unit is about to be read over `kind`. A different connection may
        publish the quaternion the other way round, so the convention starts
        again -- from what was measured last time over that same connection,
        if anything was.
        """
        if AttitudeTracker is None:
            return
        tr = self.tracker(unit)
        if tr is None or getattr(tr, "_source", None) == kind:
            return
        from sonair_benchmark.attitude import QuatConvention
        tr.convention = QuatConvention("auto")
        tr._source = kind
        tr._conv_saved = False
        rec = (self._conv_book().get(unit) or {}).get(kind) or {}
        if rec.get("convention"):
            tr.convention.remember(rec["convention"], rec.get("decided_utc", ""))
            tr._conv_saved = True
            log.info("%s over %s: quaternion convention %s, remembered",
                     unit, kind, rec["convention"])

    def _maybe_save_convention(self, unit: str, tr) -> None:
        conv = tr.convention
        kind = getattr(tr, "_source", None)
        if not kind or not conv.decided or conv.verifying or conv.pinned:
            return
        # Never learned from the simulated cell: its IMU arrives over UDP like
        # FusionHub's, and a simulator's convention remembered for a real
        # sensor would be applied to it at the next start.
        if _sim_prefix():
            return
        if getattr(tr, "_conv_saved", False) and not conv.revised:
            return
        tr._conv_saved = True
        conv.revised = False
        book = self._conv_book()
        book.setdefault(unit, {})[kind] = {
            "convention": conv.decided,
            "residual_deg": round(conv.residual_deg, 3),
            "decided_utc": time.strftime("%Y-%m-%d %H:%M", time.gmtime()) + " UTC"}
        try:
            self.CONV_PATH.parent.mkdir(parents=True, exist_ok=True)
            self.CONV_PATH.write_text(json.dumps(book, indent=2), encoding="utf-8")
        except OSError as e:
            log.warning("could not save the quaternion convention: %s", e)

    def reset_tracker(self, unit: str) -> bool:
        if AttitudeTracker is None:
            return False
        with self._tlock:
            old = self._trackers.get(unit)
            self._trackers[unit] = AttitudeTracker(unit)
        src = getattr(old, "_source", None)
        if src:
            self.set_source(unit, src)
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
                self._maybe_save_convention(unit, tr)
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
        """
        The per-unit block that goes into one recorded Sample.

        Each unit's block carries `_age_s`: how old its latest reading was
        when the row was written. A sensor that drops out does not leave a
        hole in a run file -- its last reading is written again and again --
        so without this a dropout is invisible in the file. With it, the
        read-back after every run can see it and reject the run.
        """
        now = MASTER.now()
        with self._lock:
            return {u: {**rec, "_age_s": [round(now - t, 4)]}
                    for u, (t, rec) in self._latest.items()}

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


# Is the "robot" SONAIR's simulated cell (sim_cell.py)? Set by the bridge
# from the robot link, which reads the cell's own announcement in the RTDE
# handshake. Everything recorded while it is True is labelled simulated and
# written apart from the real data, so the two can never be mixed.
SIMULATED = lambda: False      # noqa: E731
SIM_WORDS = "SIMULATED CELL (URSim controller + MuJoCo plant)"


def _sim_prefix() -> str:
    try:
        return "simcell_" if SIMULATED() else ""
    except Exception:       # noqa: BLE001
        return ""


def _cell(v) -> str:
    """
    One CSV cell. Floats keep their precision.

    This was `%.6g`, and six significant digits silently destroyed every
    time column: a host epoch time (1.79e9 s) kept a resolution of 1e3 s,
    the controller's timestamp past 10 000 s kept 0.1 s, and the log's own
    t_s past 1000 s kept 0.01 s -- coarser than the 8 ms packet period it was
    meant to order. Large magnitudes (times) are written to the microsecond;
    everything else to nine significant digits, well past any sensor here.
    """
    if v is None:
        return ""
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            return ""
        if abs(v) >= 1e4:
            return f"{v:.6f}"
        return f"{v:.9g}"
    return str(v)


def imu_row(unit: str, t: float, rec: dict) -> list[str]:
    row = [f"{t:.6f}", unit]
    for name, get in IMU_COLUMNS[2:]:
        row.append(_cell(get(rec)))
    return row


# ---------------------------------------------------------------------------
# the robot's own record
# ---------------------------------------------------------------------------

# Joint names in UR order, used to name the six columns a VECTOR6D becomes.
# "j1..j6" would be shorter and would need a lookup table every time the file
# is read; a column called `joint_temperatures_elbow_c` does not.
UR_JOINT_NAMES = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
_POSE_AXES = ("x", "y", "z", "rx", "ry", "rz")
_VEC3_AXES = ("x", "y", "z")

# Fields whose six components are a TOOL POSE (x y z rx ry rz), not six joints.
_POSE_FIELDS = {"actual_TCP_pose", "target_TCP_pose", "actual_TCP_speed",
                "actual_TCP_force"}


def ur_columns(recipe=None) -> list:
    """
    The flat column set for the robot log, derived from the RTDE recipe.

    Derived rather than written out, so a field added to the recipe appears in
    the file without a second edit -- and, more importantly, so the column set
    is the SAME on every controller. A field this controller does not provide
    is a present, empty column, which states "this robot does not report joint
    voltages"; a missing column only raises the question.
    """
    if recipe is None:
        recipe = ur_telemetry.OUTPUT_RECIPE if _HAS_URT else []
    cols = ["t_s", "host_time_s", "source"]
    for name, typ in recipe:
        if typ.startswith("VECTOR6"):
            axes = _POSE_AXES if name in _POSE_FIELDS else UR_JOINT_NAMES
            cols += [f"{name}_{a}" for a in axes]
        elif typ == "VECTOR3D":
            cols += [f"{name}_{a}" for a in _VEC3_AXES]
        else:
            cols.append(name)
    # The decoded text of the mode fields. The integers are in the file too --
    # these are for the person reading it, and cost three short strings a row.
    cols += ["robot_mode_text", "safety_mode_text", "runtime_state_text"]
    return cols


def ur_row(t: float, st: dict, recipe=None) -> list:
    if recipe is None:
        recipe = ur_telemetry.OUTPUT_RECIPE if _HAS_URT else []
    out = [_cell(round(float(t), 6)),
           _cell(st.get("_host_time")), _cell(st.get("_source"))]
    for name, typ in recipe:
        v = st.get(name)
        n = 6 if typ.startswith("VECTOR6") else 3 if typ == "VECTOR3D" else 1
        if n == 1:
            out.append(_cell(v))
            continue
        for i in range(n):
            try:
                out.append(_cell(v[i]))
            except Exception:       # noqa: BLE001
                out.append("")
    out += [_cell(st.get("robot_mode_text")), _cell(st.get("safety_mode_text")),
            _cell(st.get("runtime_state_text"))]
    return out


class UrLogger:
    """
    Every packet the robot sends, to a CSV, for as long as it is running.

    The run file records the robot on the benchmark's fixed sample grid and
    holds only the channels the gap is scored on. That is the right shape for
    scoring and the wrong shape for everything else: joint temperatures drift
    over a campaign, currents and torques say what the arm was working against,
    and none of it is in the run file. This is the other half -- the complete
    record, at the controller's own rate, one row per packet, with a column set
    that does not change between controllers.

    It SUBSCRIBES rather than polls, for the same reason the inertial logger
    does: a poller beside a 125 Hz stream sees some packets twice and misses
    others, and the misses are invisible afterwards.
    """

    FLUSH_EVERY_S = 2.0

    def __init__(self, out_dir: str | Path = "ur_logs"):
        self.out_dir = Path(out_dir)
        self._fh = None
        self.path: Path | None = None
        self.rows = 0
        self.dropped = 0
        self.error = ""
        self.started_at = 0.0
        self._t0 = 0.0
        self._last_flush = 0.0
        self._lock = threading.Lock()
        self._service = None

    def _telemetry(self):
        try:
            import ur_bridge_ext
            if ur_bridge_ext.UR.enabled and ur_bridge_ext.UR.telemetry:
                return ur_bridge_ext.UR.telemetry
        except Exception:       # noqa: BLE001
            pass
        return None

    def start(self, path: str | None = None) -> dict:
        with self._lock:
            if self._fh is not None:
                return {"ok": False, "error": "already logging the robot",
                        **self.status()}
        svc = self._telemetry()
        if svc is None:
            return {"ok": False, "error":
                    "the robot link has not been started, so there is nothing "
                    "to log. Connect to the robot first."}
        self.out_dir.mkdir(parents=True, exist_ok=True)
        name = path or f"{_sim_prefix()}ur_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        target = self.out_dir / name
        try:
            fh = target.open("w", encoding="utf-8", newline="")
            fh.write(",".join(ur_columns()) + "\n")
        except Exception as e:      # noqa: BLE001
            return {"ok": False, "error": f"could not open {target}: {e}"}
        with self._lock:
            self._fh = fh
            self.path = target
            self.rows = 0
            self.dropped = 0
            self.started_at = time.perf_counter()
            self._t0 = MASTER.now()
            self._last_flush = self.started_at
            self.error = ""
            self._service = svc
        svc.subscribe(self._on_sample)
        log.info("robot logging to %s", target)
        return {"ok": True, **self.status()}

    def _on_sample(self, st: dict) -> None:
        with self._lock:
            fh = self._fh
            if fh is None:
                return
            try:
                mono = st.get("_mono")
                t = (float(mono) - MASTER._t0) if isinstance(mono, (int, float)) \
                    else MASTER.now()
                fh.write(",".join(ur_row(t, st)) + "\n")
                self.rows += 1
            except Exception as e:      # noqa: BLE001
                self.dropped += 1
                self.error = str(e)
                return
            now = time.perf_counter()
            if now - self._last_flush >= self.FLUSH_EVERY_S:
                self._last_flush = now
                try:
                    fh.flush()
                except Exception:
                    pass

    def stop(self) -> dict:
        svc, self._service = self._service, None
        if svc is not None:
            try:
                svc.unsubscribe(self._on_sample)
            except Exception:       # noqa: BLE001
                pass
        with self._lock:
            fh, path, rows = self._fh, self.path, self.rows
            dur = (time.perf_counter() - self.started_at) if self.started_at else 0.0
            self._fh = None
        if fh is None:
            return {"ok": False, "error": "the robot was not being logged",
                    "running": False}
        try:
            fh.flush()
            fh.close()
        except Exception:
            pass
        size = path.stat().st_size if path and path.exists() else 0
        rate = rows / dur if dur > 0 else 0.0
        log.info("robot log closed: %s rows=%d", path, rows)
        return {"ok": True, "running": False, "path": str(path.resolve()),
                "rows": rows, "bytes": size, "seconds": round(dur, 1),
                "rate_hz": round(rate, 1), "dropped": self.dropped,
                "columns": len(ur_columns())}

    def status(self) -> dict:
        with self._lock:
            dur = (time.perf_counter() - self.started_at) if self.started_at else 0.0
            return {"running": self._fh is not None,
                    "path": str(self.path.resolve()) if self.path else None,
                    "rows": self.rows, "dropped": self.dropped,
                    "seconds": round(dur, 1),
                    "rate_hz": round(self.rows / dur, 1) if dur > 0.5 else 0.0,
                    "columns": len(ur_columns()),
                    "error": self.error}


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
        name = path or f"{_sim_prefix()}imu_{time.strftime('%Y%m%d_%H%M%S')}.csv"
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
UR_LOGGER = UrLogger()


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
        # Backs off, and goes quiet. This used to retry every five seconds for
        # ever, and each attempt enumerates USB devices and tries to start a
        # pipeline -- work the camera driver does while holding the Python
        # interpreter lock, which freezes every thread in the agent for its
        # duration, including the ones reading the robot. The camera's IMU is
        # optional; the robot link is not, so the optional one yields.
        wait, failures = 5.0, 0
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
                failures += 1
                if failures <= 3:
                    log.warning("D435i IMU start failed: %s — retrying in %.0f s",
                                e, wait)
                elif failures == 4:
                    log.warning("D435i IMU still unavailable after %d tries (%s). "
                                "Retrying every 2 minutes, quietly. The usual "
                                "cause is the camera on a USB 2 port, or its "
                                "colour/depth streams already holding it; the "
                                "industrial IMU and the robot are unaffected.",
                                failures, e)
                self._stop.wait(wait)
                wait = min(wait * 2.0, 120.0)
                continue
            wait, failures = 5.0, 0

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
        if kind and kind != self.kind and config is None:
            # A different transport takes different settings; carrying the
            # last one's over (a UDP port into an OpenZen link) fails to build.
            config = {}
        if kind:
            self.kind = kind
        if config is not None:
            self.config = dict(config)
        if gyro_units:
            self.gyro_units = gyro_units
        # A new link is a new clock. The OpenZen link hands over times already
        # on this PC's perf_counter, so its offset onto the master is known
        # exactly; every other transport has to earn one (see SourceClock).
        MASTER.reset_channel(self.unit, -MASTER._t0 if self.kind == "openzen"
                             else None)
        HUB.set_source(self.unit, self.kind)
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
        # `gyro_units_pref` is what the OPERATOR asked for ("auto", or pinned);
        # `gyro_units` is the link's VERDICT and only a live link has one.
        # They were the same key, which meant a stopped link reported
        # "gyro_units": "auto" -- a preference wearing a verdict's name. Every
        # reader downstream has to ask "do we know what this unit's numbers
        # mean?", and "auto" is not an answer to that question.
        base = {"unit": self.unit, "kind": self.kind, "config": dict(self.config),
                "gyro_units_pref": self.gyro_units, "gyro_units": "deciding",
                "error": self.error,
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
        self._skips = 0
        self._worst_gap = 0.0
        self._cost = {"robot": 0.0, "sensors": 0.0, "write": 0.0, "total": 0.0}
        self._cost_sum = {"robot": 0.0, "sensors": 0.0, "write": 0.0}
        self._cost_n = 0
        self.state_fn = None  # set by the bridge: () -> (q, tcp_pose)
        # () -> a telemetry service with subscribe/unsubscribe, or None. When
        # there is one, every robot packet becomes a row (see _on_packet).
        self.packet_source = None
        self._svc = None
        self.mode = "poll"

    def is_recording(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def start(self, *, run_id: str, joint_vel: float, arm_config: str,
              traj_type: str, repeat_idx: int, calib_version: str,
              rate_hz: float = 125.0, carrier_mass_kg: float = 0.0,
              carrier_id: str = "carrier-v1", carrier_com_m=None,
              operator: str = "", notes: str = "",
              allow_no_target: bool = False, experiment: str = "E2") -> dict:
        if not _HAS_BENCH:
            return {"ok": False, "error": "sonair_benchmark package not importable"}
        if self.is_recording():
            return {"ok": False, "error": "already recording; stop the current run first"}

        simulated = bool(_sim_prefix())
        if simulated:
            # A run on the simulated cell is a SIMULATION, whatever job made
            # it: labelled so in its manifest, said so in its notes, and kept
            # in its own folder.
            notes = SIM_WORDS + ("; " + notes if notes else "")
        manifest = RunManifest(
            run_id=run_id, side="sim" if simulated else "real",
            calib_version=calib_version,
            joint_vel=float(joint_vel), arm_config=arm_config,
            traj_type=traj_type, repeat_idx=int(repeat_idx),
            carrier_id=carrier_id, carrier_mass_kg=float(carrier_mass_kg),
            # Where the payload's mass sits, in the tool frame. It is what
            # decides the moment the payload adds, which is the part of it the
            # elbow actually feels, so a mass with no position is only half an
            # answer to the simulator.
            carrier_com_m=tuple(float(v) for v in (carrier_com_m or (0.0, 0.0, 0.0))),
            sample_rate_hz=float(rate_hz),
            started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            operator=operator, notes=notes, experiment=experiment,
        )
        problems = manifest.validate()
        if problems:
            return {"ok": False, "error": "; ".join(problems)}

        # IS THE COMMANDED TRAJECTORY ACTUALLY ARRIVING?
        #
        # Checked here, once, before a run exists -- because the cost of
        # getting this wrong is not one run, it is the campaign. A run file
        # without `target_q` looks complete, opens cleanly, plots correctly
        # and cannot be replayed in a simulator against anything meaningful:
        # the only trajectory in it is the one the robot actually followed,
        # and feeding that to a simulator measures nothing. Nobody finds out
        # until the sim side is built, weeks later, with the arm time already
        # spent.
        warn = ""
        if self.state_fn:
            try:
                probe = self.state_fn() or {}
                if isinstance(probe, tuple):
                    probe = {"q": probe[0], "tcp": probe[1]}
                if not probe.get("target_q"):
                    warn = ("the robot is not reporting its COMMANDED joint "
                            "trajectory (target_q), so these runs cannot be "
                            "replayed in a simulator. Start the robot link on "
                            "the Connect page; if it is already running, the "
                            "controller is answering on the fallback interface "
                            "rather than RTDE.")
                    if not allow_no_target:
                        return {"ok": False, "error": warn,
                                "missing": "target_q"}
            except Exception:
                pass

        path = (self.out_dir / "simcell" if simulated else self.out_dir) / f"{run_id}.jsonl"
        if path.exists():
            # A run recorded again -- rejected, or re-recorded under a revised
            # protocol -- never overwrites the earlier file. It is kept beside
            # it under a name the dataset reader does not pick up.
            stamp = time.strftime('%Y%m%dT%H%M%S')
            old = path.with_name(f"{path.name}.{stamp}.superseded")
            k = 1
            while old.exists():
                k += 1
                old = path.with_name(f"{path.name}.{stamp}-{k}.superseded")
            try:
                path.replace(old)
                log.info("kept the earlier %s as %s", path.name, old.name)
            except OSError as e:
                return {"ok": False, "error": f"could not set aside the earlier "
                        f"{path.name}: {e}"}
        try:
            self._writer = RunWriter(path, manifest)
        except Exception as e:
            return {"ok": False, "error": str(e)}

        self._stop.clear()
        self.current = {"run_id": run_id, "path": str(path),
                        "started": MASTER.now(), "rate_hz": rate_hz, "n": 0}
        svc = None
        if self.packet_source is not None:
            try:
                svc = self.packet_source()
            except Exception:       # noqa: BLE001
                svc = None
        self._n = 0
        self._last_mono = None
        self._skips = 0
        self._worst_gap = 0.0
        # The per-phase costs belong to the polling loop; cleared here so a
        # packet-mode run never reports the previous run's figures.
        self._cost = {"robot": 0.0, "sensors": 0.0, "write": 0.0, "total": 0.0}
        self._cost_sum = {"robot": 0.0, "sensors": 0.0, "write": 0.0}
        self._cost_n = 0
        if svc is not None and hasattr(svc, "subscribe"):
            # EVERY PACKET, NOT THE LATEST ONE WHEN THE TIMER FIRES.
            #
            # Sampling the shared robot state on a 125 Hz host timer looked
            # equivalent and was not. Packets reach this process in bursts (the
            # reader runs in its own process and hands them over in batches),
            # so a timer tick sees the same packet several times and then
            # misses the ones that came and went between ticks. Measured on the
            # first two campaign sessions: 41.6% of rows held a distinct robot
            # state -- about 49 Hz of real data under a 125 Hz label -- with
            # gaps to 1 s. Each packet now becomes exactly one row, stamped with
            # when it arrived at the socket and carrying the controller's own
            # timestamp, which is the clock the controller generated it by.
            self.mode = "packet"
            self._svc = svc
            svc.subscribe(self._on_packet)
            target = self._wait
            args = ()
        else:
            self.mode = "poll"
            target = self._loop
            args = (rate_hz,)
        self._thread = threading.Thread(target=target, args=args,
                                        daemon=True, name=f"bench-rec-{run_id}")
        self._thread.start()
        log.info("recording run %s -> %s", run_id, path)
        return {"ok": True, "run_id": run_id, "path": str(path)}

    def _wait(self) -> None:
        """Packet mode: the rows are written by _on_packet; this only lives
        as long as the recording does, so is_recording() means what it says."""
        while not self._stop.wait(0.2):
            pass

    def _on_packet(self, st: dict) -> None:
        """One RTDE packet, one row."""
        if self._stop.is_set():
            return
        mono = st.get("_mono")
        now_perf = time.perf_counter()
        t = (float(mono) - MASTER._t0) if isinstance(mono, (int, float)) \
            else MASTER.now()
        q, tcp = st.get("actual_q"), st.get("actual_TCP_pose")
        if not q:
            return
        if self._last_mono is not None and isinstance(mono, (int, float)):
            gap = float(mono) - self._last_mono
            if gap > 0.25:
                self._skips += 1
                self._worst_gap = max(self._worst_gap, gap)
        if isinstance(mono, (int, float)):
            self._last_mono = float(mono)

        def seq(v):
            return [float(x) for x in v] if v else None
        aux = {}
        ts = st.get("timestamp")
        if isinstance(ts, (int, float)):
            aux["controller_t"] = float(ts)
        extra = {}
        if _HAS_SENSORS:
            try:
                extra = sensor_hub.HUB.snapshot()
            except Exception:       # noqa: BLE001
                extra = {}
        ss = st.get("speed_scaling")
        sample = Sample(
            t=t, q=seq(q), qd=seq(st.get("actual_qd")),
            tcp_pos=seq(tcp[:3]) if tcp else None,
            tcp_rot=seq(tcp[3:6]) if tcp and len(tcp) >= 6 else None,
            target_q=seq(st.get("target_q")), target_qd=seq(st.get("target_qd")),
            target_moment=seq(st.get("target_moment")),
            speed_scaling=float(ss) if isinstance(ss, (int, float)) else None,
            robot_age_s=round(now_perf - float(mono), 4)
            if isinstance(mono, (int, float)) else None,
            imu=HUB.snapshot(), aux=aux, sensors=extra)
        with self._lock:
            if self._writer:
                try:
                    self._writer.write(sample)
                    self._n += 1
                    if self.current:
                        self.current["n"] = self._n
                except Exception as e:      # noqa: BLE001
                    log.warning("sample write failed: %s", e)

    def _loop(self, rate_hz: float) -> None:
        period = 1.0 / max(1.0, rate_hz)
        next_t = MASTER.now()
        self._skips = 0
        self._worst_gap = 0.0
        self._cost = {"robot": 0.0, "sensors": 0.0, "write": 0.0, "total": 0.0}
        self._cost_sum = {"robot": 0.0, "sensors": 0.0, "write": 0.0}
        self._cost_n = 0
        n = 0
        while not self._stop.is_set():
            now = MASTER.now()
            if now < next_t:
                # Sleep the bulk of the wait, then spin the last millisecond.
                # A bare sleep of the whole remainder overshoots: the OS is
                # free to return late, and on Windows it historically returned
                # a whole 15.6 ms scheduler tick late, which is most of an
                # interval at 125 Hz. Spinning a millisecond costs a little
                # CPU and buys a sample grid that is actually the declared one.
                wait = next_t - now
                if wait > 0.0015:
                    time.sleep(wait - 0.001)
                while MASTER.now() < next_t and not self._stop.is_set():
                    time.sleep(0)       # yield, so the spin never starves a reader
                continue
            next_t += period
            # If we fall far behind (a GC pause, a disk hiccup), resynchronise
            # rather than sprinting to catch up — a burst of samples all
            # stamped microseconds apart is worse than a visible gap.
            #
            # But resynchronising SILENTLY is how a run ends up carrying a
            # manifest that says 125 Hz over a file holding 43 Hz, with a 1.5 s
            # hole in the middle. Phase 4 generates the simulated side at the
            # declared rate, so a declared rate the real side never achieved is
            # a resampling error charged to the sim-to-real gap. Every skip is
            # counted and the worst one measured, and both go in the manifest.
            behind = MASTER.now() - next_t
            if behind > 0.25:
                self._skips += 1
                self._worst_gap = max(self._worst_gap, behind + period)
                next_t = MASTER.now()

            # WHERE THE TIME GOES, measured rather than guessed.
            #
            # A campaign came back at 28 Hz of a declared 125, with gaps of two
            # and a half seconds, and finding out why took forensics on
            # uploaded files: every sensor driver in the registry was being
            # called on every tick, so the fastest loop in the system ran at
            # the speed of its slowest driver. The loop now times its own
            # phases and reports them with the run, so the next time a rate is
            # short the answer is in the summary instead of in an investigation.
            tA = time.perf_counter()
            st = {}
            if self.state_fn:
                try:
                    st = self.state_fn() or {}
                except Exception:
                    st = {}
                # The state source returned a (q, tcp) pair before it returned
                # the commanded channels as well. Accepted here so a partially
                # updated deployment records the measured half rather than
                # nothing at all.
                if isinstance(st, tuple):
                    st = {"q": st[0], "tcp": st[1]}
            tB = time.perf_counter()
            q, tcp = st.get("q"), st.get("tcp")
            # Every registered modality goes into the same row. A sensor that
            # arrives next month is recorded from the day it is attached with
            # no change here — which is the point of the registry.
            extra = {}
            if _HAS_SENSORS:
                try:
                    extra = sensor_hub.HUB.snapshot()
                except Exception:
                    extra = {}
            tC = time.perf_counter()
            sample = Sample(
                t=now,
                q=list(q) if q else None,
                qd=st.get("qd"),
                tcp_pos=list(tcp[:3]) if tcp else None,
                tcp_rot=list(tcp[3:6]) if tcp and len(tcp) >= 6 else None,
                target_q=st.get("target_q"),
                target_qd=st.get("target_qd"),
                target_moment=st.get("target_moment"),
                speed_scaling=st.get("speed_scaling"),
                robot_age_s=st.get("robot_age_s"),
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
            tD = time.perf_counter()
            self._cost["robot"] = max(self._cost["robot"], tB - tA)
            self._cost["sensors"] = max(self._cost["sensors"], tC - tB)
            self._cost["write"] = max(self._cost["write"], tD - tC)
            self._cost["total"] = max(self._cost["total"], tD - tA)
            self._cost_sum["robot"] += tB - tA
            self._cost_sum["sensors"] += tC - tB
            self._cost_sum["write"] += tD - tC
            self._cost_n += 1

    def stop(self) -> dict:
        if not self.is_recording():
            return {"ok": False, "error": "not recording"}
        svc, self._svc = self._svc, None
        if svc is not None:
            try:
                svc.unsubscribe(self._on_packet)
            except Exception:       # noqa: BLE001
                pass
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        with self._lock:
            n = self._writer.n if self._writer else 0
            if self._writer:
                self._writer.close()
            self._writer = None
        cur = self.current or {}
        stopped = MASTER.now()
        span = max(1e-6, stopped - float(cur.get("started") or stopped))
        achieved = n / span
        asked = float(cur.get("rate_hz") or 0.0)
        self.last = {**cur, "n": n, "stopped": stopped, "mode": self.mode,
                     "achieved_rate_hz": round(achieved, 1),
                     "skipped_intervals": int(getattr(self, "_skips", 0)),
                     "worst_gap_s": round(float(getattr(self, "_worst_gap", 0.0)), 3)}
        cn = max(1, int(getattr(self, "_cost_n", 0) or 1))
        cs = getattr(self, "_cost_sum", {}) or {}
        cw = getattr(self, "_cost", {}) or {}
        self.last["cost_ms"] = {
            k: {"mean": round(cs.get(k, 0.0) / cn * 1000, 2),
                "worst": round(cw.get(k, 0.0) * 1000, 2)}
            for k in ("robot", "sensors", "write")}
        if asked and achieved < 0.8 * asked:
            # Said plainly, on the run that is affected, while the operator is
            # still standing at the cell and can do something about it.
            self.last["rate_warning"] = (
                f"This run was asked for {asked:.0f} samples a second and "
                f"managed {achieved:.0f}. The samples it holds are correctly "
                f"timed, so the run is usable — but the simulated side must be "
                f"generated at {achieved:.0f} Hz to match it, or resampled, and "
                f"a resampling this large is error charged to the gap. Close "
                f"the live camera views while recording, or record at "
                f"{achieved:.0f} Hz.")
            # Name the phase that actually cost the time, so the next step is
            # obvious instead of a guess.
            worst = max(self.last["cost_ms"], key=lambda k: self.last["cost_ms"][k]["mean"])
            ms = self.last["cost_ms"][worst]
            self.last["rate_warning"] += (
                f" Most of each tick went on {worst}: {ms['mean']:.1f} ms on "
                f"average, {ms['worst']:.0f} ms at worst, against the "
                f"{1000.0 / asked:.1f} ms a sample is allowed.")
        self.current = None
        log.info("run %s finished: %d samples, %.1f Hz achieved of %.0f asked",
                 self.last.get("run_id"), n, achieved, asked)
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


def unit_report() -> dict:
    """
    One row per inertial unit, merging everything known about it: how fast it
    is arriving (the hub), what its numbers mean (the link's unit verdict),
    how its orientation is being interpreted (the attitude tracker), and what
    its own clock is worth (the time master).

    Three separate objects each hold a piece of this, and pre-flight has to
    answer one question -- "is this unit fit to record?" -- which needs all
    four. Assembling it here rather than in the gate keeps the gate testable
    and keeps the console and the gate reading the same row.
    """
    hub = HUB.status()
    links = LINKS.status() or {}
    trackers = HUB.tracker_status() or {}
    latest = HUB.latest()
    clocks = MASTER.clock_report()
    out = {}
    for u in set(hub) | set(links) | set(trackers):
        row = dict(hub.get(u, {}))
        lk = links.get(u) or {}
        for k in ("gyro_units", "gyro_units_pref", "gyro_units_basis",
                  "gyro_units_revised", "gyro_units_evidence", "gyro_peak_raw",
                  "running", "format"):
            if k in lk:
                row[k] = lk[k]
        if u == D435I.UNIT and D435I.status().get("running"):
            row.setdefault("running", True)
        tr = trackers.get(u) or {}
        for k in ("quat_convention", "quat_convention_basis",
                  "quat_convention_remembered", "quat_convention_confirmed",
                  "quat_gravity_residual_deg", "quat_source", "sensor_clock_ok",
                  "clock_jumps", "accel_calibrated", "accel_scale_spread_pct",
                  "accel_still_median", "accel_still_samples"):
            if k in tr:
                row[k] = tr[k]
        rec = latest.get(u) or {}
        row["quat"] = rec.get("quat")
        row["units_pending"] = bool(rec.get("_units_pending"))
        row["clock"] = clocks.get(u, {})
        out[u] = row
    return out


def status() -> dict:
    """One blob the browser polls to render the acquisition panel."""
    return {
        "clock": MASTER.status(),
        "units": unit_report(),
        "sources": {"d435i": D435I.status(), "fusionhub": FUSIONHUB.status()},
        "links": LINKS.status(),
        "attitude": HUB.tracker_status(),
        "recorder": RECORDER.status(),
        "imu_log": LOGGER.status(),
        "ur_log": UR_LOGGER.status(),
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
            carrier_com_m=data.get("carrier_com_m"),
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
    if mtype == "imu_openzen_list":
        if not _HAS_LINK:
            return {"type": "imu_openzen_list_res", "ok": False, "error": _LINK_ERR}
        return {"type": "imu_openzen_list_res",
                **imu_link.list_openzen(float(data.get("seconds", 12.0)))}
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
