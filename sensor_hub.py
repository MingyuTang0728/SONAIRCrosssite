"""
sensor_hub.py — the socket every future modality plugs into.

SONAIR is a MULTIMODAL benchmark, but "multimodal" so far has meant two
hard-wired channels: an IMU path and a camera path, each with its own message
types, its own status block and its own place in the recorder. Adding eddy
current under that arrangement means touching the bridge, the recorder, the
schema and the console — four places, four chances to get it wrong, and the
work repeats for every sensor after it.

So the channel is described as DATA instead. A driver supplies a descriptor
and a `read()`; everything else — appearing in the console, being sampled into
the run file, being counted in the status block, being checked for staleness —
follows from the descriptor with no further code.

The descriptor carries two fields that are not there for tidiness:

  `simulatable`   whether Isaac can render this modality faithfully. Sam's
                  selection rule for the benchmark is that a modality must be
                  BOTH cheap to ground-truth on real hardware AND faithful to
                  simulate. Orientation, angular rate, acceleration and
                  position pass. Eddy current, ultrasound, thermography and
                  Raman do not — they are recorded here as application-domain
                  evidence, and they must never be scored as benchmark
                  channels. Writing that into the descriptor means the
                  distinction survives the person who knows it.

  `gt_cost`       how expensive ground truth is for this channel. It is the
                  other half of the same rule and it is what stops a campaign
                  planning around a modality nobody can label.

A channel that is DECLARED but has no driver yet is a first-class state, not
an absence. It shows in the console greyed out with its expected rate and
units, so the wiring, the schema and the screen are all in place the day the
hardware arrives — which is the day you least want to be writing integration
code.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, asdict
from typing import Callable


# The four modality classes the benchmark distinguishes. The wording is Sam's
# selection rule, kept where the code can see it.
SIMULATABLE = {
    "orientation":  True,
    "angular_rate": True,
    "acceleration": True,
    "position":     True,
    "force":        True,        # contact force is renderable, with effort
    "vision_rgb":   True,
    "vision_depth": True,
    "eddy_current": False,
    "ultrasound":   False,
    "thermography": False,
    "raman":        False,
    "acoustic":     False,
}

GT_COST = {
    "orientation":  "cheap — a two-axis tilt fixture and a still unit",
    "angular_rate": "cheap — a rate table, or integration against orientation",
    "acceleration": "cheap — gravity, at known tilts",
    "position":     "cheap — the robot's own forward kinematics",
    "force":        "moderate — needs a reference load cell",
    "vision_rgb":   "moderate — needs labelled imagery",
    "vision_depth": "moderate — needs a measured artefact",
    "eddy_current": "expensive — needs reference defects, destructively verified",
    "ultrasound":   "expensive — needs reference defects and a coupling procedure",
    "thermography": "expensive — needs a controlled thermal excitation",
    "raman":        "expensive — needs reference spectra per material",
    "acoustic":     "expensive — needs a characterised acoustic environment",
}


@dataclass
class Channel:
    """One measurement stream, present or merely planned."""

    id: str
    label: str
    modality: str                       # a key of SIMULATABLE
    units: str = ""
    rate_hz: float = 0.0                # nominal
    frame: str = "tcp"                  # tcp | base | camera | world | none
    vendor: str = ""
    transport: str = ""
    role: str = "application"           # "benchmark" | "application"
    status: str = "declared"            # declared | present | streaming | failed
    detail: str = ""
    fields: list = field(default_factory=list)
    reader: Callable[[], dict] | None = None
    _last: dict = field(default_factory=dict, repr=False)
    _t_last: float = 0.0
    _n: int = 0
    _err: str = ""

    def descriptor(self) -> dict:
        d = {k: v for k, v in asdict(self).items()
             if not k.startswith("_") and k != "reader"}
        d["simulatable"] = SIMULATABLE.get(self.modality)
        d["ground_truth_cost"] = GT_COST.get(self.modality, "unknown")
        d["scored"] = (self.role == "benchmark"
                       and SIMULATABLE.get(self.modality) is True)
        if self.role == "benchmark" and SIMULATABLE.get(self.modality) is False:
            d["conflict"] = (
                f"{self.modality} is marked as a benchmark channel but cannot "
                "be simulated faithfully. It belongs in the application case, "
                "not in the scored set.")
        return d


class SensorHub:
    """
    The registry. Thread-safe, never raises into a driver, and never lets one
    sensor's failure affect another's.

    `snapshot()` is what the recorder writes and `report()` is what the console
    draws, and they are deliberately different: the recorder wants values, the
    console wants values plus health. Merging them would mean either recording
    health text into every sample or hiding staleness from the operator.
    """

    STALE_S = 2.0

    def __init__(self):
        self._lock = threading.Lock()
        self._channels: dict[str, Channel] = {}

    # -- registration ------------------------------------------------------
    def register(self, channel: Channel, replace: bool = True) -> Channel:
        with self._lock:
            if channel.id in self._channels and not replace:
                return self._channels[channel.id]
            self._channels[channel.id] = channel
        return channel

    def declare(self, **kw) -> Channel:
        """A sensor that is planned but not here yet."""
        kw.setdefault("status", "declared")
        return self.register(Channel(**kw))

    def attach(self, channel_id: str, reader: Callable[[], dict],
               detail: str = "") -> dict:
        with self._lock:
            ch = self._channels.get(channel_id)
            if ch is None:
                return {"ok": False, "error": f"no channel {channel_id!r}"}
            ch.reader = reader
            ch.status = "present"
            ch.detail = detail or ch.detail
            ch._err = ""
        return {"ok": True, "id": channel_id}

    def detach(self, channel_id: str) -> dict:
        with self._lock:
            ch = self._channels.get(channel_id)
            if ch is None:
                return {"ok": False, "error": f"no channel {channel_id!r}"}
            ch.reader = None
            ch.status = "declared"
        return {"ok": True, "id": channel_id}

    def get(self, channel_id: str) -> Channel | None:
        with self._lock:
            return self._channels.get(channel_id)

    # -- sampling ----------------------------------------------------------
    def poll(self, channel_id: str | None = None) -> dict:
        """
        Read every attached channel once.

        A driver that raises is marked failed WITH ITS MESSAGE and skipped; it
        does not stop the others and it does not stop a recording. Over a
        four-week campaign a sensor will drop out, and the run that was in
        progress when it did is still worth having.
        """
        with self._lock:
            items = [(cid, ch) for cid, ch in self._channels.items()
                     if ch.reader is not None
                     and (channel_id is None or cid == channel_id)]
        now = time.monotonic()
        for cid, ch in items:
            try:
                val = ch.reader()
            except Exception as e:                  # noqa: BLE001
                ch.status = "failed"
                ch._err = str(e)
                continue
            if val is None:
                continue
            ch._last = val if isinstance(val, dict) else {"value": val}
            ch._t_last = now
            ch._n += 1
            ch.status = "streaming"
            ch._err = ""
        return {"polled": len(items)}

    def snapshot(self, fresh_only: bool = True) -> dict:
        """
        Values for one recorded sample.

        Stale channels are omitted rather than repeated. A recorder that keeps
        writing the last value of a dead sensor produces a file in which the
        dropout is invisible, and a dropout you cannot see is worse than a gap
        you can.
        """
        self.poll()
        now = time.monotonic()
        out = {}
        with self._lock:
            for cid, ch in self._channels.items():
                if not ch._last:
                    continue
                if fresh_only and (now - ch._t_last) > self.STALE_S:
                    continue
                out[cid] = dict(ch._last)
        return out

    # -- reporting ---------------------------------------------------------
    def report(self) -> dict:
        self.poll()
        now = time.monotonic()
        chans = []
        with self._lock:
            for ch in self._channels.values():
                d = ch.descriptor()
                age = (now - ch._t_last) if ch._t_last else None
                d.update({
                    "samples": ch._n,
                    "age_s": round(age, 2) if age is not None else None,
                    "stale": bool(age is not None and age > self.STALE_S),
                    "error": ch._err,
                    "latest": {k: v for k, v in list(ch._last.items())[:8]},
                })
                if d["stale"]:
                    d["status"] = "stale"
                chans.append(d)
        chans.sort(key=lambda c: (c["role"] != "benchmark", c["modality"], c["id"]))
        live = [c for c in chans if c["status"] in ("streaming", "present")]
        return {
            "channels": chans,
            "n_total": len(chans),
            "n_live": len(live),
            "n_declared": len([c for c in chans if c["status"] == "declared"]),
            "n_failed": len([c for c in chans if c["status"] == "failed"]),
            "benchmark_channels": [c["id"] for c in chans if c["scored"]],
            "application_channels": [c["id"] for c in chans if not c["scored"]],
            "conflicts": [c["conflict"] for c in chans if c.get("conflict")],
        }


HUB = SensorHub()


# ---------------------------------------------------------------------------
# the standing roster
# ---------------------------------------------------------------------------

def install_defaults(hub: SensorHub | None = None) -> SensorHub:
    """
    Declare the modalities this project has committed to, whether or not the
    hardware is on the bench yet.

    The ones without drivers are not placeholders for tidiness — they fix the
    field names, the units, the frame and the benchmark role NOW, while there
    is time to argue about them, instead of on the afternoon the sensor lands.
    """
    hub = hub or HUB
    hub.declare(id="ur_tcp", label="Tool position & force", modality="position",
                units="m, rad, N, Nm", rate_hz=125.0, frame="base",
                vendor="Universal Robots", transport="RTDE", role="benchmark",
                fields=["tcp_pose", "q", "qd", "tcp_force", "current"])
    hub.declare(id="imu_ind0", label="Industrial IMU (FusionHub)",
                modality="orientation", units="quaternion, rad/s, m/s^2",
                rate_hz=200.0, frame="tcp", vendor="FusionHub",
                transport="UDP/TCP/serial", role="benchmark",
                fields=["quat", "gyro", "accel"])
    hub.declare(id="imu_d435i", label="Camera IMU (BMI055)",
                modality="angular_rate", units="rad/s, m/s^2", rate_hz=200.0,
                frame="camera", vendor="Intel", transport="USB",
                role="benchmark", fields=["gyro", "accel", "quat"])
    hub.declare(id="cam_depth", label="Depth camera", modality="vision_depth",
                units="m", rate_hz=30.0, frame="camera", vendor="Intel D435i",
                transport="USB", role="application",
                fields=["depth", "intrinsics"])
    hub.declare(id="cam_color", label="Colour camera", modality="vision_rgb",
                units="8-bit BGR", rate_hz=30.0, frame="camera",
                vendor="Intel D435i", transport="USB", role="application",
                fields=["color"])
    hub.declare(id="cam_ir", label="Infrared stereo pair",
                modality="vision_rgb", units="8-bit mono", rate_hz=30.0,
                frame="camera", vendor="Intel D435i", transport="USB",
                role="application", fields=["ir_left", "ir_right"])
    # Not here yet, and deliberately visible.
    hub.declare(id="eddy0", label="Eddy current probe", modality="eddy_current",
                units="V (I/Q)", rate_hz=1000.0, frame="tcp",
                transport="(awaiting hardware)", role="application",
                fields=["i", "q", "lift_off"],
                detail="Application-case channel. Not scored: it cannot be "
                       "simulated faithfully, so it cannot carry a sim-to-real "
                       "gap number.")
    hub.declare(id="ut0", label="Ultrasonic probe", modality="ultrasound",
                units="A-scan, 8-bit", rate_hz=500.0, frame="tcp",
                transport="(awaiting hardware)", role="application",
                fields=["ascan", "gate_amplitude", "time_of_flight"],
                detail="Application-case channel. Needs couplant and a "
                       "standoff policy before it can be recorded usefully.")
    hub.declare(id="ir_cam0", label="Thermal camera", modality="thermography",
                units="degC", rate_hz=9.0, frame="tcp",
                transport="(awaiting hardware)", role="application",
                fields=["frame", "ambient"],
                detail="Application-case channel. Requires a controlled "
                       "excitation to mean anything.")
    return hub


install_defaults()


def wire_standard_sources(*, ur_state=None, imu_latest=None,
                          camera_state=None) -> dict:
    """
    Attach the drivers this codebase already has.

    Each argument is a zero-argument callable the bridge owns. Passing them in
    rather than importing them here keeps this module free of the bridge's
    globals and therefore testable on its own.
    """
    out = {}
    if ur_state is not None:
        out["ur_tcp"] = HUB.attach("ur_tcp", ur_state, "RTDE")
    if imu_latest is not None:
        for unit, cid in (("ind0", "imu_ind0"), ("d435i", "imu_d435i")):
            out[cid] = HUB.attach(
                cid, (lambda u=unit: (imu_latest() or {}).get(u) or None),
                "inertial hub")
    if camera_state is not None:
        for cid in ("cam_depth", "cam_color", "cam_ir"):
            out[cid] = HUB.attach(cid, (lambda c=cid: camera_state(c)), "RealSense")
    return out
