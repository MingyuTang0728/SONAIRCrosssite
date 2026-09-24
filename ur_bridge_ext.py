"""
ur_bridge_ext.py — the message layer that joins the new backends to the browser.

multimodal_bridge.py owns the socket and the existing message set. This module
owns everything added since: full UR telemetry, the complete control surface,
and the eye-in-hand 3D reconstruction. Keeping it separate means the console's
original jog path cannot be broken by a change to the reconstruction code, and
the bridge's dispatch stays one line per family.

Message families, all namespaced so routing is a prefix test:

    ur_state        (outbound)  full telemetry, every field, ~10 Hz to the UI
    ur_*            (inbound)   control — see ur_control.handle_command
    imu             (outbound)  inertial, in BOTH shapes (see note below)
    scan3d_*        (inbound)   reconstruction session control

On the IMU shape: the console expects {type:"imu", timestamp, accel, gyro} for
a single unit, and the benchmark recorder needs every unit at once. Rather than
pick one and break the other, the outbound message carries both — flat fields
for the primary unit, plus a `units` map. Consumers read whichever they know.
"""
from __future__ import annotations

import logging
import math
import os
import time

log = logging.getLogger("ur.bridge_ext")

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False
    np = None

try:
    from ur_telemetry import URTelemetry
    from ur_control import URController, Envelope, RTDEInputChannel, handle_command as ur_handle
    from ur_jog import JogController, step as jog_step
    _HAS_UR = True
except Exception as e:      # noqa: BLE001
    _HAS_UR = False
    _UR_ERR = str(e)
    log.warning("UR backends unavailable: %s", e)

try:
    import scan3d
    _HAS_SCAN = _HAS_NUMPY
except Exception:
    _HAS_SCAN = False
    scan3d = None


# =============================================================================
# 3D reconstruction session, driven from the browser
# =============================================================================

class HandEyeStore:
    """
    Where the camera sits on the tool, held in one place.

    This was a full reconstruction service — start, capture, reconstruct,
    plan — and it worked, but multiview.py replaced it with a better pipeline
    (the region is cropped in the base frame before fusion, viewpoints are
    planned from the part's own size, and a view that lands too few points is
    refused rather than merged). Two 3D pipelines behind one console is a
    choice the operator should never have to make, and the old one had no
    controls, so it was reachable only by hand-written messages.

    What survived is the part everything genuinely shares: the hand-eye
    transform. The calibration writes it here and the inspection and scanning
    code reads it here, so there is exactly one answer to "where is the
    camera" and no way for two copies to drift apart.

    The `*_fn` attributes are kept because the bridge sets them; nothing reads
    them now, and they cost nothing to accept.
    """

    def __init__(self):
        self.T_tcp_cam = None
        self.frames_fn = None
        self.pose_fn = None
        self.intrinsics_fn = None

    def status(self) -> dict:
        return {"has_handeye": self.T_tcp_cam is not None}


SCAN3D = HandEyeStore()


# =============================================================================
# UR service
# =============================================================================

class URService:
    """Telemetry plus control, with one status blob for the UI."""

    def __init__(self):
        self.telemetry = None
        self.controller = None
        self.jog = None
        self.host = ""
        self.enabled = False

    def start(self, host: str, envelope: dict | None = None,
              frequency: float = 125.0, use_rtde_inputs: bool = False) -> dict:
        if not _HAS_UR:
            return {"ok": False, "error": _UR_ERR}
        # Starting twice must not leave two telemetry threads racing for the
        # same socket: the second one wins some packets, the first one times
        # out, and the health block flickers between "streaming" and "timed
        # out" for no reason the operator can see.
        if self.telemetry or self.controller:
            log.info("restarting UR service (was %s)", self.host or "unset")
            self.stop()
        self.host = host
        env = Envelope(
            x=(envelope or {}).get("x", (-0.6, 0.6)),
            y=(envelope or {}).get("y", (-0.6, 0.6)),
            z=(envelope or {}).get("z", (0.05, 0.7)),
        ) if envelope else Envelope()
        self.controller = URController(host, env)
        if use_rtde_inputs:
            # Older controllers allow only one RTDE connection, and telemetry
            # already holds one. Off by default for that reason.
            self.controller.attach_rtde_inputs(RTDEInputChannel(host))
        self.telemetry = URTelemetry(host, frequency=frequency)
        self.telemetry.start()
        # The jog controller owns the cadence of continuous motion. The browser
        # only ever tells it what velocity is wanted; it decides when to send.
        self.jog = JogController(self.controller)
        self.jog.start()
        self.enabled = True
        return {"ok": True, "host": host, "envelope": env.as_dict()}

    def state(self) -> dict:
        return self.telemetry.state() if self.telemetry else {}

    def status(self) -> dict:
        if not self.telemetry:
            return {"enabled": False,
                    "error": _UR_ERR if not _HAS_UR else "UR service not started"}
        st = self.telemetry.status()
        st["enabled"] = True
        st["host"] = self.host
        st["control_available"] = self.controller is not None
        if self.controller:
            st["envelope"] = self.controller.envelope.as_dict()
            st["script_sent"] = self.controller.script.sent
            st["script_error"] = self.controller.script.last_error
        return st

    def stop(self) -> None:
        if self.jog:
            try:
                self.jog.shutdown()
            except Exception:
                pass
            self.jog = None
        if self.telemetry:
            try:
                self.telemetry.stop()
            except Exception:
                pass
            self.telemetry = None
        if self.controller:
            try:
                self.controller.close()
            except Exception:
                pass
            self.controller = None
        self.enabled = False


UR = URService()


# =============================================================================
# dispatch
# =============================================================================

def handle_message(data: dict) -> dict | None:
    """
    Route one browser message. Returns a reply, or None if not ours.

    UR control messages are checked BEFORE the scan messages because a stop or
    an e-stop must not be able to queue behind a reconstruction call.
    """
    t = data.get("type")
    if not isinstance(t, str):
        return None

    # Jog first, and deliberately cheap: a lock and six floats. Anything that
    # makes this path slower — a thread hop, a status query, an audit write —
    # spends the latency budget the jog exists to protect.
    if t == "jog_vel":
        if UR.jog is None:
            return {"type": "jog_res", "ok": False,
                    "msg": "robot link not started"}
        r = UR.jog.set_velocity(data.get("xd", []),
                                float(data.get("ttl_ms", 400)) / 1000.0)
        return {"type": "jog_res", "ok": r["ok"], "quiet": True}
    if t == "jog_stop":
        if UR.jog is None:
            return {"type": "jog_res", "ok": False, "msg": "robot link not started"}
        return {"type": "jog_res", **UR.jog.stop(), "quiet": True}
    if t == "jog_halt":
        if UR.jog is None:
            return {"type": "jog_res", "ok": False, "msg": "robot link not started"}
        return {"type": "jog_res", **UR.jog.halt()}
    if t == "jog_status":
        return {"type": "jog_status_res",
                **(UR.jog.status() if UR.jog else {"running": False})}
    if t == "jog_step":
        if UR.controller is None:
            return {"type": "jog_step_res", "ok": False,
                    "error": "robot link not started"}
        pose = (UR.state() or {}).get("actual_TCP_pose")
        return {"type": "jog_step_res",
                **jog_step(UR.controller, pose, data.get("axis", "x"),
                           float(data.get("distance_mm", 1.0)),
                           data.get("frame", "base"),
                           float(data.get("speed", 0.05)))}

    if t == "ur_service_status":
        return {"type": "ur_service_status", **UR.status()}

    if t == "ur_service_start":
        return {"type": "ur_service_start_res",
                **UR.start(data.get("host") or os.environ.get("UR_IP", "192.168.0.20"),
                           envelope=data.get("envelope"),
                           frequency=float(data.get("frequency", 125.0)),
                           use_rtde_inputs=bool(data.get("rtde_inputs")))}

    if t == "ur_speed_slider" and UR.controller is not None \
            and UR.controller.rtde_inputs is None:
        # Attached HERE, the first time it is actually wanted, rather than at
        # startup. It opens a second RTDE connection and older controllers
        # allow only one, so attaching it eagerly cost the telemetry stream
        # its slot on exactly the controllers least able to spare it. An
        # operator who never touches the speed slider never pays for it.
        try:
            UR.controller.attach_rtde_inputs(RTDEInputChannel(UR.host))
            log.info("RTDE input channel attached on demand for the speed slider")
        except Exception as e:      # noqa: BLE001
            return {"type": "ur_cmd_res", "cmd": t, "ok": False,
                    "msg": f"speed slider unavailable: {e}"}

    if t.startswith("ur_"):
        if UR.controller is None:
            return {"type": "ur_cmd_res", "cmd": t, "ok": False,
                    "msg": "UR control not started — send ur_service_start first"}
        return ur_handle(UR.controller, data)

    # The scan3d_* messages were removed with the service they drove; the
    # multi-view pipeline (mv_* in multimodal_bridge) does that job now.
    return None


def imu_message(hub_latest: dict, primary_unit: str = "ind0") -> dict:
    """
    Build the outbound IMU message in both shapes at once.

    Flat `accel`/`gyro`/`timestamp` for the console's existing contract, plus
    the full `units` map for the recorder. Picking one shape would have meant
    breaking a consumer that already works.
    """
    msg: dict = {"type": "imu", "units": hub_latest}
    unit = primary_unit if primary_unit in hub_latest else next(iter(hub_latest), None)
    if unit:
        d = hub_latest[unit]
        msg["unit"] = unit
        msg["timestamp"] = d.get("t", time.time())
        if "accel" in d:
            msg["accel"] = d["accel"]
        if "gyro" in d:
            msg["gyro"] = d["gyro"]
        if "quat" in d:
            msg["quat"] = d["quat"]
    return msg
