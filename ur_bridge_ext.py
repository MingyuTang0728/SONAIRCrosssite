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

class Scan3DService:
    """
    Wraps scan3d.ReconstructionSession with the state the UI needs.

    Capture is pull-based: the browser (or an automated sweep) moves the arm,
    then asks for a capture. Doing it that way rather than capturing on a timer
    means every view is taken with the arm settled, and a view taken mid-motion
    — which smears the depth image and poisons the fused cloud — cannot happen
    by accident.
    """

    def __init__(self):
        self.session = None
        self.intrinsics = None
        self.T_tcp_cam = None
        self.last_error = ""
        self.frames_fn = None        # () -> raw depth array
        self.pose_fn = None          # () -> TCP pose
        self.intrinsics_fn = None    # () -> CameraIntrinsics

    def available(self) -> tuple[bool, str]:
        if not _HAS_SCAN:
            return False, "scan3d needs numpy on the host"
        if self.frames_fn is None:
            return False, "no depth source wired — is the RealSense running?"
        return True, ""

    def start(self, T_tcp_cam=None, voxel_mm: float = 2.0) -> dict:
        ok, why = self.available()
        if not ok:
            return {"ok": False, "error": why}
        intr = None
        if self.intrinsics_fn:
            try:
                intr = self.intrinsics_fn()
            except Exception as e:
                return {"ok": False, "error": f"could not read camera intrinsics: {e}"}
        if intr is None:
            return {"ok": False, "error":
                    "camera intrinsics unavailable — start the depth stream first. "
                    "Intrinsics must come from the camera, never from a datasheet."}

        if T_tcp_cam is None:
            T_tcp_cam = self.T_tcp_cam
        if T_tcp_cam is None:
            return {"ok": False, "error":
                    "hand-eye calibration (T_tcp_cam) not set. Every point in the "
                    "reconstruction inherits this transform and a 2 deg error puts "
                    "the cloud 10 mm out at 300 mm standoff — it cannot be guessed."}

        self.T_tcp_cam = np.asarray(T_tcp_cam, dtype=float).reshape(4, 4)
        self.intrinsics = intr
        self.session = scan3d.ReconstructionSession(
            intr, self.T_tcp_cam, voxel_m=voxel_mm / 1000.0)
        return {"ok": True, "voxel_mm": voxel_mm,
                "intrinsics": {"fx": intr.fx, "fy": intr.fy, "cx": intr.cx,
                               "cy": intr.cy, "width": intr.width,
                               "height": intr.height,
                               "depth_scale": intr.depth_scale}}

    def capture(self, stride: int = 2) -> dict:
        if self.session is None:
            return {"ok": False, "error": "no reconstruction session — call scan3d_start"}
        try:
            depth = self.frames_fn()
        except Exception as e:
            return {"ok": False, "error": f"depth read failed: {e}"}
        if depth is None:
            return {"ok": False, "error": "no depth frame available right now"}
        try:
            pose = self.pose_fn() if self.pose_fn else None
        except Exception as e:
            return {"ok": False, "error": f"TCP pose read failed: {e}"}
        if not pose or len(pose) < 6:
            return {"ok": False, "error":
                    "no TCP pose — the arm must be connected, since a view "
                    "without its pose cannot be placed in the base frame"}
        try:
            res = self.session.add_view(depth, pose, stride=stride)
            return {"ok": True, **res}
        except Exception as e:
            self.last_error = str(e)
            return {"ok": False, "error": str(e)}

    def survey(self, centre, radius=0.25, height=0.35, n_views=8) -> dict:
        """The viewpoint ring, converted to TCP poses the arm can be sent to."""
        if not _HAS_SCAN:
            return {"ok": False, "error": "scan3d needs numpy"}
        if self.T_tcp_cam is None:
            return {"ok": False, "error": "hand-eye calibration not set"}
        views = scan3d.plan_survey_poses(centre, radius=radius, height=height,
                                         n_views=n_views)
        for v in views:
            v["tcp_pose"] = scan3d.survey_to_tcp(v["camera_pose"], self.T_tcp_cam)
        return {"ok": True, "views": views, "n": len(views)}

    def reconstruct(self, min_hits: int = 2, plane_tol_mm: float = 4.0,
                    link_mm: float = 8.0) -> dict:
        if self.session is None:
            return {"ok": False, "error": "no reconstruction session"}
        try:
            return self.session.reconstruct(min_hits=min_hits,
                                            plane_tol=plane_tol_mm / 1000.0,
                                            link_m=link_mm / 1000.0)
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def plan(self, standoff_mm=100.0, line_spacing_mm=5.0, step_mm=5.0,
             margin_mm=5.0, tool_rotvec=None) -> dict:
        if self.session is None:
            return {"ok": False, "error": "no reconstruction session"}
        try:
            return self.session.plan(
                standoff=standoff_mm / 1000.0,
                line_spacing=line_spacing_mm / 1000.0,
                step_along=step_mm / 1000.0,
                margin=margin_mm / 1000.0,
                tool_rotvec=tuple(tool_rotvec) if tool_rotvec else (0.0, math.pi, 0.0))
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def status(self) -> dict:
        ok, why = self.available()
        s = {"available": ok, "reason": why, "active": self.session is not None,
             "has_handeye": self.T_tcp_cam is not None}
        if self.session:
            s["cloud"] = self.session.cloud.stats()
            s["views"] = len(self.session.captures)
            if self.session.component:
                s["component"] = self.session.component.as_dict()
        return s


SCAN3D = Scan3DService()


# =============================================================================
# UR service
# =============================================================================

class URService:
    """Telemetry plus control, with one status blob for the UI."""

    def __init__(self):
        self.telemetry = None
        self.controller = None
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

    if t == "ur_service_status":
        return {"type": "ur_service_status", **UR.status()}

    if t == "ur_service_start":
        return {"type": "ur_service_start_res",
                **UR.start(data.get("host") or os.environ.get("UR_IP", "192.168.0.20"),
                           envelope=data.get("envelope"),
                           frequency=float(data.get("frequency", 125.0)),
                           use_rtde_inputs=bool(data.get("rtde_inputs")))}

    if t.startswith("ur_"):
        if UR.controller is None:
            return {"type": "ur_cmd_res", "cmd": t, "ok": False,
                    "msg": "UR control not started — send ur_service_start first"}
        return ur_handle(UR.controller, data)

    if t == "scan3d_status":
        return {"type": "scan3d_status", **SCAN3D.status()}
    if t == "scan3d_set_handeye":
        m = data.get("T_tcp_cam")
        if not m or len(m) != 4 or any(len(r) != 4 for r in m):
            return {"type": "scan3d_res", "cmd": t, "ok": False,
                    "error": "T_tcp_cam must be a 4x4 matrix"}
        SCAN3D.T_tcp_cam = np.asarray(m, dtype=float) if _HAS_NUMPY else m
        return {"type": "scan3d_res", "cmd": t, "ok": True}
    if t == "scan3d_start":
        return {"type": "scan3d_res", "cmd": t,
                **SCAN3D.start(data.get("T_tcp_cam"), data.get("voxel_mm", 2.0))}
    if t == "scan3d_survey":
        return {"type": "scan3d_survey_res",
                **SCAN3D.survey(data.get("centre", [0.4, 0.0, 0.1]),
                                data.get("radius", 0.25), data.get("height", 0.35),
                                int(data.get("n_views", 8)))}
    if t == "scan3d_capture":
        return {"type": "scan3d_capture_res", **SCAN3D.capture(int(data.get("stride", 2)))}
    if t == "scan3d_reconstruct":
        return {"type": "scan3d_reconstruct_res",
                **SCAN3D.reconstruct(int(data.get("min_hits", 2)),
                                     float(data.get("plane_tol_mm", 4.0)),
                                     float(data.get("link_mm", 8.0)))}
    if t == "scan3d_plan":
        return {"type": "scan3d_plan_res",
                **SCAN3D.plan(float(data.get("standoff_mm", 100.0)),
                              float(data.get("line_spacing_mm", 5.0)),
                              float(data.get("step_mm", 5.0)),
                              float(data.get("margin_mm", 5.0)),
                              data.get("tool_rotvec"))}
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
