"""
ur_control.py — the complete UR control surface.

Three channels, because the robot exposes three and they do different jobs:

  Secondary interface, port 30002
      Accepts URScript text. Everything that moves the arm goes here:
      movej, movel, movep, servoj, speedl, speedj, freedrive, IO, payload,
      TCP offset, tool voltage, popups. One long-lived socket, because
      reconnecting per command costs ~30 ms and turns a servoj stream into a
      stutter.

  Dashboard server, port 29999
      Line-based text commands for things URScript cannot do from inside a
      running program: power on/off, brake release, load and play a .urp,
      unlock a protective stop, close a safety popup, read the program state.

  RTDE inputs, port 30004
      Not used here. Register writes are only needed when a URScript program
      running ON the robot has to read values from the PC, which is a
      different architecture from the one this console uses.

Every motion command passes the envelope check before it is sent. That check
also exists in the relay and in multimodal_bridge, and the duplication is
deliberate — this is the last one before the wire.
"""
from __future__ import annotations

import logging
import math
import socket
import threading
import time

log = logging.getLogger("ur.control")

SECONDARY_PORT = 30002
DASHBOARD_PORT = 29999


def f(v) -> str:
    """URScript wants plain decimals; exponent notation is a syntax error there."""
    return f"{float(v):.6f}"


# =============================================================================
# Safety envelope
# =============================================================================

class Envelope:
    """Axis-aligned box in the robot base frame, plus a speed ceiling."""

    def __init__(self, x=(-0.6, 0.6), y=(-0.6, 0.6), z=(0.05, 0.7),
                 max_linear_speed=0.25, max_joint_speed=1.0):
        self.x, self.y, self.z = x, y, z
        self.max_linear_speed = max_linear_speed
        self.max_joint_speed = max_joint_speed

    def accepts_pose(self, pose) -> tuple[bool, str]:
        if not pose or len(pose) < 3:
            return False, "pose must have at least x, y, z"
        px, py, pz = float(pose[0]), float(pose[1]), float(pose[2])
        if not all(math.isfinite(v) for v in (px, py, pz)):
            return False, "pose contains a non-finite value"
        if not (self.x[0] <= px <= self.x[1]):
            return False, f"x={px:.3f} outside [{self.x[0]}, {self.x[1]}]"
        if not (self.y[0] <= py <= self.y[1]):
            return False, f"y={py:.3f} outside [{self.y[0]}, {self.y[1]}]"
        if not (self.z[0] <= pz <= self.z[1]):
            return False, f"z={pz:.3f} outside [{self.z[0]}, {self.z[1]}]"
        return True, ""

    def clamp_linear(self, v: float) -> float:
        return max(0.0, min(float(v), self.max_linear_speed))

    def clamp_joint(self, v: float) -> float:
        return max(0.0, min(float(v), self.max_joint_speed))

    def as_dict(self) -> dict:
        return {"x_min": self.x[0], "x_max": self.x[1],
                "y_min": self.y[0], "y_max": self.y[1],
                "z_min": self.z[0], "z_max": self.z[1],
                "max_linear_speed": self.max_linear_speed,
                "max_joint_speed": self.max_joint_speed}


# =============================================================================
# Secondary interface — URScript
# =============================================================================

class URScriptChannel:
    """Long-lived socket to port 30002, with lazy reconnect."""

    def __init__(self, host: str, port: int = SECONDARY_PORT):
        self.host, self.port = host, port
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self.sent = 0
        self.last_error = ""

    def _ensure(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        # 1.5 s, not 3: this is a LAN robot, so a connection that has not
        # completed by now is not slow, it is refused or the host is wrong.
        # Two attempts at 3 s each made a wrong IP look like a hang.
        s = socket.create_connection((self.host, self.port), timeout=1.5)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = s
        log.info("URScript channel open to %s:%d", self.host, self.port)
        return s

    def send(self, script: str) -> tuple[bool, str]:
        if not script.endswith("\n"):
            script += "\n"
        with self._lock:
            for attempt in (0, 1):
                try:
                    self._ensure().sendall(script.encode("utf-8"))
                    self.sent += 1
                    return True, ""
                except OSError as e:
                    self.last_error = str(e)
                    try:
                        if self._sock:
                            self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
                    if attempt == 1:
                        return False, (
                            f"URScript send to {self.host}:{self.port} failed: {e}. "
                            "Check the controller IP, that the robot is powered on, "
                            "and that it is not in local-control mode (the pendant "
                            "must be in Remote Control for external URScript).")
            return False, "unreachable"

    def close(self) -> None:
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None


# =============================================================================
# Dashboard server
# =============================================================================

class DashboardChannel:
    """
    Port 29999. Request/response, one line each way.

    A fresh connection per command on purpose: the dashboard server drops idle
    connections without warning, and a stale socket here would silently swallow
    a power-on or a protective-stop unlock — the two commands you least want to
    fail quietly.
    """

    def __init__(self, host: str, port: int = DASHBOARD_PORT):
        self.host, self.port = host, port
        self._lock = threading.Lock()

    def command(self, cmd: str, timeout: float = 3.0) -> tuple[bool, str]:
        with self._lock:
            try:
                with socket.create_connection((self.host, self.port), timeout=timeout) as s:
                    s.settimeout(timeout)
                    try:
                        s.recv(1024)          # banner
                    except socket.timeout:
                        pass
                    s.sendall((cmd.strip() + "\n").encode("utf-8"))
                    reply = s.recv(4096).decode("utf-8", "ignore").strip()
                    return True, reply
            except Exception as e:
                return False, f"dashboard '{cmd}': {e}"


# =============================================================================
# RTDE inputs — the speed slider, and nothing else for now
# =============================================================================

class RTDEInputChannel:
    """
    Writes RTDE input registers. Currently only the speed slider, which is the
    one control on the teach pendant that has no URScript equivalent.

    Older controllers accept only ONE RTDE connection at a time, so opening a
    second one here would silently kill the telemetry stream. The channel is
    therefore opt-in: it is created only when something asks for it, and it
    reports plainly if the controller refuses the connection.
    """

    # Imported lazily so ur_control stays usable without ur_telemetry present.
    def __init__(self, host: str, port: int = 30004):
        self.host, self.port = host, port
        self._client = None
        self._lock = threading.Lock()
        self._recipe_id = 0
        self.last_error = ""

    def _connect(self):
        from ur_telemetry import RTDEClient, RTDE_CONTROL_PACKAGE_SETUP_INPUTS
        import struct as _struct

        c = RTDEClient(self.host, self.port)
        c.connect()
        names = "speed_slider_mask,speed_slider_fraction"
        c._send(RTDE_CONTROL_PACKAGE_SETUP_INPUTS, names.encode("utf-8"))
        cmd, body = c._recv_packet()
        if cmd != RTDE_CONTROL_PACKAGE_SETUP_INPUTS or not body:
            raise ConnectionError("controller refused the RTDE input recipe")
        types = body[1:].decode("utf-8")
        if "NOT_FOUND" in types:
            raise ConnectionError(f"speed slider inputs unavailable: {types}")
        self._recipe_id = body[0]
        c.start()
        self._client = c
        return c

    def set_speed_slider(self, fraction: float) -> tuple[bool, str]:
        import struct as _struct
        from ur_telemetry import RTDE_DATA_PACKAGE

        with self._lock:
            try:
                c = self._client or self._connect()
                payload = (bytes([self._recipe_id])
                           + _struct.pack(">I", 1)            # mask: slider active
                           + _struct.pack(">d", float(fraction)))
                c._send(RTDE_DATA_PACKAGE, payload)
                return True, f"speed slider set to {fraction:.2f}"
            except Exception as e:
                self.last_error = str(e)
                self._client = None
                return False, f"speed slider: {e}"

    def close(self) -> None:
        with self._lock:
            if self._client:
                self._client.close()
                self._client = None


# =============================================================================
# The controller
# =============================================================================

class URController:
    """Everything the console can ask the robot to do."""

    def __init__(self, host: str, envelope: Envelope | None = None):
        self.host = host
        self.envelope = envelope or Envelope()
        self.script = URScriptChannel(host)
        self.dashboard = DashboardChannel(host)
        self.rtde_inputs: "RTDEInputChannel | None" = None
        self._freedrive = False

    def attach_rtde_inputs(self, channel: "RTDEInputChannel") -> None:
        """Wire in the RTDE input channel that carries the speed slider."""
        self.rtde_inputs = channel

    # --- motion --------------------------------------------------------------

    def movel(self, pose, a=0.3, v=0.1, r=0.0) -> tuple[bool, str]:
        ok, why = self.envelope.accepts_pose(pose)
        if not ok:
            return False, f"envelope rejected movel: {why}"
        v = self.envelope.clamp_linear(v)
        return self.script.send(
            f"movel(p[{f(pose[0])},{f(pose[1])},{f(pose[2])},"
            f"{f(pose[3])},{f(pose[4])},{f(pose[5])}], a={f(a)}, v={f(v)}, r={f(r)})")

    def movej(self, q, a=1.0, v=0.5, r=0.0, is_pose=False) -> tuple[bool, str]:
        if is_pose:
            ok, why = self.envelope.accepts_pose(q)
            if not ok:
                return False, f"envelope rejected movej: {why}"
            target = (f"p[{f(q[0])},{f(q[1])},{f(q[2])},{f(q[3])},{f(q[4])},{f(q[5])}]")
        else:
            if len(q) != 6:
                return False, "movej needs six joint angles"
            target = "[" + ",".join(f(x) for x in q) + "]"
        v = self.envelope.clamp_joint(v)
        return self.script.send(f"movej({target}, a={f(a)}, v={f(v)}, r={f(r)})")

    def movep(self, pose, a=0.3, v=0.1, r=0.01) -> tuple[bool, str]:
        ok, why = self.envelope.accepts_pose(pose)
        if not ok:
            return False, f"envelope rejected movep: {why}"
        v = self.envelope.clamp_linear(v)
        return self.script.send(
            f"movep(p[{f(pose[0])},{f(pose[1])},{f(pose[2])},"
            f"{f(pose[3])},{f(pose[4])},{f(pose[5])}], a={f(a)}, v={f(v)}, r={f(r)})")

    def servoj(self, q, t=0.008, lookahead=0.1, gain=300) -> tuple[bool, str]:
        """
        Streamed joint servo. Call it at a steady rate; a gap produces a jerk.
        Deliberately has no envelope check on joint space — the caller must
        already have verified the resulting pose, and adding IK here would put
        a solver in the hot path of a 125 Hz stream.
        """
        if len(q) != 6:
            return False, "servoj needs six joint angles"
        return self.script.send(
            "servoj([" + ",".join(f(x) for x in q) +
            f"], t={f(t)}, lookahead_time={f(lookahead)}, gain={int(gain)})")

    def speedl(self, xd, a=0.5, t=0.1) -> tuple[bool, str]:
        lin = math.sqrt(sum(float(c) ** 2 for c in xd[:3]))
        if lin > self.envelope.max_linear_speed:
            scale = self.envelope.max_linear_speed / lin
            xd = [c * scale for c in xd[:3]] + list(xd[3:6])
        return self.script.send(
            "speedl([" + ",".join(f(c) for c in xd[:6]) + f"], a={f(a)}, t={f(t)})")

    def speedj(self, qd, a=1.0, t=0.1) -> tuple[bool, str]:
        qd = [max(-self.envelope.max_joint_speed,
                  min(self.envelope.max_joint_speed, float(c))) for c in qd[:6]]
        return self.script.send(
            "speedj([" + ",".join(f(c) for c in qd) + f"], a={f(a)}, t={f(t)})")

    def stop(self, a=2.0) -> tuple[bool, str]:
        return self.script.send(f"stopl({f(a)})")

    def stopj(self, a=2.0) -> tuple[bool, str]:
        return self.script.send(f"stopj({f(a)})")

    # --- freedrive -----------------------------------------------------------

    def freedrive(self, enable: bool, axes=None) -> tuple[bool, str]:
        """
        `axes` is six 0/1 flags for a constrained freedrive, e.g. plane-only
        teaching. Passing None gives all six, which is the usual hand-guiding.
        """
        if enable:
            self._freedrive = True
            if axes and len(axes) == 6:
                flags = ",".join("1" if int(a) else "0" for a in axes)
                return self.script.send(
                    "def fd():\n"
                    "  freedrive_mode()\n"
                    f"  force_mode_set_damping(0.005)\n"
                    f"  # constrained axes: [{flags}]\n"
                    "  while True:\n    sync()\n  end\n"
                    "end\n")
            return self.script.send("def fd():\n  freedrive_mode()\n"
                                    "  while True:\n    sync()\n  end\nend\n")
        self._freedrive = False
        return self.script.send("end_freedrive_mode()")

    # --- tool, payload, IO ---------------------------------------------------

    def set_payload(self, mass_kg: float, cog=(0.0, 0.0, 0.0)) -> tuple[bool, str]:
        """
        The mass at the wrist. Worth setting correctly for its own sake, and
        doubly so here: the same number is an input to the Isaac contract, and
        a payload the controller does not know about shows up as a force
        reading that is really gravity.
        """
        if not (0.0 <= float(mass_kg) <= 5.0):
            return False, "UR5e payload must be between 0 and 5 kg"
        return self.script.send(
            f"set_payload({f(mass_kg)}, [{f(cog[0])},{f(cog[1])},{f(cog[2])}])")

    def set_tcp(self, pose) -> tuple[bool, str]:
        if len(pose) != 6:
            return False, "TCP offset needs six values"
        return self.script.send(
            f"set_tcp(p[{f(pose[0])},{f(pose[1])},{f(pose[2])},"
            f"{f(pose[3])},{f(pose[4])},{f(pose[5])}])")

    def set_digital_out(self, pin: int, value: bool) -> tuple[bool, str]:
        if not (0 <= int(pin) <= 7):
            return False, "standard digital output pin must be 0-7"
        return self.script.send(
            f"set_standard_digital_out({int(pin)}, {'True' if value else 'False'})")

    def set_tool_digital_out(self, pin: int, value: bool) -> tuple[bool, str]:
        if not (0 <= int(pin) <= 1):
            return False, "tool digital output pin must be 0 or 1"
        return self.script.send(
            f"set_tool_digital_out({int(pin)}, {'True' if value else 'False'})")

    def set_analog_out(self, pin: int, value: float) -> tuple[bool, str]:
        if not (0 <= int(pin) <= 1):
            return False, "analog output pin must be 0 or 1"
        v = max(0.0, min(1.0, float(value)))
        return self.script.send(f"set_standard_analog_out({int(pin)}, {f(v)})")

    def set_tool_voltage(self, volts: int) -> tuple[bool, str]:
        if int(volts) not in (0, 12, 24):
            return False, "tool voltage must be 0, 12 or 24"
        return self.script.send(f"set_tool_voltage({int(volts)})")

    def popup(self, text: str, title="SONAIR", warning=False) -> tuple[bool, str]:
        safe = str(text).replace('"', "'")[:200]
        return self.script.send(
            f'popup("{safe}", "{title}", {"True" if warning else "False"}, False, False)')

    def textmsg(self, text: str) -> tuple[bool, str]:
        return self.script.send(f'textmsg("{str(text)[:200]}")')

    def zero_ft_sensor(self) -> tuple[bool, str]:
        """
        Re-zero the wrist force/torque sensor.

        Do this with the tool hanging free and the payload already set, or you
        zero out gravity along with the offset and every later force reading is
        wrong by the weight of the tool.
        """
        return self.script.send("zero_ftsensor()")

    # --- dashboard -----------------------------------------------------------

    def power_on(self):        return self.dashboard.command("power on")
    def power_off(self):       return self.dashboard.command("power off")
    def brake_release(self):   return self.dashboard.command("brake release")
    def unlock_protective_stop(self): return self.dashboard.command("unlock protective stop")
    def close_safety_popup(self):     return self.dashboard.command("close safety popup")
    def close_popup(self):     return self.dashboard.command("close popup")
    def play(self):            return self.dashboard.command("play")
    def pause(self):           return self.dashboard.command("pause")
    def stop_program(self):    return self.dashboard.command("stop")
    def load_program(self, name): return self.dashboard.command(f"load {name}")
    def program_state(self):   return self.dashboard.command("programState")
    def robot_mode(self):      return self.dashboard.command("robotmode")
    def safety_status(self):   return self.dashboard.command("safetystatus")
    def get_loaded_program(self): return self.dashboard.command("get loaded program")
    def is_in_remote_control(self): return self.dashboard.command("is in remote control")
    def shutdown(self):        return self.dashboard.command("shutdown")

    def set_speed_slider(self, fraction: float) -> tuple[bool, str]:
        """
        Global speed scaling, 0.01-1.0.

        There is no URScript function for this — the speed slider is an RTDE
        INPUT (`speed_slider_mask` + `speed_slider_fraction`), which is a
        different channel from everything else in this class. The caller wires
        an RTDEInputChannel in via `attach_rtde_inputs()`; without one this
        returns a clear refusal rather than sending a script call that would
        fail on the controller with an obscure error.
        """
        v = max(0.01, min(1.0, float(fraction)))
        if self.rtde_inputs is None:
            return False, ("speed slider needs the RTDE input channel; "
                           "call attach_rtde_inputs() on the controller first")
        return self.rtde_inputs.set_speed_slider(v)

    def emergency_stop(self) -> tuple[bool, str]:
        """
        Software stop. Sends stopj on the script channel AND asks the dashboard
        to stop the program, because either channel alone can be blocked by the
        state the robot is in, and this is the one command that must land.
        """
        ok1, m1 = self.script.send("stopj(3.0)")
        ok2, m2 = self.dashboard.command("stop")
        return (ok1 or ok2), f"script: {m1 or 'sent'} | dashboard: {m2}"

    def close(self) -> None:
        self.script.close()


# =============================================================================
# Dispatch from the browser
# =============================================================================

def handle_command(ctrl: URController, data: dict) -> dict | None:
    """
    One place that maps a browser message to a robot action.

    Returns a reply dict, or None if the message is not a UR control message,
    so the bridge can chain this into its dispatch without a second table.
    """
    t = data.get("type")

    simple = {
        "ur_power_on": ctrl.power_on, "ur_power_off": ctrl.power_off,
        "ur_brake_release": ctrl.brake_release,
        "ur_unlock_protective_stop": ctrl.unlock_protective_stop,
        "ur_close_safety_popup": ctrl.close_safety_popup,
        "ur_close_popup": ctrl.close_popup,
        "ur_play": ctrl.play, "ur_pause": ctrl.pause, "ur_stop_program": ctrl.stop_program,
        "ur_program_state": ctrl.program_state, "ur_robot_mode": ctrl.robot_mode,
        "ur_safety_status": ctrl.safety_status,
        "ur_loaded_program": ctrl.get_loaded_program,
        "ur_remote_control": ctrl.is_in_remote_control,
        "ur_zero_ft": ctrl.zero_ft_sensor,
        "ur_estop": ctrl.emergency_stop,
        "ur_stop": ctrl.stop, "ur_stopj": ctrl.stopj,
    }
    if t in simple:
        ok, msg = simple[t]()
        return {"type": "ur_cmd_res", "cmd": t, "ok": ok, "msg": msg}

    if t == "ur_movel":
        ok, msg = ctrl.movel(data["pose"], data.get("a", 0.3), data.get("v", 0.1),
                             data.get("r", 0.0))
    elif t == "ur_movej":
        ok, msg = ctrl.movej(data.get("q") or data.get("pose"), data.get("a", 1.0),
                             data.get("v", 0.5), data.get("r", 0.0),
                             is_pose=bool(data.get("is_pose")))
    elif t == "ur_movep":
        ok, msg = ctrl.movep(data["pose"], data.get("a", 0.3), data.get("v", 0.1),
                             data.get("r", 0.01))
    elif t == "ur_servoj":
        ok, msg = ctrl.servoj(data["q"], data.get("t", 0.008),
                              data.get("lookahead", 0.1), data.get("gain", 300))
    elif t == "ur_speedl":
        ok, msg = ctrl.speedl(data["xd"], data.get("a", 0.5), data.get("t", 0.1))
    elif t == "ur_speedj":
        ok, msg = ctrl.speedj(data["qd"], data.get("a", 1.0), data.get("t", 0.1))
    elif t == "ur_freedrive":
        ok, msg = ctrl.freedrive(bool(data.get("enable")), data.get("axes"))
    elif t == "ur_set_payload":
        ok, msg = ctrl.set_payload(data.get("mass", 0.0), data.get("cog", (0, 0, 0)))
    elif t == "ur_set_tcp":
        ok, msg = ctrl.set_tcp(data["pose"])
    elif t == "ur_set_dout":
        ok, msg = ctrl.set_digital_out(data["pin"], bool(data["value"]))
    elif t == "ur_set_tool_dout":
        ok, msg = ctrl.set_tool_digital_out(data["pin"], bool(data["value"]))
    elif t == "ur_set_aout":
        ok, msg = ctrl.set_analog_out(data["pin"], data["value"])
    elif t == "ur_set_tool_voltage":
        ok, msg = ctrl.set_tool_voltage(data["volts"])
    elif t == "ur_popup":
        ok, msg = ctrl.popup(data.get("text", ""), data.get("title", "SONAIR"),
                             bool(data.get("warning")))
    elif t == "ur_speed_slider":
        ok, msg = ctrl.set_speed_slider(data.get("fraction", 1.0))
    elif t == "ur_load_program":
        ok, msg = ctrl.load_program(data.get("name", ""))
    elif t == "ur_script":
        script = data.get("script", "")
        if len(script) > 65536:
            ok, msg = False, "script too long (65 kB limit)"
        else:
            ok, msg = ctrl.script.send(script)
    else:
        return None

    return {"type": "ur_cmd_res", "cmd": t, "ok": ok, "msg": msg}
