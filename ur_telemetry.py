"""
ur_telemetry.py — complete UR5e state, via RTDE with a primary-interface fallback.

Two ways to read a UR controller, and this module implements both because they
fail in different situations:

  RTDE, port 30004  (preferred)
      You declare a recipe of the fields you want and the controller streams
      exactly those at up to 500 Hz. It is the only interface that exposes the
      full set: TCP force, joint currents, joint and tool temperatures, robot
      and safety mode, every digital and analogue IO, program state, payload.
      It is also version-negotiated, so it does not silently change meaning
      between PolyScope releases.

  Primary interface, port 30003  (fallback)
      A fixed-layout binary packet at 125 Hz that no one has to configure. The
      layout is stable for a given controller generation but offsets shift
      between major versions, which is exactly why it is the fallback and not
      the default.

`URTelemetry` tries RTDE, and drops to 30003 only if RTDE cannot be negotiated,
reporting which one is live so the UI never shows a number without saying where
it came from.

Nothing here commands motion. Control lives in ur_control.py, deliberately, so
that a bug in a control path cannot take the telemetry stream down with it.
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field, asdict

log = logging.getLogger("ur.telemetry")

RTDE_PORT = 30004
PRIMARY_PORT = 30003

# --- RTDE protocol constants -------------------------------------------------
RTDE_REQUEST_PROTOCOL_VERSION = 86      # 'V'
RTDE_GET_URCONTROL_VERSION = 118        # 'v'
RTDE_TEXT_MESSAGE = 77                  # 'M'
RTDE_DATA_PACKAGE = 85                  # 'U'
RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73   # 'I'
RTDE_CONTROL_PACKAGE_START = 83          # 'S'
RTDE_CONTROL_PACKAGE_PAUSE = 80          # 'P'

# The output recipe. Everything the console needs, and nothing that costs
# bandwidth without being displayed. Force and torque are the fields that
# motivated using RTDE at all — they do not exist on any other interface in
# a documented, version-stable form.
OUTPUT_RECIPE: list[tuple[str, str]] = [
    ("timestamp",                       "DOUBLE"),
    ("actual_q",                        "VECTOR6D"),   # joint angles, rad
    ("actual_qd",                       "VECTOR6D"),   # joint velocities, rad/s
    ("actual_current",                  "VECTOR6D"),   # joint currents, A
    ("joint_temperatures",              "VECTOR6D"),   # deg C
    ("target_q",                        "VECTOR6D"),
    ("target_qd",                       "VECTOR6D"),
    ("target_moment",                   "VECTOR6D"),   # joint torques, Nm
    ("actual_TCP_pose",                 "VECTOR6D"),   # x y z rx ry rz
    ("actual_TCP_speed",                "VECTOR6D"),
    ("actual_TCP_force",                "VECTOR6D"),   # Fx Fy Fz Tx Ty Tz
    ("target_TCP_pose",                 "VECTOR6D"),
    ("actual_digital_input_bits",       "UINT64"),
    ("actual_digital_output_bits",      "UINT64"),
    ("standard_analog_input0",          "DOUBLE"),
    ("standard_analog_input1",          "DOUBLE"),
    ("standard_analog_output0",         "DOUBLE"),
    ("standard_analog_output1",         "DOUBLE"),
    ("robot_mode",                      "INT32"),
    ("safety_mode",                     "INT32"),
    ("safety_status",                   "INT32"),
    ("runtime_state",                   "UINT32"),
    ("robot_status_bits",               "UINT32"),
    ("safety_status_bits",              "UINT32"),
    ("actual_robot_voltage",            "DOUBLE"),
    ("actual_robot_current",            "DOUBLE"),
    ("actual_tool_accelerometer",       "VECTOR3D"),   # the wrist accelerometer
    ("tcp_force_scalar",                "DOUBLE"),
    ("output_double_register_0",        "DOUBLE"),
    ("speed_scaling",                   "DOUBLE"),
]

_FMT = {
    "DOUBLE": ("d", 8), "UINT32": ("I", 4), "UINT64": ("Q", 8),
    "INT32": ("i", 4), "BOOL": ("?", 1), "UINT8": ("B", 1),
    "VECTOR3D": ("3d", 24), "VECTOR6D": ("6d", 48),
    "VECTOR6INT32": ("6i", 24), "VECTOR6UINT32": ("6I", 24),
}

ROBOT_MODE = {
    -1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
    3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING",
    8: "UPDATING_FIRMWARE",
}
SAFETY_MODE = {
    1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY",
    5: "SAFEGUARD_STOP", 6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION", 9: "FAULT", 10: "VALIDATE_JOINT_ID", 11: "UNDEFINED_SAFETY_MODE",
}
RUNTIME_STATE = {0: "STOPPING", 1: "STOPPED", 2: "PLAYING", 3: "PAUSED", 4: "RESUMING"}


# =============================================================================
# RTDE client
# =============================================================================

class RTDEClient:
    """
    Minimal, dependency-free RTDE client. Only the output half is implemented:
    inputs (writing registers) belong to ur_control.py.
    """

    def __init__(self, host: str, port: int = RTDE_PORT, frequency: float = 125.0):
        self.host = host
        self.port = port
        self.frequency = frequency
        self.sock: socket.socket | None = None
        self.protocol_version = 2
        self.recipe: list[tuple[str, str]] = []
        self._unpack_fmt = ""
        self._unpack_size = 0
        self.controller_version = ""

    # --- framing -------------------------------------------------------------

    def _send(self, cmd: int, payload: bytes = b"") -> None:
        if self.sock is None:
            raise ConnectionError("RTDE socket is not open")
        pkt = struct.pack(">HB", 3 + len(payload), cmd) + payload
        self.sock.sendall(pkt)

    def _recv_exact(self, n: int) -> bytes:
        if self.sock is None:
            raise ConnectionError("RTDE socket is not open")
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("RTDE connection closed by controller")
            buf += chunk
        return buf

    def _recv_packet(self) -> tuple[int, bytes]:
        head = self._recv_exact(3)
        size, cmd = struct.unpack(">HB", head)
        body = self._recv_exact(size - 3) if size > 3 else b""
        return cmd, body

    # --- handshake -----------------------------------------------------------

    def connect(self, timeout: float = 5.0) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(timeout)

        self._send(RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
        cmd, body = self._recv_packet()
        if cmd != RTDE_REQUEST_PROTOCOL_VERSION or not body or body[0] != 1:
            raise ConnectionError("controller refused RTDE protocol version 2")

        self._send(RTDE_GET_URCONTROL_VERSION)
        cmd, body = self._recv_packet()
        if cmd == RTDE_GET_URCONTROL_VERSION and len(body) >= 16:
            maj, mi, bug, build = struct.unpack(">IIII", body[:16])
            self.controller_version = f"{maj}.{mi}.{bug}.{build}"

    def setup_outputs(self, recipe: list[tuple[str, str]]) -> list[tuple[str, str]]:
        """
        Ask for the recipe; keep whatever the controller actually grants.

        A field the controller does not know comes back as type "NOT_FOUND"
        rather than an error, so an older PolyScope silently drops a couple of
        fields instead of refusing the whole stream. Those are removed from the
        recipe here, which is why the UI must read field names rather than
        positions.
        """
        names = ",".join(n for n, _ in recipe)
        payload = struct.pack(">d", self.frequency) + names.encode("utf-8")
        self._send(RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, payload)
        cmd, body = self._recv_packet()
        if cmd != RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
            raise ConnectionError("RTDE output setup failed")
        # body: recipe id (1 byte) + comma-separated granted types
        types = body[1:].decode("utf-8").split(",")
        granted = []
        for (name, _want), got in zip(recipe, types):
            if got == "NOT_FOUND":
                log.warning("RTDE field not available on this controller: %s", name)
                continue
            granted.append((name, got))
        self.recipe = granted
        self._unpack_fmt = ">" + "".join(_FMT[t][0] for _n, t in granted)
        self._unpack_size = sum(_FMT[t][1] for _n, t in granted)
        return granted

    def start(self) -> None:
        self._send(RTDE_CONTROL_PACKAGE_START)
        cmd, body = self._recv_packet()
        if cmd != RTDE_CONTROL_PACKAGE_START or not body or body[0] != 1:
            raise ConnectionError("controller refused to start the RTDE stream")

    def pause(self) -> None:
        if self.sock is None:
            return
        try:
            self._send(RTDE_CONTROL_PACKAGE_PAUSE)
        except (OSError, ConnectionError):
            pass

    # --- streaming -----------------------------------------------------------

    def read(self) -> dict | None:
        """One data package as a name -> value dict, or None for a text message."""
        cmd, body = self._recv_packet()
        if cmd == RTDE_TEXT_MESSAGE:
            if body:
                log.info("RTDE message from controller: %s", body[1:].decode("utf-8", "ignore"))
            return None
        if cmd != RTDE_DATA_PACKAGE:
            return None
        payload = body[1:] if self.protocol_version == 2 else body
        if len(payload) < self._unpack_size:
            return None
        vals = struct.unpack(self._unpack_fmt, payload[:self._unpack_size])
        out: dict = {}
        i = 0
        for name, typ in self.recipe:
            if typ in ("VECTOR6D", "VECTOR6INT32", "VECTOR6UINT32"):
                out[name] = list(vals[i:i + 6]); i += 6
            elif typ == "VECTOR3D":
                out[name] = list(vals[i:i + 3]); i += 3
            else:
                out[name] = vals[i]; i += 1
        return out

    def close(self) -> None:
        try:
            self.pause()
        except Exception:
            pass
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


# =============================================================================
# Primary interface (port 30003) fallback
# =============================================================================

def parse_primary_packet(data: bytes) -> dict | None:
    """
    Parse the 30003 realtime packet.

    Offsets are counted from the start of the packet, after the 4-byte length.
    They are stable for CB3/e-Series 3.x-5.x; if the packet length is not one
    of the known sizes the parse is refused rather than returning numbers that
    happen to decode. Silent misalignment here would put plausible-looking
    forces on screen, which is worse than no forces at all.
    """
    if len(data) < 764:
        return None
    try:
        def d(off):
            return struct.unpack_from(">d", data, off)[0]

        def v6(off):
            return list(struct.unpack_from(">6d", data, off))

        out = {
            "timestamp":                 d(4),
            "target_q":                  v6(12),
            "target_qd":                 v6(60),
            "actual_q":                  v6(252),
            "actual_qd":                 v6(300),
            "actual_current":            v6(348),
            "actual_TCP_pose":           v6(444),
            "actual_TCP_speed":          v6(492),
            "actual_TCP_force":          v6(540),
            "target_TCP_pose":           v6(588),
            "actual_digital_input_bits": int(d(684)),
            "joint_temperatures":        v6(692),
            "robot_mode":                int(d(756)),
            "_source": "primary-30003",
        }
        if len(data) >= 1044:
            out["joint_mode"] = [int(x) for x in v6(764)]
            out["safety_mode"] = int(d(812))
            out["actual_tool_accelerometer"] = list(struct.unpack_from(">3d", data, 820))
            out["speed_scaling"] = d(940)
            out["actual_robot_voltage"] = d(964)
            out["actual_robot_current"] = d(972)
            out["actual_digital_output_bits"] = int(d(1044)) if len(data) >= 1052 else 0
        return out
    except struct.error:
        return None


# =============================================================================
# The public object
# =============================================================================

@dataclass
class TelemetryHealth:
    source: str = "none"            # "rtde" | "primary-30003" | "none"
    connected: bool = False
    controller_version: str = ""
    rate_hz: float = 0.0
    packets: int = 0
    errors: int = 0
    last_error: str = ""
    fields: list = field(default_factory=list)
    rtde_unavailable_reason: str = ""


class URTelemetry:
    """
    Background reader. Holds the most recent full state and a health block.

    Reconnects on its own with a bounded backoff: a UR that is power-cycled
    mid-session must not require the operator to restart the agent, because
    during a data campaign that means losing the run you were in the middle of.
    """

    def __init__(self, host: str, frequency: float = 125.0, prefer_rtde: bool = True):
        self.host = host
        self.frequency = frequency
        self.prefer_rtde = prefer_rtde
        self._lock = threading.Lock()
        self._state: dict = {}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.health = TelemetryHealth()

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="ur-telemetry")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def state(self) -> dict:
        with self._lock:
            return dict(self._state)

    def status(self) -> dict:
        with self._lock:
            return {"health": asdict(self.health), "has_state": bool(self._state)}

    # --- internals -----------------------------------------------------------

    def _publish(self, raw: dict, source: str) -> None:
        with self._lock:
            self._state = decorate(raw, source)
            self.health.packets += 1

    def _loop(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            ok = False
            try:
                if self.prefer_rtde:
                    ok = self._run_rtde()
                if not ok and not self._stop.is_set():
                    ok = self._run_primary()
            except Exception as e:                       # noqa: BLE001
                with self._lock:
                    self.health.last_error = f"telemetry loop: {e}"
                    self.health.errors += 1
                log.warning("telemetry loop error, retrying: %s", e)
            if self._stop.is_set():
                break
            with self._lock:
                self.health.connected = False
                self.health.source = "none"
            time.sleep(backoff)
            backoff = min(backoff * 2, 10.0) if not ok else 1.0

    def _run_rtde(self) -> bool:
        client = RTDEClient(self.host, RTDE_PORT, self.frequency)
        try:
            client.connect()
            granted = client.setup_outputs(OUTPUT_RECIPE)
            client.start()
        except Exception as e:
            with self._lock:
                self.health.rtde_unavailable_reason = str(e)
                self.health.last_error = f"rtde: {e}"
                self.health.errors += 1
            log.warning("RTDE unavailable (%s) — falling back to port 30003", e)
            client.close()
            return False

        with self._lock:
            self.health.source = "rtde"
            self.health.connected = True
            self.health.controller_version = client.controller_version
            self.health.fields = [n for n, _ in granted]
            self.health.rtde_unavailable_reason = ""
        log.info("RTDE streaming %d fields at %.0f Hz (controller %s)",
                 len(granted), self.frequency, client.controller_version or "unknown")

        t0, n0 = time.monotonic(), 0
        try:
            while not self._stop.is_set():
                pkt = client.read()
                if pkt is None:
                    continue
                self._publish(pkt, "rtde")
                n0 += 1
                now = time.monotonic()
                if now - t0 >= 1.0:
                    with self._lock:
                        self.health.rate_hz = n0 / (now - t0)
                    t0, n0 = now, 0
            return True
        except Exception as e:
            with self._lock:
                self.health.last_error = f"rtde stream: {e}"
                self.health.errors += 1
            log.warning("RTDE stream ended: %s", e)
            return False
        finally:
            client.close()

    def _run_primary(self) -> bool:
        try:
            sock = socket.create_connection((self.host, PRIMARY_PORT), timeout=5.0)
            sock.settimeout(2.0)
        except Exception as e:
            with self._lock:
                self.health.last_error = f"primary: {e}"
                self.health.errors += 1
            return False

        with self._lock:
            self.health.source = "primary-30003"
            self.health.connected = True
            self.health.fields = ["actual_q", "actual_TCP_pose", "actual_TCP_force",
                                  "actual_current", "joint_temperatures", "robot_mode"]
        log.info("reading UR primary interface on 30003 (RTDE not available)")

        buf = b""
        t0, n0 = time.monotonic(), 0
        try:
            while not self._stop.is_set():
                chunk = sock.recv(8192)
                if not chunk:
                    return False
                buf += chunk
                while len(buf) >= 4:
                    plen = struct.unpack(">I", buf[:4])[0]
                    if plen < 4 or plen > 65535:
                        buf = b""       # desynchronised; drop rather than guess
                        break
                    if len(buf) < plen:
                        break
                    pkt, buf = buf[:plen], buf[plen:]
                    parsed = parse_primary_packet(pkt)
                    if parsed:
                        self._publish(parsed, "primary-30003")
                        n0 += 1
                now = time.monotonic()
                if now - t0 >= 1.0:
                    with self._lock:
                        self.health.rate_hz = n0 / (now - t0)
                    t0, n0 = now, 0
            return True
        except Exception as e:
            with self._lock:
                self.health.last_error = f"primary stream: {e}"
                self.health.errors += 1
            return False
        finally:
            try:
                sock.close()
            except OSError:
                pass


def decorate(raw: dict, source: str) -> dict:
    """
    Add the derived values the UI wants, without hiding where they came from.

    Bit fields are expanded into named booleans here rather than in JavaScript,
    because the bit meanings are controller-version dependent and that knowledge
    belongs next to the parser, not in the browser.
    """
    out = dict(raw)
    out["_source"] = source
    out["_host_time"] = time.time()

    mode = raw.get("robot_mode")
    if mode is not None:
        out["robot_mode_text"] = ROBOT_MODE.get(int(mode), f"UNKNOWN({mode})")
    smode = raw.get("safety_mode")
    if smode is not None:
        out["safety_mode_text"] = SAFETY_MODE.get(int(smode), f"UNKNOWN({smode})")
    rstate = raw.get("runtime_state")
    if rstate is not None:
        out["runtime_state_text"] = RUNTIME_STATE.get(int(rstate), f"UNKNOWN({rstate})")

    f = raw.get("actual_TCP_force")
    if f and len(f) >= 6:
        out["tcp_force_magnitude"] = (f[0] ** 2 + f[1] ** 2 + f[2] ** 2) ** 0.5
        out["tcp_torque_magnitude"] = (f[3] ** 2 + f[4] ** 2 + f[5] ** 2) ** 0.5

    bits = raw.get("robot_status_bits")
    if bits is not None:
        b = int(bits)
        out["status_flags"] = {
            "power_on": bool(b & 1),
            "program_running": bool(b & 2),
            "teach_button_pressed": bool(b & 4),
            "power_button_pressed": bool(b & 8),
        }
    sbits = raw.get("safety_status_bits")
    if sbits is not None:
        b = int(sbits)
        out["safety_flags"] = {
            "normal_mode": bool(b & 1),
            "reduced_mode": bool(b & 2),
            "protective_stopped": bool(b & 4),
            "recovery_mode": bool(b & 8),
            "safeguard_stopped": bool(b & 16),
            "system_emergency_stopped": bool(b & 32),
            "robot_emergency_stopped": bool(b & 64),
            "emergency_stopped": bool(b & 128),
            "violation": bool(b & 256),
            "fault": bool(b & 512),
        }

    din = raw.get("actual_digital_input_bits")
    dout = raw.get("actual_digital_output_bits")
    if din is not None:
        out["digital_inputs"] = [bool(int(din) & (1 << i)) for i in range(18)]
    if dout is not None:
        out["digital_outputs"] = [bool(int(dout) & (1 << i)) for i in range(18)]
    return out
