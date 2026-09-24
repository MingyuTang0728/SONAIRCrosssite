"""
multimodal_bridge.py — SONAIR UR Host Agent
==============================================

Runs on the UoN workstation that is physically wired to the UR5e.
Sole responsibility: be the trusted bridge between the cloud relay
and the physical robot.

Two faces:
  1. LOCAL face (port 8765): the UoN operator's browser, on the same
     LAN, connects here for low-latency control. This is the original
     behaviour, preserved for backwards compatibility and for the
     case where the relay is unreachable (e.g. ISP outage).

  2. RELAY face (outbound wss): the agent dials out to the SONAIR
     cloud relay. Once connected, it identifies as the agent for a
     specific 'room' (typically "uon-cell-1"). The relay then forwards
     it commands from any operators who are bound to that room.

Why outbound-only to the relay?
  University firewalls almost universally allow outbound 443/wss but
  block inbound. Reversing the topology lets us deploy without IT
  involvement and avoids exposing the robot cell to the public
  internet directly.

Trust model:
  - The agent only accepts motion commands that arrive over an
    AUTHENTICATED relay connection. The relay has already arbitrated
    authority and applied envelope/vcap checks, but we re-check
    locally as a final safety net.
  - The local 8765 port is treated as "cell-trusted" — same LAN as
    the robot; if someone has access to it they're already inside the
    physical safety perimeter.

Author: Mingyu Tang, University of Nottingham
"""
import asyncio
import websockets
import json
import base64
import os
import time
import socket
import struct
import sys
import threading
import traceback
from collections import deque
import logging
from ftplib import FTP
from datetime import datetime, timezone
from pathlib import Path

# Optional vision deps — keep import-time tolerant so the agent still
# boots on a developer laptop without the RealSense SDK.
try:
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    _HAS_VISION = True
    _VISION_ERR = ""
except ImportError as _ve:
    _HAS_VISION = False
    _VISION_ERR = str(_ve)
    print("[agent] WARNING: vision deps missing (cv2/numpy/pyrealsense2). "
          "Camera streaming disabled.")

# Benchmark acquisition — inertial ingestion, the time master, run recording.
# Kept in its own module so the bridge stays responsible only for the robot.
try:
    import camera_negotiate
except Exception:                # noqa: BLE001
    camera_negotiate = None

try:
    import vision_inspect
    _HAS_VISINSP = True
except Exception as _vi:         # noqa: BLE001
    _HAS_VISINSP = False
    vision_inspect = None
    print(f"[agent] WARNING: vision_inspect unavailable ({_vi}).")

try:
    import camera_service
    _HAS_CAMSVC = True
except Exception as _ce:         # noqa: BLE001
    _HAS_CAMSVC = False
    camera_service = None
    print(f"[agent] WARNING: camera_service unavailable ({_ce}).")

try:
    import ur_bridge_ext
    _HAS_EXT = True
except Exception as _e:          # noqa: BLE001
    _HAS_EXT = False
    ur_bridge_ext = None
    print(f"[agent] WARNING: ur_bridge_ext unavailable ({_e}); "
          "full UR telemetry, control and 3D scanning disabled.")

try:
    import handeye
    _HAS_HANDEYE = True
except Exception as _he:         # noqa: BLE001
    _HAS_HANDEYE = False
    handeye = None
    print(f"[agent] WARNING: handeye unavailable ({_he}).")

try:
    import multiview
    _HAS_MV = True
except Exception as _mv:         # noqa: BLE001
    _HAS_MV = False
    multiview = None
    print(f"[agent] WARNING: multiview unavailable ({_mv}).")

try:
    import rs_features
    _HAS_RSF = True
except Exception as _rf:         # noqa: BLE001
    _HAS_RSF = False
    rs_features = None
    print(f"[agent] WARNING: rs_features unavailable ({_rf}).")

try:
    import sensor_hub
    _HAS_SENSORS = True
except Exception as _sh:         # noqa: BLE001
    _HAS_SENSORS = False
    sensor_hub = None
    print(f"[agent] WARNING: sensor_hub unavailable ({_sh}).")

try:
    import bench_agent
    _HAS_BENCH = True
except Exception as _e:          # noqa: BLE001 - never block the robot on this
    _HAS_BENCH = False
    bench_agent = None
    print(f"[agent] WARNING: bench_agent unavailable ({_e}); "
          "benchmark recording disabled.")


# ============================================================
# Configuration
# ============================================================
UR_IP            = os.environ.get("UR_IP", "192.168.0.20")

# ---------------------------------------------------------------------------
# THE ROBOT'S ADDRESS, in one place.
#
# There are five separate channels to a UR and they used to disagree about
# where the robot was. Telemetry followed the address typed into the console;
# the dashboard (29999), the URScript channel (30002), the program list (FTP)
# and the realtime reader (30003) all kept whatever UR_IP held at import time.
# Unless the robot happened to sit at the compiled-in default, that meant
# joint angles streamed perfectly while power on, brake release, load
# program, play, stop, freedrive and every I/O control quietly did nothing --
# each one timing out against an address nobody had typed. The symptom is a
# console that looks connected and is half dead, and nothing on screen says
# which half.
#
# One setter now moves all of them. Changing it bumps a generation counter and
# drops the long-lived sockets, so the reader and command threads notice on
# their next pass and reconnect to the new address rather than holding a
# connection to the old one until something times out.
# ---------------------------------------------------------------------------
_ur_addr_lock = threading.Lock()
_ur_addr_gen  = 0


def robot_host() -> str:
    """The address every UR channel must use. Read it, never cache it."""
    with _ur_addr_lock:
        return UR_IP


def robot_addr_gen() -> int:
    with _ur_addr_lock:
        return _ur_addr_gen


def set_robot_host(host: str) -> dict:
    """Point every channel at `host`. Returns what changed, for the log."""
    global UR_IP, _ur_addr_gen
    host = str(host or "").strip()
    if not host:
        return {"changed": False, "host": robot_host(), "error": "no address given"}
    with _ur_addr_lock:
        if host == UR_IP:
            return {"changed": False, "host": host}
        was, UR_IP = UR_IP, host
        _ur_addr_gen += 1
        gen = _ur_addr_gen
    log.info("robot address changed: %s -> %s (generation %d)", was, host, gen)
    # Drop the sockets that are pinned to the old address. Each owning loop
    # reconnects on its own; closing from here is what makes it notice now
    # rather than at the end of a 30 s receive timeout.
    for sock in (ur_socket_tx, ur_control_tx):
        try:
            if sock is not None:
                sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            if sock is not None:
                sock.close()
        except Exception:
            pass
    return {"changed": True, "host": host, "was": was, "generation": gen}

# Local face — UoN operator's browser on same LAN
LOCAL_HOST       = "0.0.0.0"
LOCAL_PORT       = int(os.environ.get("AGENT_LOCAL_PORT", 8765))

# Relay face — outbound connection to SONAIR cloud
# Set to e.g. wss://relay.sonair.uon.ac.uk/ws  (production)
#           or ws://localhost:8770             (local dev)
# Empty string = local-only mode (no cross-site teleop)
RELAY_URL         = os.environ.get("RELAY_URL", "")
RELAY_AGENT_TOKEN = os.environ.get("RELAY_AGENT_TOKEN", "agent-default-replace-me")
RELAY_ROOM        = os.environ.get("RELAY_ROOM", "uon-cell-1")
RELAY_SITE        = os.environ.get("RELAY_SITE", "UoN")

# Local agent-side workspace envelope (final safety net)
ENVELOPE = {
    "x_min": -0.6, "x_max": 0.6,
    "y_min": -0.6, "y_max": 0.6,
    "z_min":  0.05, "z_max": 0.7,
}

# Benchmark acquisition
BENCH_RUN_DIR      = os.environ.get("BENCH_RUN_DIR", "./bench_runs")
BENCH_FUSIONHUB_PORT = int(os.environ.get("BENCH_FUSIONHUB_PORT", 5005))
BENCH_ENABLE_D435I_IMU = os.environ.get("BENCH_D435I_IMU", "1") != "0"
BENCH_ENABLE_FUSIONHUB = os.environ.get("BENCH_FUSIONHUB", "1") != "0"

# Audit log
AUDIT_DIR = Path(os.environ.get("AGENT_AUDIT_DIR", "./agent_audit"))
AUDIT_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# Logging
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("agent")

try:
    import automation
    _HAS_AUTO = True
    _AUTO_ERR = ""
except Exception as e:      # noqa: BLE001
    automation = None
    _HAS_AUTO = False
    _AUTO_ERR = str(e)


def f5(val):
    """Format a float with no scientific notation — URScript can't parse 1e-5."""
    return f"{float(val):.5f}"


def audit(event_kind, payload=None):
    rec = {
        "ts":      datetime.now(timezone.utc).isoformat(),
        "kind":    event_kind,
        "payload": payload or {},
    }
    fname = AUDIT_DIR / f"audit_{datetime.utcnow():%Y%m%d}.jsonl"
    try:
        with open(fname, "a", encoding="utf-8") as fp:
            fp.write(json.dumps(rec, default=str) + "\n")
    except Exception as e:
        log.warning("audit write failed: %s", e)


# ============================================================
# Global UR state (shared between threads)
# ============================================================
global_rgb_frame   = None
global_depth_frame = None   # D435i aligned depth (colormap, for display)
global_depth_raw   = None   # D435i aligned depth, RAW uint16 — metric, for scan3d
global_depth_intr  = None   # CameraIntrinsics read FROM the camera, never guessed
global_color_intr  = None   # same grid: depth is aligned to colour before use
_CAMERA_NOTES      = []     # mode substitutions the negotiator had to make
_CAMERA_LAST_ERROR = ""     # last startup failure, already explained
global_ir1_frame   = None   # D435i left IR
global_ir2_frame   = None   # D435i right IR
global_frame_meta  = {}     # per-frame timestamps and the clock domain
global_rs_profile  = None   # live pipeline profile, for extrinsics
_FILTERS           = None   # rs_features.FilterChain, built on pipeline start
global_actual_q    = [0.0] * 6
global_tcp_pose    = [0.0] * 6
camera_lock        = threading.Lock()
data_lock          = threading.Lock()
ur_socket_tx       = None  # 30003 long-lived status socket (RX only)
ur_control_tx      = None  # 30002 long-lived command socket (TX for speedl/speedj)
ur_control_lock    = threading.Lock()

# RealSense live camera config — written by camera_config messages,
# read by camera_thread on the next pipeline restart.
_rs_config_lock  = threading.Lock()
_rs_config       = {
    "stereo_res":   "640x480",
    "stereo_fps":   30,
    "depth_en":     True,
    # The infrared pair is what depth is COMPUTED from, and it is the only
    # way to see why depth is missing. It ships on.
    "ir1_en":       True,
    "ir2_en":       True,
    "emitter":      "laser",
    "depth_ae":     True,
    "depth_exp":    8500,
    "depth_gain":   16,
    "post_proc":    True,
    "colormap":     2,
    "depth_units":  0.001,
    # 1280x720, not 640x360. The colour stream is what the calibration board
    # is detected in, and detection needs enough pixels across one square: a
    # 7.5 mm square at 400 mm standoff lands about 11 px across at 640x360 and
    # about 23 px at 1280x720. Below roughly 15 px the corner detector stops
    # finding dense boards at all, which presents as "board not detected" with
    # a board that is plainly in shot and perfectly in focus.
    "rgb_res":      "1280x720",
    "rgb_fps":      30,
    "rgb_en":       True,
    "rgb_ae":       True,
    "rgb_exp":      166,
    "rgb_gain":     64,
    "rgb_brightness": 0,
    "rgb_contrast": 50,
    "rgb_sharpness": 50,
    "rgb_white_balance": 4600,
    "rgb_wb_auto":  True,
    "show_rgb":     True,
    "show_depth":   True,
    "show_ir1":     True,
    "show_ir2":     True,
    # Post-processing is now a named chain with real parameters (rs_features).
    # "post_proc" above stays as the master switch so old clients still work.
    "filters":      {},
}
_rs_restart_evt  = threading.Event()  # set -> camera_thread restarts pipeline


# ============================================================
# UR low-level comms
# ============================================================
# ---------------------------------------------------------------------------
# FAULTS, kept where someone can see them.
#
# An agent that dies quietly is an agent that gets blamed for the wrong thing.
# Every exception that would otherwise vanish into a log file nobody has open
# lands here, with its traceback, and the console can ask for the list. That
# turns "it disconnected and I do not know why" -- which costs a round trip
# and a guess -- into a line of text naming the message that did it.
# ---------------------------------------------------------------------------
_FAULTS: "deque" = deque(maxlen=60)
_fault_lock = threading.Lock()


def record_fault(where: str, exc: BaseException, context=None) -> dict:
    entry = {
        "at": time.strftime("%H:%M:%S"),
        "where": where,
        "error": f"{type(exc).__name__}: {exc}",
        "context": context,
        "traceback": traceback.format_exc(limit=8),
    }
    with _fault_lock:
        _FAULTS.append(entry)
    log.error("fault in %s (%s): %s", where, context, entry["error"])
    log.debug("%s", entry["traceback"])
    return entry


def faults() -> list:
    with _fault_lock:
        return list(_FAULTS)


def send_dashboard_cmd(cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((robot_host(), 29999))
            s.recv(1024)
            s.sendall((cmd + "\n").encode("utf-8"))
            res = s.recv(1024).decode("utf-8").strip()
            log.info("dashboard ← %s", res)
            return res
    except Exception as e:
        return f"Error: {e}"


def send_urscript_to_robot(script_content, stop_first=False):
    """Inject URScript via port 30002.
    Always newline-terminate short URScript commands; otherwise UR may buffer
    them and movement appears to do nothing.
    """
    try:
        if not script_content.endswith("\n"):
            script_content += "\n"
        if stop_first:
            send_dashboard_cmd("stop")
            time.sleep(0.05)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((robot_host(), 30002))
            s.sendall(script_content.encode("utf-8"))
        log.info("urscript injected (%d bytes)", len(script_content))
        return True
    except Exception as e:
        log.error("urscript inject failed: %s", e)
        return False


def send_realtime_script(script_content):
    """Send small motion URScript through the persistent 30002 command socket.

    Important: 30003 is used as the real-time state stream only. On UR/e-Series,
    writing speedl/speedj back to the same 30003 socket is unreliable: the
    browser may show commands being sent while the arm does not move.
    """
    global ur_control_tx
    if not script_content.endswith("\n"):
        script_content += "\n"

    with ur_control_lock:
        s = ur_control_tx
        if s is not None:
            try:
                s.sendall(script_content.encode("utf-8"))
                return True
            except Exception as e:
                log.warning("30002 command socket failed: %s — reconnecting", e)
                try:
                    s.close()
                except Exception:
                    pass
                ur_control_tx = None

    # Safe fallback for occasional commands while the persistent command socket
    # is reconnecting. High-rate joystick resumes smoothly after reconnect.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as one:
            one.settimeout(0.35)
            one.connect((robot_host(), 30002))
            one.sendall(script_content.encode("utf-8"))
        return True
    except Exception as e:
        log.warning("realtime command DROPPED: 30002 unavailable (%s)", e)
        return False


def ur_control_thread():
    """Maintain a long-lived 30002 command socket for smooth speedl/speedj."""
    global ur_control_tx
    while True:
        s = None
        try:
            host = robot_host()
            gen = robot_addr_gen()
            log.info("connecting UR 30002 command @ %s ...", host)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((host, 30002))
            s.settimeout(None)
            with ur_control_lock:
                old = ur_control_tx
                ur_control_tx = s
                if old is not None and old is not s:
                    try: old.close()
                    except Exception: pass
            log.info("UR 30002 command connected")
            while True:
                time.sleep(1.0)
                # An address change invalidates this socket even though it is
                # still perfectly healthy: it is healthy to the WRONG robot.
                if robot_addr_gen() != gen:
                    log.info("robot address changed — dropping 30002 to %s", host)
                    break
                try:
                    s.sendall(b"# keepalive\n")
                except Exception:
                    break
        except Exception as e:
            log.warning("UR 30002 command error: %s", e)
        finally:
            with ur_control_lock:
                if ur_control_tx is s:
                    ur_control_tx = None
            try:
                if s is not None: s.close()
            except Exception:
                pass
            time.sleep(1.0)


def fetch_urp_list():
    try:
        ftp = FTP(robot_host(), timeout=3)
        ftp.login()
        ftp.cwd("/programs")
        files = ftp.nlst()
        ftp.quit()
        return [f for f in files if f.endswith(".urp")]
    except Exception:
        return []


def estop_ur():
    log.warning("E-STOP")
    send_dashboard_cmd("stop")
    audit("estop_executed")


def envelope_accepts(pose):
    if not pose or len(pose) < 3:
        return True
    x, y, z = pose[0], pose[1], pose[2]
    return (ENVELOPE["x_min"] <= x <= ENVELOPE["x_max"]
            and ENVELOPE["y_min"] <= y <= ENVELOPE["y_max"]
            and ENVELOPE["z_min"] <= z <= ENVELOPE["z_max"])


def ur_io_thread():
    """Keep 30003 long-lived; parse real-time status; update globals.
    Supports both CB3 (1220 bytes) and e-Series UR5e (1116 bytes).
    Offsets for q_actual (252) and tcp_actual (444) are identical on both.
    """
    global global_actual_q, global_tcp_pose, ur_socket_tx
    _logged_size = set()   # avoid spamming log with unknown packet sizes
    while True:
        try:
            host = robot_host()
            gen = robot_addr_gen()
            log.info("connecting UR 30003 @ %s ...", host)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)          # generous connect timeout
            s.connect((host, 30003))
            s.settimeout(30.0)         # recv timeout — long enough to survive brief UR pauses
            ur_socket_tx = s
            log.info("UR 30003 connected")
            buffer = b""
            while True:
                if robot_addr_gen() != gen:
                    log.info("robot address changed — dropping 30003 to %s", host)
                    break
                chunk = s.recv(4096)
                if not chunk:
                    log.warning("UR 30003 closed by robot (empty recv)")
                    break
                buffer += chunk
                while len(buffer) >= 4:
                    packet_len = struct.unpack("!i", buffer[0:4])[0]
                    # sanity check — avoid runaway on corrupt data
                    if packet_len <= 0 or packet_len > 8192:
                        buffer = b""
                        break
                    if len(buffer) >= packet_len:
                        packet = buffer[:packet_len]
                        buffer = buffer[packet_len:]
                        # UR5e e-Series: 1116 bytes
                        # UR CB3:         1220 bytes
                        # Both have q_actual at byte 252, tcp_actual at byte 444
                        if packet_len in (1116, 1220):
                            try:
                                q   = list(struct.unpack("!6d", packet[252:300]))
                                tcp = list(struct.unpack("!6d", packet[444:492]))
                                with data_lock:
                                    global_actual_q = q
                                    global_tcp_pose = tcp
                            except struct.error:
                                pass
                        else:
                            if packet_len not in _logged_size:
                                log.debug("UR 30003 unknown packet size %d (not 1116/1220)", packet_len)
                                _logged_size.add(packet_len)
                    else:
                        break
        except socket.timeout:
            log.warning("UR 30003 recv timeout — UR may be in protective stop or local mode")
            ur_socket_tx = None
            time.sleep(2)
        except Exception as e:
            log.warning("UR 30003 error: %s", e)
            ur_socket_tx = None
            time.sleep(2)


# ============================================================
# RealSense
# ============================================================
def _parse_res(res_str):
    """'640x480' -> (640, 480)"""
    w, h = res_str.split("x")
    return int(w), int(h)


def _apply_sensor_options(sensor, ae, exp, gain, **kwargs):
    """Safely set exposure/gain options on a RealSense sensor."""
    try:
        if ae:
            sensor.set_option(rs.option.enable_auto_exposure, 1)
        else:
            sensor.set_option(rs.option.enable_auto_exposure, 0)
            sensor.set_option(rs.option.exposure, exp)
            sensor.set_option(rs.option.gain, gain)
    except Exception as e:
        log.debug("sensor option error: %s", e)


def camera_thread():
    """
    D435i capture thread.  Re-reads _rs_config on every pipeline (re-)start.
    Set _rs_restart_evt to hot-reload the config without killing the thread.

    Streams kept in globals (all uint8 BGR numpy arrays):
      global_rgb_frame   — color
      global_depth_frame — depth (colourised)
      global_ir1_frame   — left IR  (BGR converted from Y8)
      global_ir2_frame   — right IR (BGR converted from Y8)
    """
    if not _HAS_VISION:
        return
    global global_rgb_frame, global_depth_frame, global_ir1_frame, global_ir2_frame
    global global_depth_raw, global_depth_intr, global_color_intr, global_frame_meta

    while True:  # outer loop: restart pipeline on config change
        _rs_restart_evt.clear()

        # --- snapshot config under lock ---
        with _rs_config_lock:
            cfg_snap = dict(_rs_config)

        sw, sh  = _parse_res(cfg_snap["stereo_res"])
        sfps    = cfg_snap["stereo_fps"]
        rw, rh  = _parse_res(cfg_snap["rgb_res"])
        rfps    = cfg_snap["rgb_fps"]

        pipeline = rs.pipeline()
        rscfg    = rs.config()

        if cfg_snap["depth_en"]:
            rscfg.enable_stream(rs.stream.depth, sw, sh, rs.format.z16, sfps)
        if cfg_snap["ir1_en"]:
            rscfg.enable_stream(rs.stream.infrared, 1, sw, sh, rs.format.y8, sfps)
        if cfg_snap["ir2_en"]:
            rscfg.enable_stream(rs.stream.infrared, 2, sw, sh, rs.format.y8, sfps)
        if cfg_snap["rgb_en"]:
            rscfg.enable_stream(rs.stream.color, rw, rh, rs.format.bgr8, rfps)

        # Ask the device what it supports BEFORE starting, and substitute the
        # nearest workable mode. Without this, one unsupported triple fails the
        # whole config and takes depth and colour down with it — reported only
        # as "Couldn't resolve requests", which names neither the stream nor a
        # way out.
        _neg_notes = []
        try:
            _devs = list(rs.context().query_devices())
            if _devs and camera_negotiate is not None:
                cfg_snap, _neg_notes = camera_negotiate.negotiate(_devs[0], cfg_snap)
                if _neg_notes:
                    for _n in _neg_notes:
                        log.warning("camera: %s", _n)
                    sw, sh = _parse_res(cfg_snap["stereo_res"])
                    sfps   = cfg_snap["stereo_fps"]
                    rw, rh = _parse_res(cfg_snap["rgb_res"])
                    rfps   = cfg_snap["rgb_fps"]
                    rscfg = rs.config()
                    if cfg_snap["depth_en"]:
                        rscfg.enable_stream(rs.stream.depth, sw, sh, rs.format.z16, sfps)
                    if cfg_snap["ir1_en"]:
                        rscfg.enable_stream(rs.stream.infrared, 1, sw, sh, rs.format.y8, sfps)
                    if cfg_snap["ir2_en"]:
                        rscfg.enable_stream(rs.stream.infrared, 2, sw, sh, rs.format.y8, sfps)
                    if cfg_snap["rgb_en"]:
                        rscfg.enable_stream(rs.stream.color, rw, rh, rs.format.bgr8, rfps)
        except Exception as _e:
            log.debug("mode negotiation skipped: %s", _e)
        global _CAMERA_NOTES
        _CAMERA_NOTES = list(_neg_notes)

        align      = rs.align(rs.stream.color)
        colorizer  = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, float(cfg_snap["colormap"]))

        # Post-processing is Intel's full chain with its real parameters, in
        # Intel's order and through disparity space — see rs_features. The old
        # three hardcoded filters silently ran in depth space, which changes
        # what they do to the far field.
        global _FILTERS
        if _HAS_RSF:
            fcfg = dict(cfg_snap.get("filters") or {})
            fcfg.setdefault("enabled", bool(cfg_snap.get("post_proc", True)))
            _FILTERS = rs_features.FilterChain(fcfg)
        else:
            _FILTERS = None
            spatial  = rs.spatial_filter()
            temporal = rs.temporal_filter()
            hole_fill = rs.hole_filling_filter()

        try:
            profile = pipeline.start(rscfg)
            global global_rs_profile
            global_rs_profile = profile
            dev     = profile.get_device()
            serial  = dev.get_info(rs.camera_info.serial_number)

            # Intrinsics come from the camera itself. Every 3D measurement
            # downstream is scaled by these, and a datasheet value for "the
            # D435i" is wrong for any individual unit.
            try:
                depth_sensor_for_scale = dev.first_depth_sensor()
                _scale = depth_sensor_for_scale.get_depth_scale()
                # THE COLOUR intrinsics, not the depth ones.
                #
                # Every depth frame here is passed through align(stream.color)
                # before anything touches it, so the array called "depth" is
                # on the COLOUR image grid and carries the COLOUR intrinsics.
                # Reading the unaligned depth profile instead gave fx from a
                # 640x480 depth stream for pixels measured on a 1280x720
                # colour image -- roughly half the true focal length -- which
                # scales every deprojected point, every board pose and the
                # hand-eye answer that follows from them by that same factor,
                # consistently enough that nothing ever looks wrong.
                _csp = profile.get_stream(rs.stream.color).as_video_stream_profile() \
                    if cfg_snap["rgb_en"] else None
                _dsp = profile.get_stream(rs.stream.depth).as_video_stream_profile() \
                    if cfg_snap["depth_en"] else None
                _vsp = _csp or _dsp
                _i = _vsp.get_intrinsics()
                if _HAS_EXT:
                    from scan3d import CameraIntrinsics
                    global_depth_intr = CameraIntrinsics.from_realsense(_i, _scale)
                    global_color_intr = global_depth_intr
                    log.info("intrinsics (%s, aligned grid) fx=%.1f fy=%.1f "
                             "cx=%.1f cy=%.1f %dx%d scale=%.6f",
                             "colour" if _csp else "depth",
                             _i.fx, _i.fy, _i.ppx, _i.ppy,
                             _i.width, _i.height, _scale)
            except Exception as e:
                log.warning("could not read intrinsics: %s — "
                            "3D reconstruction will refuse to start", e)
            log.info("RealSense D435i started  serial=%s  stereo=%dx%d@%d  rgb=%dx%d@%d",
                     serial, sw, sh, sfps, rw, rh, rfps)

            # --- Emitter ---
            depth_sensor = dev.first_depth_sensor()
            try:
                depth_sensor.set_option(
                    rs.option.emitter_enabled,
                    1 if cfg_snap["emitter"] == "laser" else 0
                )
                depth_sensor.set_option(
                    rs.option.depth_units, cfg_snap["depth_units"]
                )
            except Exception as e:
                log.debug("emitter/depth_units option: %s", e)

            # --- Depth sensor exposure/gain ---
            _apply_sensor_options(
                depth_sensor,
                cfg_snap["depth_ae"],
                cfg_snap["depth_exp"],
                cfg_snap["depth_gain"],
            )

            # --- RGB sensor exposure/gain/wb ---
            try:
                rgb_sensor = dev.query_sensors()[1]  # index 1 is colour sensor
                _apply_sensor_options(
                    rgb_sensor,
                    cfg_snap["rgb_ae"],
                    cfg_snap["rgb_exp"],
                    cfg_snap["rgb_gain"],
                )
                if cfg_snap["rgb_wb_auto"]:
                    rgb_sensor.set_option(rs.option.enable_auto_white_balance, 1)
                else:
                    rgb_sensor.set_option(rs.option.enable_auto_white_balance, 0)
                    rgb_sensor.set_option(rs.option.white_balance,
                                          cfg_snap["rgb_white_balance"])
                for opt, key in [(rs.option.brightness,  "rgb_brightness"),
                                 (rs.option.contrast,    "rgb_contrast"),
                                 (rs.option.sharpness,   "rgb_sharpness")]:
                    try:
                        rgb_sensor.set_option(opt, cfg_snap[key])
                    except Exception:
                        pass
            except Exception as e:
                log.debug("rgb sensor options: %s", e)

        except Exception as e:
            detail = str(e)
            try:
                _devs = list(rs.context().query_devices())
                if _devs and camera_negotiate is not None:
                    detail = camera_negotiate.describe_failure(_devs[0], cfg_snap, e)
            except Exception:
                pass
            global _CAMERA_LAST_ERROR
            _CAMERA_LAST_ERROR = detail
            log.warning("camera startup failed: %s", detail)
            log.warning("retrying in 3 s")
            time.sleep(3)
            continue

        # --- Inner frame loop ---
        while not _rs_restart_evt.is_set():
            try:
                frames  = pipeline.wait_for_frames(timeout_ms=3000)
                aligned = align.process(frames)

                color_f = aligned.get_color_frame() if cfg_snap["rgb_en"] else None
                depth_f = aligned.get_depth_frame() if cfg_snap["depth_en"] else None
                ir1_f   = frames.get_infrared_frame(1) if cfg_snap["ir1_en"] else None
                ir2_f   = frames.get_infrared_frame(2) if cfg_snap["ir2_en"] else None

                # Post-processing on depth
                if depth_f is not None:
                    if _FILTERS is not None:
                        depth_f = _FILTERS.process(depth_f)
                    elif cfg_snap["post_proc"]:
                        depth_f = spatial.process(depth_f)
                        depth_f = temporal.process(depth_f)
                        depth_f = hole_fill.process(depth_f)

                # Frame timing, and which clock it is in. Recorded every frame
                # because a run whose frames turn out to be stamped on the host
                # clock has an extra jitter term in its timing budget, and that
                # has to be knowable afterwards rather than guessed.
                if _HAS_RSF:
                    try:
                        src = depth_f if depth_f is not None else color_f
                        global_frame_meta = rs_features.frame_metadata(src)
                    except Exception:
                        pass

                rgb_arr   = np.asanyarray(color_f.get_data())  if color_f else None
                # RAW first: the colourised copy is for the operator's eyes and
                # has no metric content left in it.
                depth_raw = np.asanyarray(depth_f.get_data()).copy() if depth_f else None
                depth_arr = np.asanyarray(
                    colorizer.colorize(depth_f).get_data()
                ) if depth_f else None
                ir1_arr   = np.asanyarray(ir1_f.get_data())    if ir1_f   else None
                ir2_arr   = np.asanyarray(ir2_f.get_data())    if ir2_f   else None

                if ir1_arr is not None and ir1_arr.ndim == 2:
                    ir1_arr = cv2.cvtColor(ir1_arr, cv2.COLOR_GRAY2BGR)
                if ir2_arr is not None and ir2_arr.ndim == 2:
                    ir2_arr = cv2.cvtColor(ir2_arr, cv2.COLOR_GRAY2BGR)

                with camera_lock:
                    global_rgb_frame   = rgb_arr
                    global_depth_frame = depth_arr
                    global_depth_raw   = depth_raw
                    global_ir1_frame   = ir1_arr
                    global_ir2_frame   = ir2_arr

            except RuntimeError as e:
                log.debug("rs wait_for_frames timeout: %s", e)
                time.sleep(0.05)
            except Exception as e:
                log.warning("camera frame error: %s", e)
                time.sleep(0.01)

        # --- Clean stop before restart ---
        try:
            pipeline.stop()
        except Exception:
            pass
        log.info("RealSense pipeline restarting with new config …")
        time.sleep(0.5)


# ============================================================
# Single point of UR motion execution.
# Whether the command came from local 8765 or via the relay, it
# eventually funnels here. We do the FINAL envelope/safety check
# here as a defense-in-depth net behind the relay's checks.
# ============================================================
def execute_motion(data):
    """Execute jog/movel/run_script. Returns (ok, reason)."""
    mtype = data.get("type")
    vcap  = float(data.get("_vcap", 1.0))  # relay may have throttled

    if mtype == "jog":
        q = data.get("q")
        if not q or len(q) != 6:
            return False, "bad_payload"
        script = (
            f"servoj([{f5(q[0])},{f5(q[1])},{f5(q[2])},"
            f"{f5(q[3])},{f5(q[4])},{f5(q[5])}], "
            f"a=1.4, v={f5(1.2*vcap)}, t=0.08, "
            f"lookahead_time=0.2, gain=200)\n"
        )
        ok = send_realtime_script(script)
        if ok:
            log.debug("jog sent: q=%s vcap=%.2f", [f5(x) for x in q], vcap)
        else:
            log.warning("jog DROPPED: 30003 not connected (UR in Local mode?)")
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "movel":
        p = data.get("pose")
        if not p or len(p) != 6:
            return False, "bad_payload"
        if not envelope_accepts(p):
            audit("motion_denied_local_envelope", {"pose": p})
            return False, "envelope_violation"
        script = (
            f"def web_movel():\n"
            f"  movel(p[{f5(p[0])},{f5(p[1])},{f5(p[2])},"
            f"{f5(p[3])},{f5(p[4])},{f5(p[5])}], "
            f"a={f5(0.1*vcap)}, v={f5(0.05*vcap)}, r=0.0)\n"
            f"end\n"
        )
        send_urscript_to_robot(script)
        return True, "ok"

    if mtype == "speedl":
        # Cartesian velocity via 30003 long-lived socket — zero stop overhead.
        # t=0.15 auto-stops if no new command within 150ms (dead-man safety).
        xd = data.get("xd")
        if not xd or len(xd) != 6:
            return False, "bad_payload"
        a = float(data.get("a", 0.8))
        t = float(data.get("t", 0.15))
        script = (f"speedl([{f5(xd[0])},{f5(xd[1])},{f5(xd[2])},"
                  f"{f5(xd[3])},{f5(xd[4])},{f5(xd[5])}],a={f5(a)},t={f5(t)})")
        ok = send_realtime_script(script)
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "speedl_stop":
        ok = send_realtime_script("stopl(3.0)")
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "speedj":
        # Joint-space velocity command via the same persistent 30002 command
        # channel used by the TCP joystick. This avoids high-rate run_script
        # socket churn and gives smooth dead-man control.
        qd = data.get("qd")
        if not qd or len(qd) != 6:
            return False, "bad_payload"
        a = float(data.get("a", 1.2))
        t = float(data.get("t", 0.08))
        qd = [float(v) * vcap for v in qd]
        script = (f"speedj([{f5(qd[0])},{f5(qd[1])},{f5(qd[2])},"
                  f"{f5(qd[3])},{f5(qd[4])},{f5(qd[5])}],"
                  f"a={f5(a)},t={f5(t)})")
        ok = send_realtime_script(script)
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "speedj_stop":
        a = float(data.get("a", 1.2))
        ok = send_realtime_script(f"stopj({f5(a)})")
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "freedrive_start":
        # Freedrive must be sent as valid multi-line URScript. The previous
        # compact string could be parsed unreliably by URScript. Keep this
        # isolated from the realtime speedl/speedj socket.
        axes = data.get("freeAxes", [1,1,1,1,1,1])
        axes = [1 if int(a) else 0 for a in axes[:6]]
        if len(axes) != 6:
            axes = [1,1,1,1,1,1]
        axes_str = ",".join(str(a) for a in axes)
        script = (
            "def sonair_fd():\n"
            f"  freedrive_mode(freeAxes=[{axes_str}])\n"
            "  sleep(30.0)\n"
            "  end_freedrive_mode()\n"
            "end\n"
        )
        ok = send_urscript_to_robot(script, stop_first=False)
        audit("freedrive_start", {"freeAxes": axes})
        log.info("freedrive START axes=%s", axes)
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "freedrive_stop":
        script = (
            "def sonair_fd_stop():\n"
            "  end_freedrive_mode()\n"
            "  stopj(2.0)\n"
            "end\n"
        )
        ok = send_urscript_to_robot(script, stop_first=False)
        audit("freedrive_stop", {})
        log.info("freedrive STOP")
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "run_script":
        script = data.get("script", "")
        send_urscript_to_robot(script)
        audit("run_script", {"size": len(script),
                             "origin": data.get("_origin")})
        return True, "ok"

    return False, "unknown_type"


# ============================================================
# Local face — UoN browser on same LAN connects here directly
# ============================================================
# ============================================================
# Vision inspection: locate -> plan -> detect.
# Kept as one in-process session so the locate step's working arrays (the part
# mask and the height field) survive into detect() without being serialised
# and sent to a browser that has no use for a 640x480 float array.
# ============================================================
_INSPECT = {"located": None, "plan": None, "detect": None}


def _handeye():
    """
    The camera-to-base transform, if both halves are known.

    The TCP pose comes through _tcp_now(), which prefers the verified RTDE
    telemetry and only falls back to the legacy 30003 globals. Reading the
    globals directly — as this did — meant that on a cell running the new
    telemetry path the pose was all zeros, so this returned None and every
    3D feature reported "no hand-eye calibration" moments after one had been
    solved and applied.
    """
    if not _HAS_EXT:
        return None
    T_tcp_cam = ur_bridge_ext.SCAN3D.T_tcp_cam
    if T_tcp_cam is None:
        return None
    pose = _tcp_now()
    if not pose or len(pose) < 6 or not any(pose):
        return None
    try:
        import numpy as _np
        from scan3d import pose_to_matrix
        return pose_to_matrix(pose) @ _np.asarray(T_tcp_cam, dtype=float)
    except Exception:
        return None


def _strip(d):
    """Drop the in-process working arrays before anything is serialised."""
    return {k: v for k, v in d.items() if not k.startswith("_")}


def _handle_inspect(data: dict):
    mtype = data.get("type")
    with camera_lock:
        depth_raw = global_depth_raw
        color = global_rgb_frame
        intr = global_depth_intr

    if mtype == "inspect_locate":
        if intr is None:
            return {"type": "inspect_locate_res", "ok": False,
                    "error": "camera intrinsics unavailable — start the depth "
                             "stream first. They must come from the camera."}
        T = _handeye()
        res = vision_inspect.locate(
            depth_raw, intr,
            getattr(intr, "depth_scale", 0.001),
            T_base_cam=T,
            min_height_mm=float(data.get("min_height_mm", 5.0)),
            max_range_m=float(data.get("max_range_m", 1.2)))
        _INSPECT["located"] = res if res.get("ok") else None
        out = {"type": "inspect_locate_res", **_strip(res)}
        if res.get("ok") and T is None:
            out["warning"] = ("no hand-eye calibration and TCP pose, so the "
                              "outline has image coordinates only. Set the "
                              "hand-eye transform to plan a robot path.")
        return out

    if mtype == "inspect_plan":
        loc = _INSPECT["located"]
        if not loc:
            return {"type": "inspect_plan_res", "ok": False,
                    "error": "locate the component first"}
        res = vision_inspect.plan(
            loc,
            spacing_mm=float(data.get("spacing_mm", 5.0)),
            step_mm=float(data.get("step_mm", 5.0)),
            standoff_mm=float(data.get("standoff_mm", 100.0)),
            margin_mm=float(data.get("margin_mm", 5.0)),
            mode=data.get("mode", "raster"))
        _INSPECT["plan"] = res if res.get("ok") else None
        return {"type": "inspect_plan_res", **res}

    if mtype == "inspect_detect":
        loc = _INSPECT["located"]
        if not loc:
            return {"type": "inspect_detect_res", "ok": False,
                    "error": "locate the component first"}
        res = vision_inspect.detect(
            color, loc, intr,
            getattr(intr, "depth_scale", 0.001),
            depth_thresh_mm=float(data.get("depth_thresh_mm", 1.5)),
            visual_thresh=int(data.get("visual_thresh", 22)),
            min_area_mm2=float(data.get("min_area_mm2", 0.5)),
            T_base_cam=_handeye())
        _INSPECT["detect"] = res if res.get("ok") else None
        return {"type": "inspect_detect_res", **res}

    if mtype == "inspect_status":
        loc = _INSPECT["located"]
        return {"type": "inspect_status_res",
                "available": _HAS_VISINSP,
                "has_depth": depth_raw is not None,
                "has_color": color is not None,
                "has_intrinsics": intr is not None,
                "has_handeye": _handeye() is not None,
                "located": bool(loc),
                "planned": bool(_INSPECT["plan"]),
                "n_candidates": (_INSPECT["detect"] or {}).get("n_total", 0)}
    return None


# ============================================================
# Hand-eye calibration, multi-view reconstruction, and the rest of the camera.
#
# All three keep their working state IN THIS PROCESS. The arrays involved — a
# board's corner list, a voxel grid, a fused cloud — are large, are useless to
# a browser, and are needed by the NEXT step, so serialising them out and back
# would be pure cost. What crosses the socket is numbers and decisions.
# ============================================================
_HE = {"session": None, "last_deep": 0.0}
_MV = {"session": None, "region": None, "plan": None}


def _tcp_now():
    if _HAS_EXT and ur_bridge_ext.UR.enabled:
        st = ur_bridge_ext.UR.state() or {}
        pose = st.get("actual_TCP_pose")
        if pose and any(pose):
            return list(pose)
    with data_lock:
        pose = list(global_tcp_pose)
    return pose if pose and any(pose) else None


def _apply_handeye(T_tcp_cam) -> dict:
    """One place where the calibration takes effect, so it cannot half-apply."""
    if not _HAS_EXT:
        return {"ok": False, "error": "ur_bridge_ext unavailable"}
    try:
        import numpy as _np
        ur_bridge_ext.SCAN3D.T_tcp_cam = _np.asarray(T_tcp_cam, dtype=float).reshape(4, 4)
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": str(e)}
    return {"ok": True}


def _handle_handeye(data: dict):
    mtype = data.get("type")
    sess = _HE["session"]

    if mtype == "handeye_status":
        loaded = handeye.load() if _HAS_HANDEYE else {"ok": False}
        out = {"type": "handeye_status_res",
               "available": _HAS_HANDEYE,
               "applied": bool(_HAS_EXT and ur_bridge_ext.SCAN3D.T_tcp_cam is not None),
               "saved": {k: loaded.get(k) for k in
                         ("calib_version", "translation_mm", "rotation_deg",
                          "target_spread_mm", "verdict", "solved_utc", "path")}
               if loaded.get("ok") else None}
        if _HAS_HANDEYE:
            out.update(sess.status() if sess else
                       {"n": 0, "error": handeye.available()[1]})
        return out

    if not _HAS_HANDEYE:
        return {"type": "handeye_res", "ok": False,
                "error": "hand-eye calibration needs OpenCV and numpy on the "
                         "host — run: pip install opencv-python numpy"}

    if mtype == "handeye_begin":
        spec = handeye.TargetSpec(**{k: v for k, v in (data.get("target") or {}).items()
                                     if k in handeye.TargetSpec.__dataclass_fields__})
        _HE["session"] = handeye.HandEyeSession(spec)
        return {"type": "handeye_res", "cmd": "begin", "ok": True,
                "target": spec.as_dict(),
                "note": ("Fix the board where the camera can see it and the arm "
                         "can move around it. Take a dozen poses, rotating the "
                         "tool 30-60 deg about ALL THREE axes between them, at "
                         "a range of distances.")}

    if mtype == "handeye_preview":
        # Detect without storing: the operator can see whether the board is
        # found before committing a pose, which is the difference between
        # twelve good samples and twelve samples.
        with camera_lock:
            color = global_rgb_frame
            intr = global_depth_intr
        spec = (sess.spec if sess else handeye.TargetSpec(
            **{k: v for k, v in (data.get("target") or {}).items()
               if k in handeye.TargetSpec.__dataclass_fields__}))
        # Deep search — every presentation of the image, plus the hunt for
        # what size the board actually is — costs several seconds on a dense
        # board, and this runs three times a second. So it runs on a timer of
        # its own: quick on every frame, deep once every few seconds while the
        # board is not being found, and immediately when the operator asks.
        # The operator still gets "it is a 24x17 board" without pressing
        # anything, and the socket is never blocked waiting for it.
        now = time.monotonic()
        deep = bool(data.get("deep")) or (now - _HE.get("last_deep", 0.0) > 5.0)
        res = handeye.detect_target(color, spec, intr,
                                    effort="full" if deep else "quick")
        if deep and not res.get("ok"):
            _HE["last_deep"] = now
        # The corner list is for drawing. It used to be truncated at 200
        # points to keep the message small, which on a 408-corner board drew
        # the FIRST 200 -- a solid patch over part of the board, with the rest
        # bare. That reads as "it only found half of it" when the whole board
        # was found perfectly, and it is the single most alarming thing the
        # page can show for no reason at all.
        #
        # Thinned instead of truncated, evenly across the grid, so the overlay
        # covers the board the way the detection does. The outline is sent
        # separately: the board's real perimeter, which a polyline through
        # every corner in scan order never was.
        if res.get("corners"):
            res["outline"] = handeye.board_outline(
                res["corners"], spec.cols, spec.rows)
            res["corners_total"] = len(res["corners"])
            res["corners"] = handeye.thin_corners(
                res["corners"], spec.cols, spec.rows, budget=260)
        # Send back the very frame the corners were measured on. Drawing them
        # over whatever frame the browser happens to hold puts the overlay a
        # pose or two behind while the arm is moving, and an overlay that is
        # silently offset from its image is worse than no overlay — it reads
        # as a calibration error rather than as latency.
        if _HAS_VISION and color is not None:
            try:
                ok_enc, buf = cv2.imencode(".jpg", color,
                                           [cv2.IMWRITE_JPEG_QUALITY, 58])
                if ok_enc:
                    res["frame"] = base64.b64encode(buf).decode("utf-8")
            except Exception:
                pass
        return {"type": "handeye_preview_res", **res}

    if sess is None:
        return {"type": "handeye_res", "cmd": mtype, "ok": False,
                "error": "start a calibration first"}

    if mtype == "handeye_identify":
        # Counting inner corners by eye on a fine board is the single
        # commonest way to get a calibration quietly wrong, and it is a thing
        # a machine does better. Point the camera at the board, press once.
        with camera_lock:
            color = global_rgb_frame
        if color is None:
            return {"type": "handeye_identify_res", "ok": False,
                    "error": "no camera picture — start the colour stream"}
        import numpy as _np
        img = _np.asarray(color)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        found = handeye.measure_board(gray)
        if not found:
            return {"type": "handeye_identify_res", "ok": False,
                    "error": "could not read a chessboard in this picture. "
                             "Get the whole board in frame, filling most of "
                             "it, with a pale margin all round."}
        cols, rows, corners = found
        pitch = handeye.corner_pitch_px(corners, cols, rows)
        out = {"type": "handeye_identify_res", "ok": True,
               "cols": cols, "rows": rows,
               "corners": int(cols * rows),
               "pitch_px": round(pitch, 1) if pitch else None,
               "frame_px": [int(gray.shape[1]), int(gray.shape[0])]}
        # How far away this board can still be read, from the optics rather
        # than from trial and error. It is the number that decides whether a
        # fine board suits the working distance at all, and it is not
        # something anyone can judge by looking.
        with camera_lock:
            intr = global_color_intr or global_depth_intr
        sq = float((data.get("target") or {}).get("square_mm") or 0.0)
        if intr is not None and sq > 0:
            try:
                fx = float(handeye._K_from(intr)[0][0, 0])
                out["max_distance_mm"] = round(fx * sq / 15.0)
                out["fx"] = round(fx, 1)
            except Exception:
                pass
        return out

    if mtype == "handeye_auto_plan":
        # Plan the next round of poses. Round one needs nothing known; round
        # two is planned from the rough answer round one produced.
        stage = data.get("stage", "bootstrap")
        pose = _tcp_now()
        if stage == "bootstrap":
            # How far the board is, right now. The operator can only start
            # this while the live view is showing it found, so this is always
            # available -- and it is what lets the first round orbit the board
            # instead of sweeping past it.
            pivot_m = None
            with camera_lock:
                color = global_rgb_frame
                intr = global_depth_intr
            det = handeye.detect_target(color, sess.spec, intr, effort="quick")
            if det.get("ok") and det.get("distance_mm"):
                pivot_m = float(det["distance_mm"]) / 1000.0
            res = handeye.plan_bootstrap_poses(
                pose, pivot_m=pivot_m,
                envelope=ENVELOPE if data.get("use_envelope", True) else None)
            res["pivot_mm"] = round(pivot_m * 1000) if pivot_m else None
        else:
            guess = None
            if sess.result and sess.result.get("ok"):
                guess = sess.result["T_tcp_cam"]
            elif _HAS_EXT and ur_bridge_ext.SCAN3D.T_tcp_cam is not None:
                guess = ur_bridge_ext.SCAN3D.T_tcp_cam

            # Where the board is, from the round that just finished. This
            # needs no photograph: the first round saw it from a dozen poses
            # and the answer is in the session already. Re-photographing it
            # at whatever pose that round ended on made the whole calibration
            # hinge on one frame, taken from the most awkward viewpoint in the
            # set -- and when that frame failed, the run aborted and left the
            # operator with poses the console itself refuses to use.
            T_cam_target = sess.target_in_camera(pose)
            source = "the first round"
            if T_cam_target is None:
                # No solve to build on. Fall back to looking, and say so.
                with camera_lock:
                    color = global_rgb_frame
                    intr = global_depth_intr
                det = handeye.detect_target(color, sess.spec, intr)
                if not det.get("ok") or "T_cam_target" not in det:
                    return {"type": "handeye_auto_plan_res", "ok": False,
                            "error": "there is no first-round answer to plan "
                                     "from, and the board is not visible from "
                                     "here either. " + det.get("error", "")}
                T_cam_target = det["T_cam_target"]
                source = "what the camera can see now"

            res = handeye.plan_auto_poses(
                pose, T_cam_target,
                n_poses=int(data.get("n_poses", 14)), stage="fine",
                T_tcp_cam_guess=guess, board_size_m=sess.spec.size_m(),
                envelope=ENVELOPE if data.get("use_envelope", True) else None)
            res["planned_from"] = source
        return {"type": "handeye_auto_plan_res", "stage": stage, **res}

    if mtype == "handeye_capture":
        with camera_lock:
            color = global_rgb_frame
            intr = global_depth_intr
        # A capture happens once, with the arm stopped and the operator
        # waiting. Nothing is saved by searching less hard here, and a pose
        # lost to a quick search costs a whole move.
        return {"type": "handeye_capture_res",
                **sess.add(color, _tcp_now(), intr)}
    if mtype == "handeye_undo":
        return {"type": "handeye_capture_res", **sess.remove_last()}
    if mtype == "handeye_clear":
        return {"type": "handeye_capture_res", **sess.clear()}
    if mtype == "handeye_solve":
        res = sess.solve(data.get("method", "all"))
        if res.get("ok") and data.get("apply", True):
            res["applied"] = _apply_handeye(res["T_tcp_cam"]).get("ok", False)
        return {"type": "handeye_solve_res", **res}
    if mtype == "handeye_save":
        res = handeye.save(sess.result, data.get("path") or handeye.DEFAULT_PATH)
        return {"type": "handeye_res", "cmd": "save", **res}
    if mtype == "handeye_load":
        res = handeye.load(data.get("path") or handeye.DEFAULT_PATH)
        if res.get("ok"):
            res["applied"] = _apply_handeye(res["T_tcp_cam"]).get("ok", False)
        return {"type": "handeye_res", "cmd": "load", **res}
    return None


def _handle_multiview(data: dict):
    mtype = data.get("type")
    with camera_lock:
        depth_raw = global_depth_raw
        intr = global_depth_intr

    if mtype == "mv_status":
        sess = _MV["session"]
        return {"type": "mv_status_res",
                "available": _HAS_MV,
                "error": "" if _HAS_MV else "multiview needs numpy on the host",
                "has_handeye": bool(_HAS_EXT and ur_bridge_ext.SCAN3D.T_tcp_cam is not None),
                "has_depth": depth_raw is not None,
                "has_intrinsics": intr is not None,
                "region": _MV["region"],
                "plan": {k: (_MV["plan"] or {}).get(k) for k in
                         ("n", "standoff_mm", "tilt_deg", "rings", "explain",
                          "azimuth_coverage_deg", "travel_mm")}
                if _MV["plan"] else None,
                **(sess.status() if sess else {"views": 0})}

    if not _HAS_MV:
        return {"type": "mv_res", "ok": False,
                "error": "multiview needs numpy on the host"}

    if mtype == "mv_region":
        T = _handeye()
        res = multiview.region_from_roi(
            depth_raw, intr, data.get("roi") or [0, 0, 100, 100],
            T_base_cam=T,
            depth_scale=getattr(intr, "depth_scale", 0.001) if intr else 0.001)
        _MV["region"] = res if res.get("ok") else None
        _MV["plan"] = None
        return {"type": "mv_region_res", **_strip(res)}

    if mtype == "mv_plan":
        reg = _MV["region"]
        if not reg:
            return {"type": "mv_plan_res", "ok": False,
                    "error": "draw a box around the part first"}
        T_tcp_cam = ur_bridge_ext.SCAN3D.T_tcp_cam if _HAS_EXT else None
        res = multiview.plan_views(
            reg, intr=intr, T_tcp_cam=T_tcp_cam,
            n_views=data.get("n_views"),
            standoff_mm=data.get("standoff_mm"),
            tilt_deg=data.get("tilt_deg"),
            rings=data.get("rings"),
            overlap=float(data.get("overlap", 0.55)),
            envelope=ENVELOPE if data.get("use_envelope", True) else None,
            start_pose=_tcp_now())
        _MV["plan"] = res if res.get("ok") else None
        return {"type": "mv_plan_res", **res}

    if mtype == "mv_begin":
        reg = _MV["region"]
        if not reg:
            return {"type": "mv_res", "cmd": "begin", "ok": False,
                    "error": "draw a box around the part first"}
        T_tcp_cam = ur_bridge_ext.SCAN3D.T_tcp_cam if _HAS_EXT else None
        if T_tcp_cam is None:
            return {"type": "mv_res", "cmd": "begin", "ok": False,
                    "error": "hand-eye calibration not set — do step 2 first"}
        if intr is None:
            return {"type": "mv_res", "cmd": "begin", "ok": False,
                    "error": "camera intrinsics unavailable — start the depth stream"}
        try:
            _MV["session"] = multiview.MultiViewSession(
                intr, T_tcp_cam, reg,
                voxel_mm=float(data.get("voxel_mm", 1.5)),
                margin_mm=float(data.get("margin_mm", 25.0)))
        except Exception as e:      # noqa: BLE001
            return {"type": "mv_res", "cmd": "begin", "ok": False, "error": str(e)}
        return {"type": "mv_res", "cmd": "begin", "ok": True,
                **_MV["session"].status()}

    sess = _MV["session"]
    if sess is None:
        return {"type": "mv_res", "cmd": mtype, "ok": False,
                "error": "start a scan first"}

    if mtype == "mv_capture":
        return {"type": "mv_capture_res",
                **sess.add_view(depth_raw, _tcp_now(),
                                stride=int(data.get("stride", 2)),
                                label=data.get("label", ""))}
    if mtype == "mv_build":
        return {"type": "mv_build_res",
                **sess.build(min_hits=int(data.get("min_hits", 2)),
                             plane_tol_mm=float(data.get("plane_tol_mm", 4.0)),
                             link_mm=float(data.get("link_mm", 8.0)),
                             keep_plane=bool(data.get("keep_plane", False)))}
    if mtype == "mv_preview":
        return {"type": "mv_preview_res",
                **sess.preview(max_points=int(data.get("max_points", 12000)),
                               source=data.get("source", "surface"))}
    if mtype == "mv_plan_path":
        res = sess.plan_path(
            standoff=float(data.get("standoff_mm", 100.0)) / 1000.0,
            line_spacing=float(data.get("spacing_mm", 5.0)) / 1000.0,
            step_along=float(data.get("step_mm", 5.0)) / 1000.0,
            margin=float(data.get("margin_mm", 5.0)) / 1000.0)
        return {"type": "mv_plan_path_res", **res}
    if mtype == "mv_export":
        name = data.get("name") or f"scan_{int(time.time())}"
        return {"type": "mv_res", "cmd": "export",
                **sess.export_ply(Path("scans") / f"{name}.ply")}
    return None


def _handle_rs(data: dict):
    mtype = data.get("type")
    if mtype == "rs_enumerate":
        if not _HAS_RSF:
            return {"type": "rs_enumerate_res", "available": False,
                    "error": "rs_features unavailable"}
        return {"type": "rs_enumerate_res", **rs_features.enumerate_device()}
    if not _HAS_RSF:
        return {"type": "rs_res", "cmd": mtype, "ok": False,
                "error": "rs_features unavailable"}
    if mtype == "rs_options":
        return {"type": "rs_res", "cmd": "options",
                **rs_features.set_options(data.get("settings") or [])}
    if mtype == "rs_emitter":
        return {"type": "rs_res", "cmd": "emitter",
                **rs_features.set_emitter(data.get("mode", "on"),
                                          data.get("laser_power"))}
    if mtype == "rs_filters":
        cfg = data.get("config") or {}
        with _rs_config_lock:
            merged = {**(_rs_config.get("filters") or {}), **cfg}
            _rs_config["filters"] = merged
        # Applied live where possible; a filter chain needs no pipeline
        # restart, so the stream does not blink for a slider change.
        if _FILTERS is not None:
            _FILTERS.configure(merged)
            desc = _FILTERS.describe()
        else:
            desc = rs_features.FilterChain(merged).describe()
        return {"type": "rs_filters_res", "ok": True, **desc}
    if mtype == "rs_selfcal":
        return {"type": "rs_selfcal_res",
                **rs_features.self_calibrate(
                    data.get("mode", "calibrate"),
                    float(data.get("target_distance_mm", 600.0)),
                    int(data.get("speed", 2)))}
    if mtype == "rs_advanced_get":
        return {"type": "rs_advanced_res", "cmd": "get", **rs_features.advanced_get()}
    if mtype == "rs_advanced_set":
        return {"type": "rs_advanced_res", "cmd": "set",
                **rs_features.advanced_set(data.get("json", ""))}
    if mtype == "rs_advanced_enable":
        return {"type": "rs_advanced_res", "cmd": "enable",
                **rs_features.advanced_enable(bool(data.get("on", True)))}
    if mtype == "rs_extrinsics":
        return {"type": "rs_extrinsics_res",
                **rs_features.stream_extrinsics(global_rs_profile)}
    if mtype == "rs_metadata":
        return {"type": "rs_metadata_res", **(global_frame_meta or
                                              {"available": False})}
    if mtype == "rs_ir_diagnose":
        with camera_lock:
            ir1, ir2, draw = global_ir1_frame, global_ir2_frame, global_depth_raw
        return {"type": "rs_ir_diagnose_res", **rs_features.diagnose_ir(ir1, ir2, draw)}
    if mtype == "rs_pointcloud":
        with camera_lock:
            draw, color, intr = global_depth_raw, global_rgb_frame, global_depth_intr
        name = data.get("name") or f"cloud_{int(time.time())}"
        return {"type": "rs_res", "cmd": "pointcloud",
                **rs_features.export_pointcloud(
                    draw, intr, Path("scans") / f"{name}.ply", color,
                    stride=int(data.get("stride", 2)))}
    return None


def _encode_preview(frames: dict, width: int, quality: int) -> dict:
    """
    Turn the live frames into a message small enough to send twenty times a
    second, without touching what the algorithms see.

    THE PREVIEW IS NOT THE MEASUREMENT. Detection, reconstruction and the
    hand-eye solve all read `global_rgb_frame` at full resolution; this makes
    a picture for a person, and a person cannot see the difference between
    1280 px and 720 px in a 600 px wide panel. Sending the full frame cost
    four times the bytes for no visible gain, and those bytes were what
    pushed the socket past what the browser could drain.

    Runs on a worker thread. The JPEG encode is the expensive part and it
    must not happen on the event loop.
    """
    out = {"type": "camera_frame"}
    for name, img in frames.items():
        try:
            h, w = img.shape[:2]
            if w > width:
                scale = width / float(w)
                img = cv2.resize(img, (int(w * scale), int(h * scale)),
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img,
                                   [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
            if ok:
                out[name] = base64.b64encode(buf).decode("utf-8")
                out[name + "_px"] = [int(img.shape[1]), int(img.shape[0])]
        except Exception as e:      # noqa: BLE001
            # One unencodable frame must not cost the other three.
            log.debug("preview encode failed for %s: %s", name, e)
    return out


# ============================================================
# Automation — the cell runs the job
# ============================================================

class _CellContext:
    """
    Everything `automation` is allowed to touch, as plain callables.

    The runner takes this rather than importing the bridge, which is what
    lets the stop path and the failure path be exercised with no robot, no
    camera and no sensors attached. A runner that has only ever been tried
    against real hardware has never had its abort tested.
    """

    def robot_enabled(self) -> bool:
        return bool(_HAS_EXT and ur_bridge_ext.UR.enabled)

    def robot_state(self) -> dict:
        return (ur_bridge_ext.UR.state() if _HAS_EXT else {}) or {}

    def robot_health(self) -> dict:
        st = (ur_bridge_ext.UR.status() if _HAS_EXT else {}) or {}
        return st.get("health", {}) or {}

    def robot_host(self) -> str:
        return robot_host()

    def tcp_pose(self):
        with data_lock:
            return list(global_tcp_pose) if global_tcp_pose else None

    def camera_age_s(self):
        if not _HAS_VISION:
            return None
        with camera_lock:
            return 0.0 if global_rgb_frame is not None else None

    def camera_info(self) -> dict:
        with _rs_config_lock:
            cfg = dict(_rs_config)
        with camera_lock:
            intr = global_color_intr or global_depth_intr
        out = {"colour": cfg.get("rgb_res"), "depth": cfg.get("stereo_res"),
               "emitter": cfg.get("emitter")}
        if intr is not None:
            for k in ("fx", "fy", "cx", "cy", "depth_scale"):
                v = getattr(intr, k, None)
                if v is not None:
                    out[k] = round(float(v), 6)
        return out

    def calibration(self):
        if not _HAS_HANDEYE:
            return None
        sess = _HE.get("session")
        if sess is not None and sess.result and sess.result.get("ok"):
            return sess.result
        loaded = handeye.load()
        return loaded if loaded.get("ok") else None

    def imu_status(self) -> dict:
        return bench_agent.HUB.status() if _HAS_BENCH else {}

    def clock_status(self) -> dict:
        return bench_agent.MASTER.status() if _HAS_BENCH else {}

    def channel_registry(self):
        if not _HAS_BENCH:
            return []
        try:
            return sensor_hub.HUB.report().get("channels", [])
        except Exception:
            return []

    def out_dir(self) -> str:
        return str(Path("bench_runs").resolve().parent)

    # -- actions ---------------------------------------------------------
    def move_to(self, pose, speed):
        if not _HAS_EXT or ur_bridge_ext.UR.controller is None:
            return False, "the robot link has not been started"
        return ur_bridge_ext.UR.controller.movel(list(pose), a=0.5, v=float(speed))

    def halt(self):
        if _HAS_EXT and ur_bridge_ext.UR.jog is not None:
            ur_bridge_ext.UR.jog.halt()

    def is_recording(self) -> bool:
        return bool(_HAS_BENCH and bench_agent.RECORDER.is_recording())

    def record_start(self, args) -> dict:
        return bench_agent.RECORDER.start(**args)

    def record_stop(self) -> dict:
        return bench_agent.RECORDER.stop()

    def imu_logging(self) -> bool:
        return bool(_HAS_BENCH and bench_agent.LOGGER.running())

    def imu_log_start(self, path=None) -> dict:
        return bench_agent.LOGGER.start(path)

    def imu_log_stop(self) -> dict:
        return bench_agent.LOGGER.stop()

    def job_started_at(self) -> float:
        return RUNNER.started_at if RUNNER else 0.0

    def export_dataset(self, name) -> dict:
        return automation.export_dataset(
            name, out_root=Path("datasets"), runs_dir=Path("bench_runs"),
            imu_dir=Path("imu_logs"), ctx=self)


CELL = _CellContext()
RUNNER = automation.Runner(CELL) if _HAS_AUTO else None


def _handle_automation(data: dict):
    mtype = data.get("type")
    if not _HAS_AUTO:
        return {"type": "auto_res", "ok": False,
                "error": f"automation unavailable: {_AUTO_ERR}"}

    if mtype == "auto_status":
        return {"type": "auto_status_res", "ok": True, **RUNNER.status()}

    if mtype == "auto_preflight":
        return {"type": "auto_preflight_res", **automation.preflight(CELL)}

    if mtype == "auto_jobs":
        jobs = automation.builtin_jobs(CELL.tcp_pose())
        return {"type": "auto_jobs_res", "ok": True,
                "jobs": {k: v.as_dict() for k, v in jobs.items()}}

    if mtype == "auto_start":
        spec = data.get("job")
        if isinstance(spec, str):
            jobs = automation.builtin_jobs(CELL.tcp_pose())
            job = jobs.get(spec)
            if job is None:
                return {"type": "auto_res", "ok": False,
                        "error": f"no job called {spec!r}"}
        elif isinstance(spec, dict):
            job = automation.Job(
                name=spec.get("name", "job"),
                steps=spec.get("steps") or [],
                repeats=int(spec.get("repeats", 1)),
                sweep_key=spec.get("sweep_key", ""),
                sweep_values=spec.get("sweep_values") or [],
                notes=spec.get("notes", ""))
        else:
            return {"type": "auto_res", "ok": False, "error": "no job given"}
        return {"type": "auto_res", "cmd": "start", **RUNNER.start(job)}

    if mtype == "auto_stop":
        return {"type": "auto_res", "cmd": "stop", **RUNNER.stop()}

    if mtype == "auto_export":
        return {"type": "auto_res", "cmd": "export",
                **CELL.export_dataset(data.get("name") or "dataset")}

    return None


async def local_handler(websocket):
    log.info("local browser connected")

    # What this browser wants to be sent, and how big.
    #
    # It used to be everything, always: colour, depth and BOTH infrared
    # frames, JPEG-encoded and base64-ed every 50 ms whether or not the page
    # was showing a picture at all. Measured at the resolutions the console
    # now asks for, that is 269 kB a tick -- 5.2 MB/s -- pushed at a browser
    # sitting on the Robot page looking at joint angles. The console tells the
    # agent what it is actually displaying and gets that and nothing else.
    prefs = {"streams": set(), "width": 720, "quality": 55}

    async def stream():
        errors = 0
        slow = 0
        period = 0.05
        last_health = 0.0
        while True:
            try:
                if _HAS_VISION and prefs["streams"]:
                    with camera_lock:
                        avail = {"rgb": global_rgb_frame,
                                 "depth": global_depth_frame,
                                 "ir1": global_ir1_frame,
                                 "ir2": global_ir2_frame}
                    want = {k: v for k, v in avail.items()
                            if v is not None and k in prefs["streams"]}
                    if want:
                        # Encoding happens OFF the event loop. Four JPEGs a
                        # tick is tens of milliseconds of CPU, and doing it
                        # here meant the loop that also answers the keepalive
                        # ping spent most of every 50 ms inside libjpeg.
                        frame_msg = await asyncio.to_thread(
                            _encode_preview, want, prefs["width"], prefs["quality"])
                        t_send = time.monotonic()
                        await websocket.send(json.dumps(frame_msg))
                        # Adaptive: if the client cannot drain what we send,
                        # `send` blocks on the transport. Slow down rather
                        # than queue -- a backlog delays the keepalive too,
                        # and a missed keepalive closes the connection, which
                        # is indistinguishable at the browser from a crash.
                        took = time.monotonic() - t_send
                        if took > 0.10:
                            period = min(0.5, period * 1.5)
                            slow += 1
                            if slow in (5, 25, 100):
                                log.warning("browser is not keeping up "
                                            "(%.0f ms to send) — preview now "
                                            "%.0f fps", took * 1000, 1 / period)
                        elif period > 0.05 and took < 0.02:
                            period = max(0.05, period / 1.2)
                with data_lock:
                    q   = global_actual_q
                    tcp = global_tcp_pose
                if q:
                    await websocket.send(json.dumps({"type": "state", "q": q}))
                if tcp:
                    await websocket.send(json.dumps({"type": "tcp_pose", "q": tcp}))
                if _HAS_BENCH:
                    # One message carrying every inertial unit at once, in both
                    # the flat single-unit shape the console reads and the map
                    # the recorder needs. The browser renders it; acquisition
                    # never depends on it, so a slow or absent browser cannot
                    # back-pressure a run.
                    units = bench_agent.HUB.latest()
                    if units:
                        msg = (ur_bridge_ext.imu_message(units) if _HAS_EXT
                               else {"type": "imu", "units": units})
                        msg["rec"] = bench_agent.RECORDER.status()["recording"]
                        await websocket.send(json.dumps(msg))
                # A HEARTBEAT THAT DOES NOT DEPEND ON PICTURES.
                #
                # The lamps used to infer "camera alive" from the arrival of
                # camera frames. Now that a page only gets the streams it
                # displays, that inference says "no picture" on every page
                # that is not showing one -- which is true and useless, and
                # reads as a fault. Liveness is reported on its own, once a
                # second, and costs a couple of hundred bytes.
                now_h = time.monotonic()
                if now_h - last_health >= 1.0:
                    last_health = now_h
                    with camera_lock:
                        cam_ok = global_rgb_frame is not None
                        cam_shape = (list(global_rgb_frame.shape[1::-1])
                                     if cam_ok else None)
                    await websocket.send(json.dumps({
                        "type": "cell_health",
                        "camera": {"available": _HAS_VISION, "live": cam_ok,
                                   "size": cam_shape,
                                   "error": "" if _HAS_VISION else _VISION_ERR},
                        "robot": {"host": robot_host(),
                                  "enabled": bool(_HAS_EXT and ur_bridge_ext.UR.enabled)},
                        "streams": sorted(prefs["streams"]),
                        "preview_fps": round(1.0 / period, 1),
                    }))

                if _HAS_EXT and ur_bridge_ext.UR.enabled:
                    # The full field set, at a tenth of the RTDE rate. The UI
                    # cannot use 125 Hz and sending it would spend the whole
                    # socket budget on numbers nobody reads.
                    st = ur_bridge_ext.UR.state()
                    if st:
                        await websocket.send(json.dumps({"type": "ur_state", "s": st}))
                await asyncio.sleep(period)
            except asyncio.CancelledError:
                break
            except websockets.exceptions.ConnectionClosed:
                break
            except Exception as exc:                         # noqa: BLE001
                # This loop carries the camera, the robot state and every
                # inertial reading. It used to `break` on the first exception,
                # silently and for good: one bad frame and the console went
                # blank -- no picture, no joint angles, no sensors -- while
                # the socket stayed open, so it did not even look
                # disconnected. Nothing was logged either.
                #
                # A transient fault is now survived. A persistent one gives
                # up loudly rather than spinning.
                errors += 1
                record_fault("live stream", exc, f"error {errors}")
                if errors >= 20:
                    log.error("live stream failing repeatedly — stopping it")
                    try:
                        await websocket.send(json.dumps({
                            "type": "agent_fault", "on": "live stream",
                            "error": "the live stream stopped after 20 errors",
                            "fatal": True}))
                    except Exception:
                        pass
                    break
                await asyncio.sleep(0.5)

    stream_task = asyncio.create_task(stream())
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except Exception:
                continue
            mtype = data.get("type")
            # ONE MESSAGE MUST NEVER TAKE THE CONNECTION WITH IT.
            #
            # Every handler below used to run bare inside this loop, so any
            # exception any of them raised -- a robot socket that closed
            # mid-command, a reply holding a value json could not encode, a
            # field a newer console sent that an older agent did not expect --
            # escaped the `async for`, past the ConnectionClosed handler, and
            # out of local_handler, which closes the websocket. From the
            # operator's side that is "I pressed a button and it
            # disconnected", with nothing on screen naming the button.
            #
            # The guard costs one try block and turns every one of those into
            # a message on the page instead.
            try:
                await _dispatch(websocket, data, mtype, prefs)
            except websockets.exceptions.ConnectionClosed:
                raise
            except Exception as exc:                         # noqa: BLE001
                entry = record_fault("message handler", exc, mtype)
                try:
                    await websocket.send(json.dumps({
                        "type": "agent_fault",
                        "on": mtype,
                        "error": entry["error"],
                        "at": entry["at"],
                    }))
                except Exception:
                    pass
            continue

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as exc:                                 # noqa: BLE001
        record_fault("local_handler", exc, None)
    finally:
        stream_task.cancel()
        log.info("local browser disconnected")


async def _dispatch(websocket, data, mtype, prefs):
    """
    Handle one message from the console.

    Split out of the receive loop for one reason: so the loop can wrap it. As
    long as the dispatch was inline, there was no place to put a guard that
    did not also swallow the loop's own control flow.
    """

    # What this page is showing, so the agent sends that and nothing else.
    if mtype == "stream_prefs":
        want = data.get("streams")
        prefs["streams"] = set(want) & {"rgb", "depth", "ir1", "ir2"} if want else set()
        prefs["width"] = max(240, min(1280, int(data.get("width", 720))))
        prefs["quality"] = max(25, min(90, int(data.get("quality", 55))))
        await websocket.send(json.dumps({
            "type": "stream_prefs_res", "ok": True,
            "streams": sorted(prefs["streams"]),
            "width": prefs["width"], "quality": prefs["quality"]}))
        return

    # Why the last thing went wrong, in the operator's own words.
    if mtype == "agent_faults":
        await websocket.send(json.dumps({
            "type": "agent_faults_res", "ok": True, "faults": faults(),
            "robot_host": robot_host()}))
        return

    # auth from frontend — local face accepts any (LAN-trusted)
    if mtype == "auth":
        await websocket.send(json.dumps({
            "type":            "auth_ok",
            "session":         "local",
            "role":            "host",
            "site":            RELAY_SITE,
            "envelope":        ENVELOPE,
            "authority_state": "host_operator",
            "agent_online":    True,
        }))
        return
    if mtype == "ping":
        await websocket.send(json.dumps({"type": "pong",
                                         "ts": data.get("ts")}))
        return
    if mtype == "estop":
        estop_ur()
        return
    # Jog messages run INLINE. They are a lock and six floats, and the
    # whole point of moving the cadence to the host was to stop robot
    # motion waiting on anything that can stall. A thread-pool hop per
    # jog message reintroduces exactly that.
    if _HAS_EXT and str(mtype or "").startswith("jog_"):
        reply = ur_bridge_ext.handle_message(data)
        if reply is not None and not reply.pop("quiet", False):
            await websocket.send(json.dumps(reply))
        return

    # Starting the robot link is where the address the operator typed
    # becomes THE address, for every channel, before anything tries to
    # use it. Doing it here rather than inside ur_bridge_ext keeps the
    # one setter in the one module that owns the other four channels.
    if mtype == "ur_service_start":
        moved = await asyncio.to_thread(set_robot_host, data.get("host"))
        if moved.get("error"):
            await websocket.send(json.dumps({
                "type": "ur_service_start_res", "ok": False,
                "error": moved["error"]}))
            return

    if _HAS_EXT:
        reply = await asyncio.to_thread(ur_bridge_ext.handle_message, data)
        if reply is not None:
            if mtype == "ur_service_start":
                reply["address_applied_to"] = [
                    "telemetry (RTDE 30004)", "realtime (30003)",
                    "URScript (30002)", "dashboard (29999)",
                    "program list (FTP)"]
            await websocket.send(json.dumps(reply))
            return
    if _HAS_BENCH and (str(mtype or "").startswith("bench_")
                       or str(mtype or "").startswith("imu_")
                       or mtype == "sensors_report"):
        reply = await asyncio.to_thread(bench_agent.handle_message, data)
        if reply is not None:
            await websocket.send(json.dumps(reply))
        return
    if mtype in ("jog", "movel", "run_script",
                  "speedl", "speedl_stop", "speedj", "speedj_stop",
                  "freedrive_start", "freedrive_stop"):
        ok, reason = await asyncio.to_thread(execute_motion, data)
        if not ok:
            await websocket.send(json.dumps({
                "type":   "cmd_rejected",
                "seq":    data.get("seq"),
                "reason": reason,
            }))
        return
    if mtype == "get_urp_list":
        lst = await asyncio.to_thread(fetch_urp_list)
        await websocket.send(json.dumps({"type": "urp_list",
                                         "list": lst}))
        return
    if mtype == "dashboard":
        res = await asyncio.to_thread(send_dashboard_cmd,
                                      data.get("cmd", ""))
        await websocket.send(json.dumps({"type": "dashboard_res",
                                         "res": res}))
        return
    if mtype == "camera_config":
        # Frontend sends the full desired config dict.
        # We merge it into _rs_config and signal the camera thread
        # to restart the pipeline with new settings.
        new_cfg = data.get("config", {})
        with _rs_config_lock:
            _rs_config.update(new_cfg)
        _rs_restart_evt.set()
        audit("camera_config_applied", new_cfg)
        await websocket.send(json.dumps({
            "type":            "camera_config_ack",
            "config":          {**_rs_config},
            "vision_available": _HAS_VISION,
            "vision_error":    "" if _HAS_VISION else _VISION_ERR,
        }))
        if _HAS_VISION:
            log.info("camera_config applied: %s", new_cfg)
        else:
            log.warning("camera_config stored but NO CAMERA SUPPORT on this "
                        "agent (%s) — the config will take effect only once "
                        "the vision dependencies are installed", _VISION_ERR)
        return
    if str(mtype or "").startswith("handeye_"):
        reply = await asyncio.to_thread(_handle_handeye, data)
        if reply is not None:
            await websocket.send(json.dumps(reply))
        return
    if str(mtype or "").startswith("mv_"):
        reply = await asyncio.to_thread(_handle_multiview, data)
        if reply is not None:
            await websocket.send(json.dumps(reply))
        return
    if str(mtype or "").startswith("auto_"):
        reply = await asyncio.to_thread(_handle_automation, data)
        if reply is not None:
            await websocket.send(json.dumps(reply))
        return

    if str(mtype or "").startswith("rs_"):
        reply = await asyncio.to_thread(_handle_rs, data)
        if reply is not None:
            await websocket.send(json.dumps(reply))
        return
    if _HAS_VISINSP and str(mtype or "").startswith("inspect_"):
        reply = await asyncio.to_thread(_handle_inspect, data)
        if reply is not None:
            await websocket.send(json.dumps(reply))
        return

    if _HAS_CAMSVC and mtype in ("camera_probe", "camera_stats",
                                 "camera_point", "camera_option"):
        with camera_lock:
            depth_raw = global_depth_raw
            intr = global_depth_intr
        scale = (getattr(intr, "depth_scale", None)
                 or _rs_config.get("depth_units", 0.001))

        if mtype == "camera_probe":
            res = await asyncio.to_thread(camera_service.probe)
            res["intrinsics"] = camera_service.live_intrinsics(intr, scale)
            res["streaming"] = depth_raw is not None
            res["vision_available"] = _HAS_VISION
            res["vision_error"] = "" if _HAS_VISION else _VISION_ERR
            await websocket.send(json.dumps({"type": "camera_probe_res", **res}))
        elif mtype == "camera_stats":
            res = camera_service.stats(depth_raw, scale,
                                       float(data.get("roi_frac", 0.25)))
            await websocket.send(json.dumps({"type": "camera_stats_res", **res}))
        elif mtype == "camera_point":
            res = camera_service.point(depth_raw, data.get("x", 0),
                                       data.get("y", 0), intr, scale,
                                       int(data.get("window", 5)))
            await websocket.send(json.dumps({"type": "camera_point_res", **res}))
        else:
            res = await asyncio.to_thread(
                camera_service.set_option,
                data.get("sensor", "depth"),
                data.get("option", ""),
                data.get("value", 0))
            await websocket.send(json.dumps({"type": "camera_option_res", **res}))
        return

    if mtype == "get_camera_config":
        # Frontend requesting current live config (e.g. on reconnect)
        with _rs_config_lock:
            snap = dict(_rs_config)
        await websocket.send(json.dumps({
            "type":            "camera_config_ack",
            "config":          snap,
            "vision_available": _HAS_VISION,
            "vision_error":    "" if _HAS_VISION else _VISION_ERR,
        }))
        return

    # Nothing claimed it. Say so rather than dropping it: a message the
    # agent does not know is usually a console newer than the agent,
    # and silence makes that look like a dead button.
    log.debug("unhandled message type %r", mtype)
    await websocket.send(json.dumps({
        "type": "agent_unhandled", "on": mtype}))


# ============================================================
# Relay face — agent dials out to cloud relay, stays connected
# ============================================================
async def relay_uplink():
    """Maintain an outbound connection to the relay. Auto-reconnect on drop."""
    if not RELAY_URL:
        log.info("RELAY_URL not set — running in LOCAL-ONLY mode "
                 "(no cross-site teleop)")
        # Block forever so the local server keeps running
        await asyncio.Future()

    backoff = 1.0
    while True:
        try:
            log.info("dialing relay: %s", RELAY_URL)
            async with websockets.connect(RELAY_URL,
                                          ping_interval=20,
                                          ping_timeout=20) as ws:
                # Authenticate as the agent for our designated room
                await ws.send(json.dumps({
                    "type":  "auth",
                    "role":  "agent",
                    "token": RELAY_AGENT_TOKEN,
                    "site":  RELAY_SITE,
                    "room":  RELAY_ROOM,
                    "ts":    int(time.time() * 1000),
                }))
                first = await asyncio.wait_for(ws.recv(), timeout=5.0)
                resp  = json.loads(first)
                if resp.get("type") != "auth_ok":
                    log.error("relay auth failed: %s", resp)
                    await asyncio.sleep(10)
                    continue
                log.info("relay AUTH OK as agent for room=%s", RELAY_ROOM)
                audit("relay_connected", {"room": RELAY_ROOM})
                backoff = 1.0  # reset on success

                stop_evt = asyncio.Event()

                async def upstream():
                    """Push UR state and camera to the relay continuously."""
                    while not stop_evt.is_set():
                        try:
                            if _HAS_VISION:
                                with camera_lock:
                                    rgb   = global_rgb_frame
                                    depth = global_depth_frame
                                    ir1   = global_ir1_frame
                                    ir2   = global_ir2_frame
                                frame_msg = {"type": "camera_frame"}
                                if rgb is not None:
                                    _, buf = cv2.imencode(".jpg", rgb,
                                                          [cv2.IMWRITE_JPEG_QUALITY, 50])
                                    frame_msg["rgb"] = base64.b64encode(buf).decode("utf-8")
                                if depth is not None:
                                    _, buf = cv2.imencode(".jpg", depth,
                                                          [cv2.IMWRITE_JPEG_QUALITY, 40])
                                    frame_msg["depth"] = base64.b64encode(buf).decode("utf-8")
                                if ir1 is not None:
                                    _, buf = cv2.imencode(".jpg", ir1,
                                                          [cv2.IMWRITE_JPEG_QUALITY, 40])
                                    frame_msg["ir1"] = base64.b64encode(buf).decode("utf-8")
                                if ir2 is not None:
                                    _, buf = cv2.imencode(".jpg", ir2,
                                                          [cv2.IMWRITE_JPEG_QUALITY, 40])
                                    frame_msg["ir2"] = base64.b64encode(buf).decode("utf-8")
                                if len(frame_msg) > 1:
                                    try:
                                        await asyncio.wait_for(ws.send(json.dumps(frame_msg)), timeout=0.5)
                                    except asyncio.TimeoutError:
                                        log.debug("upstream frame timeout — skip")
                            with data_lock:
                                q   = global_actual_q
                                tcp = global_tcp_pose
                            if q:
                                await asyncio.wait_for(ws.send(json.dumps({"type":"state","q":q})), timeout=0.3)
                            if tcp:
                                await asyncio.wait_for(ws.send(json.dumps({"type":"tcp_pose","q":tcp})), timeout=0.3)
                            await asyncio.sleep(0.10)  # 10 Hz uplink
                        except websockets.exceptions.ConnectionClosed:
                            break
                        except Exception:
                            break

                async def downstream():
                    """Receive commands from relay and execute on UR."""
                    try:
                        async for raw in ws:
                            try:
                                data = json.loads(raw)
                            except Exception:
                                continue
                            mtype = data.get("type")

                            if mtype == "ping":
                                await ws.send(json.dumps({"type": "pong",
                                                          "ts": data.get("ts")}))
                                continue
                            if mtype == "estop":
                                estop_ur()
                                continue
                            if mtype in ("jog", "movel", "run_script",
                                          "speedl", "speedl_stop", "speedj", "speedj_stop",
                                          "freedrive_start", "freedrive_stop"):
                                origin_role = data.get("_origin_role", "guest")
                                authority_granted = data.get("_authority_granted", False)
                                if origin_role == "guest" and not authority_granted:
                                    audit("relay_motion_rejected_no_authority",{"origin":data.get("_origin")})
                                    continue
                                ok, reason = await asyncio.to_thread(execute_motion, data)
                                if not ok:
                                    audit("relay_motion_rejected_local",{"reason":reason,"origin":data.get("_origin")})
                                continue
                            if mtype == "get_urp_list":
                                lst = await asyncio.to_thread(fetch_urp_list)
                                await ws.send(json.dumps({"type": "urp_list",
                                                          "list": lst}))
                                continue
                            if mtype == "dashboard":
                                res = await asyncio.to_thread(send_dashboard_cmd,
                                                              data.get("cmd", ""))
                                await ws.send(json.dumps({"type": "dashboard_res",
                                                          "res": res}))
                                continue
                    finally:
                        stop_evt.set()

                await asyncio.gather(upstream(), downstream())

        except (websockets.exceptions.ConnectionClosed,
                websockets.exceptions.InvalidStatusCode,
                ConnectionRefusedError, OSError) as e:
            log.warning("relay link dropped: %s", e)
        except Exception as e:
            log.exception("relay uplink error: %s", e)

        wait = min(backoff, 30.0)
        log.info("relay reconnect in %.1fs", wait)
        audit("relay_disconnected")
        await asyncio.sleep(wait)
        backoff = min(backoff * 1.7, 30.0)


# ============================================================
# Entry
# ============================================================
async def main():
    threading.Thread(target=ur_io_thread, daemon=True).start()
    threading.Thread(target=ur_control_thread, daemon=True).start()
    if _HAS_VISION:
        threading.Thread(target=camera_thread, daemon=True).start()

    if _HAS_EXT:
        # Full telemetry + control. The existing 30003 reader in ur_io_thread
        # stays as it is; this is a second, richer view that the new panels
        # read, so nothing that already worked changes behaviour.
        res = ur_bridge_ext.UR.start(robot_host(), envelope={
            "x": (ENVELOPE["x_min"], ENVELOPE["x_max"]),
            "y": (ENVELOPE["y_min"], ENVELOPE["y_max"]),
            "z": (ENVELOPE["z_min"], ENVELOPE["z_max"]),
        })
        log.info(" UR service:   %s", res)

        # Wire the 3D reconstruction to the live camera and the live TCP pose.
        def _depth_now():
            with camera_lock:
                return global_depth_raw

        def _tcp_now():
            with data_lock:
                return list(global_tcp_pose)

        def _intr_now():
            with camera_lock:
                return global_depth_intr

        ur_bridge_ext.SCAN3D.frames_fn = _depth_now
        ur_bridge_ext.SCAN3D.pose_fn = _tcp_now
        ur_bridge_ext.SCAN3D.intrinsics_fn = _intr_now

        handeye = os.environ.get("HANDEYE_T_TCP_CAM", "")
        if handeye:
            try:
                ur_bridge_ext.SCAN3D.T_tcp_cam = json.loads(handeye)
                log.info(" Hand-eye:     loaded from HANDEYE_T_TCP_CAM")
            except Exception as e:
                log.warning("HANDEYE_T_TCP_CAM is not valid JSON: %s", e)

    if _HAS_BENCH:
        # The recorder reads robot state through this hook rather than
        # importing the globals, so the two modules stay decoupled.
        bench_agent.RECORDER.out_dir = Path(BENCH_RUN_DIR)
        bench_agent.RECORDER.out_dir.mkdir(parents=True, exist_ok=True)

        def _robot_state():
            # Prefer the new telemetry: it is version-negotiated over RTDE where
            # available, its 30003 parser refuses rather than guessing on an
            # unexpected packet length, and it carries the full field set. The
            # legacy globals stay as the fallback so recording still works if
            # the UR service failed to start.
            if _HAS_EXT and ur_bridge_ext.UR.enabled:
                st = ur_bridge_ext.UR.state()
                q = st.get("actual_q")
                pose = st.get("actual_TCP_pose")
                if q and pose:
                    return list(q), list(pose)
            with data_lock:
                return list(global_actual_q), list(global_tcp_pose)

        bench_agent.RECORDER.state_fn = _robot_state
        started = bench_agent.start_sources(
            d435i=BENCH_ENABLE_D435I_IMU and _HAS_VISION,
            fusionhub=BENCH_ENABLE_FUSIONHUB,
            fusionhub_port=BENCH_FUSIONHUB_PORT,
        )
        log.info(" Benchmark:    runs -> %s  sources=%s",
                 bench_agent.RECORDER.out_dir.resolve(), started)

    if _HAS_SENSORS:
        # Attach the drivers this agent already owns to the modality registry.
        # Everything registered here is recorded into every run automatically,
        # so the sensors still in their boxes need a descriptor, not a patch.
        def _ur_channel():
            if _HAS_EXT and ur_bridge_ext.UR.enabled:
                st = ur_bridge_ext.UR.state() or {}
                if st.get("actual_TCP_pose"):
                    return {k: st[k] for k in
                            ("actual_q", "actual_qd", "actual_TCP_pose",
                             "actual_TCP_force", "actual_current")
                            if k in st}
            return None

        def _camera_channel(cid):
            with camera_lock:
                if cid == "cam_depth":
                    if global_depth_raw is None:
                        return None
                    i = global_depth_intr
                    return {"fill": float((global_depth_raw > 0).mean())
                            if _HAS_VISION else None,
                            "intrinsics": {"fx": i.fx, "fy": i.fy, "cx": i.cx,
                                           "cy": i.cy} if i else None}
                if cid == "cam_color":
                    return {"present": global_rgb_frame is not None} \
                        if global_rgb_frame is not None else None
                if cid == "cam_ir":
                    if global_ir1_frame is None:
                        return None
                    return {"left": True, "right": global_ir2_frame is not None}
            return None

        wired = sensor_hub.wire_standard_sources(
            ur_state=_ur_channel,
            imu_latest=(bench_agent.HUB.latest if _HAS_BENCH else None),
            camera_state=_camera_channel)
        rep = sensor_hub.HUB.report()
        log.info(" Modalities:   %d registered (%d scored, %d awaiting hardware)",
                 rep["n_total"], len(rep["benchmark_channels"]), rep["n_declared"])

    log.info("=" * 64)
    log.info(" SONAIR UR Host Agent")
    log.info(" UR target:    %s", robot_host())
    log.info(" Local face:   ws://%s:%d  (UoN browser)", LOCAL_HOST, LOCAL_PORT)
    log.info(" Relay URL:    %s", RELAY_URL or "(disabled — local-only mode)")
    log.info(" Relay room:   %s", RELAY_ROOM)
    log.info(" Audit dir:    %s", AUDIT_DIR.resolve())
    if _HAS_VISION:
        log.info(" Camera:       RealSense support present")
    else:
        log.info("=" * 64)
        log.warning(" CAMERA DISABLED — the vision dependencies are missing:")
        log.warning("   %s", _VISION_ERR)
        log.warning(" No camera thread will start, so no frames will ever reach")
        log.warning(" the browser and 3D scanning will refuse to run. Install them")
        log.warning(" into THIS interpreter:")
        log.warning("   %s -m pip install pyrealsense2 opencv-python numpy", sys.executable)
        log.warning(" If pyrealsense2 has no wheel for this Python, use Python 3.11.")
        log.warning(" Everything else — the robot, FusionHub, the recorder — works.")
    log.info("=" * 64)

    # Run the local server and the relay uplink concurrently
    async with websockets.serve(local_handler, LOCAL_HOST, LOCAL_PORT,
                                max_size=8 * 1024 * 1024):
        await relay_uplink()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("agent stopped")
