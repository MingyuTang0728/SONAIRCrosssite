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
import threading
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
except ImportError:
    _HAS_VISION = False
    print("[agent] WARNING: vision deps missing (cv2/numpy/pyrealsense2). "
          "Camera streaming disabled.")


# ============================================================
# Configuration
# ============================================================
UR_IP            = os.environ.get("UR_IP", "192.168.0.20")

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
global_depth_frame = None   # D435i aligned depth (colormap JPEG)
global_ir1_frame   = None   # D435i left IR
global_ir2_frame   = None   # D435i right IR
global_actual_q    = [0.0] * 6
global_tcp_pose    = [0.0] * 6
camera_lock        = threading.Lock()
data_lock          = threading.Lock()
ur_socket_tx       = None  # 30003 long-lived send socket

# RealSense live camera config — written by camera_config messages,
# read by camera_thread on the next pipeline restart.
_rs_config_lock  = threading.Lock()
_rs_config       = {
    "stereo_res":   "640x480",
    "stereo_fps":   30,
    "depth_en":     True,
    "ir1_en":       False,
    "ir2_en":       False,
    "emitter":      "laser",
    "depth_ae":     True,
    "depth_exp":    8500,
    "depth_gain":   16,
    "post_proc":    True,
    "colormap":     2,
    "depth_units":  0.001,
    "rgb_res":      "640x360",
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
    "show_ir1":     False,
    "show_ir2":     False,
}
_rs_restart_evt  = threading.Event()  # set -> camera_thread restarts pipeline


# ============================================================
# UR low-level comms
# ============================================================
def send_dashboard_cmd(cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((UR_IP, 29999))
            s.recv(1024)
            s.sendall((cmd + "\n").encode("utf-8"))
            res = s.recv(1024).decode("utf-8").strip()
            log.info("dashboard ← %s", res)
            return res
    except Exception as e:
        return f"Error: {e}"


def send_urscript_to_robot(script_content, stop_first=False):
    """Inject URScript via port 30002.
    stop_first=False (default): inject directly without stopping UR first.
      This preserves 30003 real-time data flow (no more recv timeouts).
    stop_first=True: send dashboard stop before injection (only for long URP programs).
    """
    try:
        if stop_first:
            send_dashboard_cmd("stop")
            time.sleep(0.05)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((UR_IP, 30002))
            s.sendall(script_content.encode("utf-8"))
        log.info("urscript injected (%d bytes)", len(script_content))
        return True
    except Exception as e:
        log.error("urscript inject failed: %s", e)
        return False


def send_realtime_script(script_content):
    """Send URScript via the long-lived 30003 socket (servoj).
    Works on both CB3 and UR5e e-Series when UR is in Remote Control mode.
    Returns False (silently) if socket is not yet connected.
    """
    global ur_socket_tx
    try:
        if ur_socket_tx is not None:
            ur_socket_tx.sendall(script_content.encode("utf-8"))
            return True
        else:
            log.debug("send_realtime_script: ur_socket_tx is None (30003 not connected)")
            return False
    except Exception as e:
        log.warning("send_realtime_script failed: %s — will reconnect", e)
        ur_socket_tx = None
        return False


def fetch_urp_list():
    try:
        ftp = FTP(UR_IP, timeout=3)
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
            log.info("connecting UR 30003 @ %s ...", UR_IP)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)          # generous connect timeout
            s.connect((UR_IP, 30003))
            s.settimeout(30.0)         # recv timeout — long enough to survive brief UR pauses
            ur_socket_tx = s
            log.info("UR 30003 connected")
            buffer = b""
            while True:
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

        align      = rs.align(rs.stream.color)
        colorizer  = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, float(cfg_snap["colormap"]))

        # Spatial + temporal filters for post-processing
        spatial  = rs.spatial_filter()
        temporal = rs.temporal_filter()
        hole_fill = rs.hole_filling_filter()

        try:
            profile = pipeline.start(rscfg)
            dev     = profile.get_device()
            serial  = dev.get_info(rs.camera_info.serial_number)
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
            log.warning("camera startup failed: %s — retrying in 3 s", e)
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
                if depth_f and cfg_snap["post_proc"]:
                    depth_f = spatial.process(depth_f)
                    depth_f = temporal.process(depth_f)
                    depth_f = hole_fill.process(depth_f)

                rgb_arr   = np.asanyarray(color_f.get_data())  if color_f else None
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
        # freedrive_mode() via URScript works in Remote Control mode
        # even though PolyScope UI blocks the button (Script Manual §13.1.15).
        # Use stop_first=False to avoid disrupting 30003.
        axes = data.get("freeAxes", [1,1,1,1,1,1])
        axes_str = ",".join(str(int(a)) for a in axes)
        script = (f"def sonair_fd():"
                  f"  freedrive_mode(freeAxes=[{axes_str}])"
                  f"  sleep(30.0)"
                  f"  end_freedrive_mode()"
                  f"end")
        ok = send_urscript_to_robot(script, stop_first=False)
        audit("freedrive_start", {"freeAxes": axes})
        log.info("freedrive START axes=%s", axes)
        return ok, "ok" if ok else "ur_not_connected"

    if mtype == "freedrive_stop":
        script = ("sec sonair_fd_stop():"
                  "  end_freedrive_mode()"
                  "  stopj(2.0)"
                  "end")
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
async def local_handler(websocket):
    log.info("local browser connected")

    async def stream():
        while True:
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
                                              [cv2.IMWRITE_JPEG_QUALITY, 60])
                        frame_msg["rgb"] = base64.b64encode(buf).decode("utf-8")
                    if depth is not None:
                        _, buf = cv2.imencode(".jpg", depth,
                                              [cv2.IMWRITE_JPEG_QUALITY, 50])
                        frame_msg["depth"] = base64.b64encode(buf).decode("utf-8")
                    if ir1 is not None:
                        _, buf = cv2.imencode(".jpg", ir1,
                                              [cv2.IMWRITE_JPEG_QUALITY, 50])
                        frame_msg["ir1"] = base64.b64encode(buf).decode("utf-8")
                    if ir2 is not None:
                        _, buf = cv2.imencode(".jpg", ir2,
                                              [cv2.IMWRITE_JPEG_QUALITY, 50])
                        frame_msg["ir2"] = base64.b64encode(buf).decode("utf-8")
                    if len(frame_msg) > 1:  # at least one stream ready
                        await websocket.send(json.dumps(frame_msg))
                with data_lock:
                    q   = global_actual_q
                    tcp = global_tcp_pose
                if q:
                    await websocket.send(json.dumps({"type": "state", "q": q}))
                if tcp:
                    await websocket.send(json.dumps({"type": "tcp_pose", "q": tcp}))
                await asyncio.sleep(0.05)
            except asyncio.CancelledError:
                break
            except Exception:
                break

    stream_task = asyncio.create_task(stream())
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except Exception:
                continue
            mtype = data.get("type")

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
                continue
            if mtype == "ping":
                await websocket.send(json.dumps({"type": "pong",
                                                 "ts": data.get("ts")}))
                continue
            if mtype == "estop":
                estop_ur()
                continue
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
                continue
            if mtype == "get_urp_list":
                lst = await asyncio.to_thread(fetch_urp_list)
                await websocket.send(json.dumps({"type": "urp_list",
                                                 "list": lst}))
                continue
            if mtype == "dashboard":
                res = await asyncio.to_thread(send_dashboard_cmd,
                                              data.get("cmd", ""))
                await websocket.send(json.dumps({"type": "dashboard_res",
                                                 "res": res}))
                continue
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
                    "type":   "camera_config_ack",
                    "config": {**_rs_config},
                }))
                log.info("camera_config applied: %s", new_cfg)
                continue
            if mtype == "get_camera_config":
                # Frontend requesting current live config (e.g. on reconnect)
                with _rs_config_lock:
                    snap = dict(_rs_config)
                await websocket.send(json.dumps({
                    "type":   "camera_config_ack",
                    "config": snap,
                }))
                continue
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        stream_task.cancel()
        log.info("local browser disconnected")


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
    if _HAS_VISION:
        threading.Thread(target=camera_thread, daemon=True).start()

    log.info("=" * 64)
    log.info(" SONAIR UR Host Agent")
    log.info(" UR target:    %s", UR_IP)
    log.info(" Local face:   ws://%s:%d  (UoN browser)", LOCAL_HOST, LOCAL_PORT)
    log.info(" Relay URL:    %s", RELAY_URL or "(disabled — local-only mode)")
    log.info(" Relay room:   %s", RELAY_ROOM)
    log.info(" Audit dir:    %s", AUDIT_DIR.resolve())
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
