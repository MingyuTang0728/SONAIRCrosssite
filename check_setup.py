"""
check_setup.py — pre-flight check before the first data capture.

Run this FIRST, and again after every fix. It checks, in the order that
things actually go wrong:

    1. Python version            (pyrealsense2 has no wheel for every version)
    2. Where you are running     (OneDrive and non-ASCII paths both bite)
    3. Packages                  (which are required, which only lose a feature)
    4. Repository files
    5. UR5e reachability         (per port, because each port is a separate
                                  capability and they fail independently)
    6. RealSense camera
    7. FusionHub UDP stream
    8. Write permission for the run directory

Nothing here touches the robot or moves anything. It only opens and closes
TCP connections and listens on a UDP port.

    python check_setup.py
    python check_setup.py --ur 192.168.0.20 --fusionhub-port 5005
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time
import unicodedata
from pathlib import Path

# --- output helpers ---------------------------------------------------------

_WIN = sys.platform.startswith("win")

def _supports_colour() -> bool:
    if not sys.stdout.isatty():
        return False
    if _WIN:
        # Windows Terminal and PowerShell 7 do; the old conhost does not.
        return os.environ.get("WT_SESSION") or os.environ.get("TERM_PROGRAM")
    return True

_C = _supports_colour()
def _c(code, text):
    return f"\033[{code}m{text}\033[0m" if _C else text

OK   = _c("32", "[ OK ]")
WARN = _c("33", "[WARN]")
FAIL = _c("31", "[FAIL]")
INFO = _c("36", "[ -- ]")

results = {"ok": 0, "warn": 0, "fail": 0}
todo: list[str] = []

def ok(msg):
    print(f"{OK} {msg}"); results["ok"] += 1

def warn(msg, fix=None):
    print(f"{WARN} {msg}"); results["warn"] += 1
    if fix: todo.append(("warn", msg, fix))

def fail(msg, fix=None):
    print(f"{FAIL} {msg}"); results["fail"] += 1
    if fix: todo.append(("fail", msg, fix))

def info(msg):
    print(f"{INFO} {msg}")

def section(title):
    print()
    print(_c("1", f"=== {title} ==="))


# --- 1. Python --------------------------------------------------------------

def check_python():
    section("1. Python")
    v = sys.version_info
    info(f"Python {v.major}.{v.minor}.{v.micro}  ({sys.executable})")

    if v < (3, 9):
        fail(f"Python {v.major}.{v.minor} is too old. This project needs 3.9+.",
             "Install Python 3.11 from python.org and tick 'Add python.exe to PATH'.")
    elif (v.major, v.minor) in ((3, 9), (3, 10), (3, 11)):
        ok(f"Python {v.major}.{v.minor} — pyrealsense2 publishes wheels for this version.")
    else:
        warn(f"Python {v.major}.{v.minor}: pyrealsense2 often has NO prebuilt wheel here, "
             f"so `pip install pyrealsense2` may fail even though everything else works.",
             "If the camera install fails, install Python 3.11 alongside and use "
             "`py -3.11 -m venv .venv` for this project. Everything except the "
             "camera works on any 3.9+.")

    if sys.prefix != sys.base_prefix:
        ok(f"Running inside a virtual environment ({Path(sys.prefix).name}).")
    else:
        warn("Not in a virtual environment — packages go system-wide.",
             "Create one:  py -3.11 -m venv .venv   then   .\\.venv\\Scripts\\Activate.ps1")


# --- 2. Location ------------------------------------------------------------

def check_location():
    section("2. Where this is running")
    here = Path(__file__).resolve().parent
    info(f"Project folder: {here}")

    parts = str(here)
    if "OneDrive" in parts or "Dropbox" in parts or "Google Drive" in parts:
        fail("The project is inside a cloud-synced folder (OneDrive/Dropbox/Drive).",
             "Move it to a local path such as C:\\SONAIR before recording.\n"
             "        Recording writes a JSON line 125 times a second. A sync client "
             "will fight you for the file handle,\n"
             "        can lock a run mid-capture, and with Files On-Demand a file "
             "can be evicted to the cloud\n"
             "        while the code still expects it on disk.")

    non_ascii = [ch for ch in parts if ord(ch) > 127]
    if non_ascii:
        uniq = "".join(sorted(set(non_ascii)))
        warn(f"Path contains non-ASCII characters: {uniq}",
             "Some native libraries (pyrealsense2 among them) mishandle non-ASCII "
             "paths on Windows.\n"
             "        A plain ASCII path like C:\\SONAIR removes the whole class of problem.")

    if " " in parts:
        warn("Path contains spaces — every `cd` needs quotes around it.",
             'Use:  cd "C:\\path with spaces"   or move to a path without spaces.')

    if len(parts) > 150:
        warn(f"Path is {len(parts)} characters. Windows still has 260-char limits in places.",
             "A short path like C:\\SONAIR avoids it.")

    if not non_ascii and " " not in parts and "OneDrive" not in parts:
        ok("Path is local, ASCII and space-free.")


# --- 3. Packages ------------------------------------------------------------

PACKAGES = [
    # import name,    pip name,          required?, what you lose without it
    ("websockets",    "websockets",      True,  "the bridge cannot start at all"),
    ("numpy",         "numpy",           False, "3D scanning; the panel refuses to start rather than "
                                                "producing wrong geometry"),
    ("cv2",           "opencv-python",   False, "camera image encoding, hand-eye calibration and "
                                                "the inspection pipeline"),
    ("pyrealsense2",  "pyrealsense2",    False, "the RealSense camera and its built-in IMU"),
    ("zmq",           "pyzmq",           False, "FusionHub's External Output node, which is a ZeroMQ "
                                                "publisher (tcp://*:port)"),
    ("serial",        "pyserial",        False, "an IMU connected straight to a COM port; not needed "
                                                "for FusionHub over the network"),
]


# Modules that travel with this file. A missing one is a broken checkout, not
# a missing dependency, so it is reported differently — "pip install" is not
# the fix and saying so wastes an afternoon.
LOCAL_MODULES = [
    ("imu_link",   "reading FusionHub over UDP/TCP/serial/file, and finding it"),
    ("handeye",    "measuring where the camera sits on the tool"),
    ("multiview",  "multi-view 3D scanning and path planning on the model"),
    ("rs_features", "the full camera feature set: infrared, projector modes, "
                    "filters, self-calibration"),
    ("sensor_hub", "the modality registry every recorded run is built from"),
    ("scan3d",     "3D reconstruction primitives"),
]


def check_local_modules():
    section("3b. This project's own modules")
    import importlib
    missing = []
    for mod, what in LOCAL_MODULES:
        try:
            importlib.import_module(mod)
            ok(f"{mod} — {what}")
        except ImportError as e:
            missing.append(mod)
            fail(f"{mod} will not import ({e})", f"{mod}.py is part of this project. "
                f"Run this from the folder that contains it, and check the "
                f"download is complete — pip cannot fix this one.")
        except Exception as e:      # noqa: BLE001
            fail(f"{mod} raised on import: {e}",
                "This is a fault in the file itself, not a missing package.")
    return missing

def check_packages():
    section("3. Packages")
    import importlib
    missing_required, missing_optional = [], []
    for mod, pip_name, required, loses in PACKAGES:
        try:
            m = importlib.import_module(mod)
            ver = getattr(m, "__version__", "")
            ok(f"{pip_name}{' ' + ver if ver else ''}")
        except Exception as e:
            if required:
                fail(f"{pip_name} MISSING — {loses}")
                missing_required.append(pip_name)
            else:
                warn(f"{pip_name} missing — without it you lose: {loses}")
                missing_optional.append(pip_name)

    if missing_required or missing_optional:
        allp = " ".join(missing_required + missing_optional)
        # Name THIS interpreter. A workstation has several Pythons, and a bare
        # `pip install` lands in whichever is on PATH — routinely not the one
        # the agent runs in, so the install succeeds and the import still
        # fails, which reads as the instruction being wrong.
        todo.append(("fail" if missing_required else "warn",
                     "Install the missing packages into THIS interpreter",
                     f'"{sys.executable}" -m pip install {allp}'))


# --- 4. Repository files ----------------------------------------------------

NEEDED = [
    "multimodal_bridge.py", "bench_agent.py", "ur_telemetry.py", "ur_control.py",
    "scan3d.py", "ur_bridge_ext.py", "Remote_control_Benchmark.html",
    "sonair_benchmark/__init__.py", "vendor/three/three.min.js",
]

def check_files():
    section("4. Repository files")
    here = Path(__file__).resolve().parent
    missing = [f for f in NEEDED if not (here / f).exists()]
    if missing:
        for f in missing:
            fail(f"missing: {f}")
        todo.append(("fail", "Repository is incomplete",
                     "Re-download the branch as a ZIP and extract it fully, "
                     "or `git clone` it."))
    else:
        ok(f"all {len(NEEDED)} key files present")

    # vendor/three is what makes the 3D stage work with no internet
    draco = here / "vendor" / "three" / "draco" / "draco_decoder.wasm"
    if draco.exists():
        ok("vendored three.js + DRACO decoder present (3D stage works offline)")
    else:
        warn("vendor/three/draco/ incomplete — DRACO-compressed GLB files will not load.")


# --- 5. UR5e ----------------------------------------------------------------

UR_PORTS = [
    (29999, "dashboard",         "power on/off, brakes, program control, protective-stop recovery"),
    (30002, "secondary/URScript","all motion, IO, payload, freedrive"),
    (30003, "primary realtime",  "125 Hz telemetry fallback (force, currents, temperatures)"),
    (30004, "RTDE",              "full telemetry: IO bits, program state, speed scaling"),
]

def check_ur(host: str, timeout: float = 1.5):
    section(f"5. UR5e at {host}")
    if not host:
        warn("no UR IP given — skipping (pass --ur 192.168.0.20)")
        return

    reachable = {}
    for port, name, what in UR_PORTS:
        t0 = time.time()
        s = socket.socket(); s.settimeout(timeout)
        try:
            s.connect((host, port))
            reachable[port] = True
            ok(f"port {port:5d} {name:18s} open   ({(time.time()-t0)*1000:.0f} ms) — {what}")
        except Exception as e:
            reachable[port] = False
            kind = type(e).__name__
            print(f"{FAIL if port in (30002, 30003) else WARN} "
                  f"port {port:5d} {name:18s} {kind} — {what}")
            results["fail" if port in (30002, 30003) else "warn"] += 1
        finally:
            s.close()

    if not any(reachable.values()):
        todo.append(("fail", f"No UR port answered at {host}",
                     "Check: the controller is powered on; the Ethernet cable is in;\n"
                     "        the PC and the robot are on the same subnet "
                     "(Settings > Network on the pendant);\n"
                     "        the IP is right — read it on the pendant under "
                     "Settings > System > Network."))
    elif not reachable.get(30004):
        todo.append(("warn", "RTDE (30004) not reachable",
                     "Telemetry falls back to port 30003. You still get force, currents\n"
                     "        and temperatures; you lose IO bits, program state and speed "
                     "scaling."))
    if reachable.get(30002):
        todo.append(("warn", "Before sending any motion, put the pendant in Remote Control",
                     "External URScript is refused in Local mode, and the symptom is a\n"
                     "        connection timeout rather than a clear refusal."))


# --- 6. RealSense -----------------------------------------------------------

def check_camera():
    section("6. RealSense camera")
    try:
        import pyrealsense2 as rs
    except Exception:
        warn("pyrealsense2 not installed — skipping camera check.")
        return
    try:
        ctx = rs.context()
        devices = list(ctx.query_devices())
    except Exception as e:
        fail(f"could not query RealSense devices: {e}")
        return

    if not devices:
        fail("no RealSense device found.",
             "Plug the D435i into a USB 3 port with the cable that came with it.\n"
             "        On USB 2 the camera enumerates but silently drops to a reduced\n"
             "        stream set, which reads as a bandwidth bug rather than a cable problem.\n"
             "        Confirm it appears in Intel RealSense Viewer first.")
        return

    for d in devices:
        try:
            name = d.get_info(rs.camera_info.name)
            serial = d.get_info(rs.camera_info.serial_number)
            fw = d.get_info(rs.camera_info.firmware_version)
            ok(f"{name}  serial={serial}  firmware={fw}")
            try:
                usb = d.get_info(rs.camera_info.usb_type_descriptor)
                if str(usb).startswith("2"):
                    warn(f"USB {usb} — this is USB 2. Depth and colour cannot both run "
                         f"at full rate.",
                         "Use a USB 3 port (blue) and the original cable.")
                else:
                    ok(f"USB {usb}")
            except Exception:
                pass
            sensors = [s.get_info(rs.camera_info.name) for s in d.query_sensors()]
            info(f"   sensors: {', '.join(sensors)}")
            if any("Motion" in s for s in sensors):
                ok("   built-in IMU present (this is a D435i, not a plain D435)")
            else:
                warn("   no motion module — this camera has no built-in IMU.")
        except Exception as e:
            warn(f"device present but could not be interrogated: {e}")


# --- 7. FusionHub -----------------------------------------------------------

def check_fusionhub(port: int, seconds: float = 3.0):
    section(f"7. FusionHub stream on UDP {port}")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", port))
    except OSError as e:
        warn(f"cannot bind UDP {port}: {e}",
             "Something already holds that port — most likely the bridge is already\n"
             "        running. That is fine; stop it before re-running this check.")
        s.close()
        return
    s.settimeout(0.5)

    info(f"listening for {seconds:.0f} s …")
    n, first, t0 = 0, None, time.time()
    while time.time() - t0 < seconds:
        try:
            data, _ = s.recvfrom(65535)
            n += 1
            if first is None:
                # Keep the WHOLE packet: it gets parsed below, and truncating
                # here made the parse fail on a packet that was perfectly valid.
                first = data
        except socket.timeout:
            continue
        except OSError:
            break
    s.close()

    if n == 0:
        warn("no packets received.",
             f"In FusionHub, set the output to JSON over UDP to 127.0.0.1:{port}.\n"
             "        If your build cannot do that, record in FusionHub and use the replay\n"
             "        path instead — the schema is identical:\n"
             "          python -m sonair_benchmark phase0 --fusionhub exported.csv "
             "--expected-hz 200 --out phase0/ind0.json")
        return

    rate = n / seconds
    ok(f"{n} packets in {seconds:.0f} s  (~{rate:.0f} Hz)")
    try:
        import json
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from sonair_benchmark.imu import parse_fusionhub_row
        row = json.loads(first.decode("utf-8", "ignore"))
        t_src, rec = parse_fusionhub_row(row)
        if not rec:
            warn("packets arrive but no known fields were recognised.",
                 f"First packet: {first[:160]!r}\n"
                 "        The parser accepts qw/qx/qy/qz or w/x/y/z, gx/gy/gz or "
                 "gyro_x/..., ax/ay/az or accel_x/...\n"
                 "        Tell me the actual field names and I will add them.")
        else:
            ok(f"parsed fields: {', '.join(sorted(rec))}"
               + (f"   (source timestamp present)" if t_src is not None else ""))
            if t_src is None:
                warn("no timestamp field — arrival time would be used, which carries "
                     "network jitter.")
    except Exception as e:
        warn(f"packets arrive but could not be parsed as JSON: {e}",
             f"First packet: {first[:160]!r}\n"
             "        FusionHub must be set to JSON output, not binary or OSC.")


# --- 8. Write permission ----------------------------------------------------

def check_write():
    section("8. Run directory")
    here = Path(__file__).resolve().parent
    d = Path(os.environ.get("BENCH_RUN_DIR", here / "bench_runs"))
    try:
        d.mkdir(parents=True, exist_ok=True)
        probe = d / ".write_probe"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink()
        ok(f"writable: {d}")
    except Exception as e:
        fail(f"cannot write to {d}: {e}",
             "Set BENCH_RUN_DIR to a folder you own, e.g.\n"
             '        $env:BENCH_RUN_DIR = "C:\\SONAIR\\bench_runs"')


# --- summary ----------------------------------------------------------------

def summary():
    section("Summary")
    print(f"  {results['ok']} passed   {results['warn']} warnings   {results['fail']} failures")
    if not todo:
        print()
        print(_c("32", "  Nothing to fix. Start the agent:"))
        print("      python multimodal_bridge.py")
        return 0

    print()
    print(_c("1", "  Do these, in order:"))
    n = 0
    for level, what, fix in todo:
        n += 1
        tag = _c("31", "MUST") if level == "fail" else _c("33", "note")
        print(f"\n  {n}. [{tag}] {what}")
        for line in str(fix).splitlines():
            print(f"        {line}" if not line.startswith("        ") else line)
    print()
    return 1 if results["fail"] else 0


def main():
    ap = argparse.ArgumentParser(description="SONAIR pre-flight check")
    ap.add_argument("--ur", default=os.environ.get("UR_IP", ""),
                    help="UR controller IP, e.g. 192.168.0.20")
    ap.add_argument("--fusionhub-port", type=int,
                    default=int(os.environ.get("BENCH_FUSIONHUB_PORT", 5005)))
    ap.add_argument("--listen", type=float, default=3.0,
                    help="seconds to listen for FusionHub packets")
    a = ap.parse_args()

    print(_c("1", "SONAIR pre-flight check"))
    print("Nothing here moves the robot. It only opens and closes connections.")

    check_python()
    check_location()
    check_packages()
    check_local_modules()
    check_files()
    check_ur(a.ur)
    check_camera()
    check_fusionhub(a.fusionhub_port, a.listen)
    check_write()
    return summary()


if __name__ == "__main__":
    sys.exit(main())
