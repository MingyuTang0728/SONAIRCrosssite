"""
sim_cell.py -- the whole platform, with the robot simulated.

The console and the agent talk to a UR. This program IS one, as far as they
can tell, at its own address (127.0.0.2 by default): point the console's
robot address at it and every page, job, campaign, recording and safety
check works unchanged -- against a simulated arm.

It is built from two simulators, each doing the half it is faithful at:

  URSim     Universal Robots' own controller software (their Docker image).
            It runs our URScript exactly as the real controller does and
            generates the commanded trajectory, target_q, with the real
            controller's own motion planner -- movej profiles, speedj,
            blending, speed scaling, protective stops. Nothing here imitates
            that, because an imitation would be a second, different
            controller, and its differences would be charged to the gap.
  MuJoCo    the plant: the arm's dynamics following that target_q (the same
            model, sim_mujoco.Arm, the benchmark's S0 baseline replays), and
            the inertial sensor on the flange.

    console/agent ──RTDE──► sim_cell :30004 ◄──RTDE── URSim (target_q, state)
                  ──script/dashboard──► sim_cell ──relay──► URSim
                                         │  MuJoCo steps target_q
                  ◄──UDP IMU (5005)──────┘  actual_q, actual_qd, TCP, IMU

What the agent receives is URSim's packet with the MEASURED channels
(actual_q, actual_qd, actual_TCP_pose) replaced by the plant's, and the
simulated IMU arrives on UDP like any other inertial unit.

The cell announces itself in the RTDE handshake ("SONAIR simulated cell"),
the way a controller sends any text message. The agent reads that, labels
every run it records as simulated and keeps it out of the real dataset, and
the console says SIMULATED CELL in its header. A simulated run can never be
mistaken for a real one.

Run (once URSim is up, see docs/Simulated_Cell.md):

    python sim_cell.py                       # URSim on 127.0.0.1, serve 127.0.0.2
    python sim_cell.py --plant ursim         # no MuJoCo: URSim's own (perfect) joints
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import ur_telemetry as urt                             # noqa: E402

log = logging.getLogger("sim_cell")

MARKER = "SONAIR simulated cell"
RELAY_PORTS = (29999, 30001, 30002, 30003)
RTDE_PORT = 30004
IMU_HZ = 100.0

# RTDE command bytes
_V, _v, _O, _I, _S, _P, _U, _M = 86, 118, 79, 73, 83, 80, 85, 77


def _pkt(cmd: int, body: bytes = b"") -> bytes:
    return struct.pack(">HB", 3 + len(body), cmd) + body


def _recv_exact(sock, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf


def _recv_pkt(sock) -> tuple[int, bytes, bytes]:
    head = _recv_exact(sock, 3)
    size, cmd = struct.unpack(">HB", head)
    body = _recv_exact(sock, size - 3) if size > 3 else b""
    return cmd, body, head + body


def _text_message(text: str, source: str = "sim_cell", level: int = 2) -> bytes:
    m, s = text.encode()[:255], source.encode()[:255]
    return _pkt(_M, bytes([len(m)]) + m + bytes([len(s)]) + s + bytes([level]))


def _encode(recipe, values: dict) -> bytes:
    fmt, vals = ">", []
    for name, typ in recipe:
        code, _size = urt._FMT[typ]
        fmt += code
        v = values.get(name)
        n = {"VECTOR6D": 6, "VECTOR6INT32": 6, "VECTOR6UINT32": 6,
             "VECTOR3D": 3}.get(typ, 1)
        if v is None:
            v = [0] * n if n > 1 else 0
        if n > 1:
            vals.extend(list(v)[:n])
        else:
            vals.append(v)
    return struct.pack(fmt, *vals)


# ---------------------------------------------------------------------------
# the plant
# ---------------------------------------------------------------------------

class Plant:
    """The arm and its IMU, following URSim's commanded joints."""

    def __init__(self, kind: str = "mujoco", menagerie: str | None = None,
                 carrier_file: str = "carrier.json",
                 imu_cal_file: str = "calib/imu_cal.json",
                 imu_noise: bool = True):
        self.kind = kind
        self.arm = None
        self.mount = None
        self.why = ""
        self.imu_noise = imu_noise
        self._last_ts = None
        self._tool_off = None
        if kind == "mujoco":
            try:
                import sim_mujoco as sm
                ok, why = sm.available()
                if not ok:
                    raise RuntimeError(why)
                import twin
                cands = [Path(menagerie)] if menagerie else twin.menagerie_dirs()
                men = next((d for d in cands
                            if (d / sm.MODEL_DIR / "scene.xml").exists()), cands[0])
                mass = 0.0
                try:
                    mass = float(json.loads(Path(carrier_file).read_text())
                                 .get("carrier_mass_kg") or 0.0)
                except (OSError, ValueError):
                    pass
                self.arm = sm.Arm(sm.ensure_model(men), carrier_mass_kg=mass)
                cal = None
                try:
                    cal = json.loads(Path(imu_cal_file).read_text())
                except (OSError, ValueError):
                    pass
                self.mount = sm.ImuMount(cal)
                self.dt = float(self.arm.m.opt.timestep)
            except Exception as e:      # noqa: BLE001
                self.kind, self.why = "ursim", str(e)
                log.warning("MuJoCo plant unavailable (%s); passing URSim's "
                            "own joints through, and no IMU", e)

    def words(self) -> str:
        if self.kind == "mujoco":
            import mujoco
            return f"MuJoCo {mujoco.__version__} plant (S0 model)"
        return "URSim joints (no plant model)"

    def step(self, st: dict) -> dict:
        """Patch one URSim packet with the plant's measured channels."""
        if self.arm is None or not st.get("target_q"):
            return st
        ts = float(st.get("timestamp") or 0.0)
        if self._last_ts is None or not (0.0 < ts - self._last_ts < 0.25):
            # first packet, or URSim restarted: start the arm where it is
            self.arm.reset(st.get("actual_q") or st["target_q"])
            self._last_ts = ts
        dt = max(0.0, ts - self._last_ts)
        self._last_ts = ts
        if dt > 0:
            self.arm.drive(st["target_q"], dt)
        q, qd = self.arm.joints(), self.arm.joint_vels()
        out = dict(st)
        out["actual_q"], out["actual_qd"] = q, qd
        # The tool point: URSim's TCP offset, carried on the plant's flange.
        tcp = st.get("actual_TCP_pose")
        if tcp and st.get("actual_q"):
            try:
                import campaign_runner as cr
                if self._tool_off is None:
                    self._tool_off = cr._tool_offset(st["actual_q"], tcp)
                p = cr._tool_at(q, self._tool_off)
                out["actual_TCP_pose"] = p + list(tcp[3:6])
            except Exception:       # noqa: BLE001
                pass
        return out

    def imu(self, t: float) -> dict | None:
        if self.arm is None:
            return None
        import random
        g = self.mount.gyro(self.arm.read("imu_gyro"))
        a = self.mount.vec(self.arm.read("imu_acc"))
        qw = self.mount.quat(self.arm.read("imu_quat"))
        if self.imu_noise:
            # the measured floor of the real LPMS-B2 (E0): ~0.05 deg/s gyro
            # noise, ~0.01 m/s^2 accelerometer noise
            g = [v + random.gauss(0.0, math.radians(0.05)) for v in g]
            a = [v + random.gauss(0.0, 0.01) for v in a]
        return {"timestamp": t, "quat_w": qw[0], "quat_x": qw[1],
                "quat_y": qw[2], "quat_z": qw[3],
                "gyro_x": g[0], "gyro_y": g[1], "gyro_z": g[2],
                "accel_x": a[0], "accel_y": a[1], "accel_z": a[2]}


# ---------------------------------------------------------------------------
# the cell
# ---------------------------------------------------------------------------

class SimCell:
    def __init__(self, ursim: str = "127.0.0.1", listen: str = "127.0.0.2",
                 plant: Plant | None = None, imu_to=("127.0.0.1", 5005),
                 rtde_port: int = RTDE_PORT, ursim_rtde_port: int = RTDE_PORT,
                 relay_ports=RELAY_PORTS, frequency: float = 125.0):
        self.ursim, self.listen = ursim, listen
        self.plant = plant or Plant("ursim")
        self.imu_to = imu_to
        self.rtde_port, self.ursim_rtde_port = rtde_port, ursim_rtde_port
        self.relay_ports = relay_ports
        self.frequency = frequency
        self.types: dict[str, str] = {}
        self.version = (5, 11, 0, 0)
        self._clients: list = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.packets = 0
        self.ursim_message = ""
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._imu_next = 0.0
        self.ready = threading.Event()

    # -- URSim side ----------------------------------------------------------
    def _master(self):
        while not self._stop.is_set():
            client = urt.RTDEClient(self.ursim, self.ursim_rtde_port, self.frequency)
            try:
                client.connect()
                v = (client.controller_version or "5.11.0.0").split(".")
                self.version = tuple(int(x) for x in v[:4]) + (0,) * (4 - len(v))
                granted = client.setup_outputs(urt.OUTPUT_RECIPE)
                self.types = {n: t for n, t in granted}
                client.start()
                self.ursim_message = client.last_message
                log.info("reading URSim %s at %s: %d fields", client.controller_version,
                         self.ursim, len(granted))
                self.ready.set()
                while not self._stop.is_set():
                    st = client.read()
                    if st is None:
                        continue
                    st = self.plant.step(st)
                    self.packets += 1
                    self._broadcast(st)
                    self._send_imu(st)
            except Exception as e:      # noqa: BLE001
                log.warning("URSim link: %s; retrying", e)
                time.sleep(1.0)
            finally:
                client.close()

    def _send_imu(self, st):
        t = float(st.get("timestamp") or 0.0)
        if t < self._imu_next:
            return
        self._imu_next = max(self._imu_next + 1.0 / IMU_HZ, t) if self._imu_next else t
        rec = self.plant.imu(t)
        if rec and self.imu_to:
            try:
                self._udp.sendto(json.dumps(rec).encode(), self.imu_to)
            except OSError:
                pass

    def _broadcast(self, st):
        with self._lock:
            clients = list(self._clients)
        for c in clients:
            try:
                c["sock"].sendall(_pkt(_U, bytes([c["id"]]) + _encode(c["recipe"], st)))
            except OSError:
                with self._lock:
                    if c in self._clients:
                        self._clients.remove(c)

    # -- agent side: RTDE ------------------------------------------------------
    def _serve_rtde(self):
        srv = self._listener(self.rtde_port)
        while not self._stop.is_set():
            try:
                c, _ = srv.accept()
            except OSError:
                return
            threading.Thread(target=self._rtde_client, args=(c,), daemon=True).start()

    def _rtde_client(self, c):
        """Outputs are served from the plant; an INPUT connection (the speed
        slider) is handed through to URSim untouched."""
        self.ready.wait(30)
        try:
            while True:
                cmd, body, raw = _recv_pkt(c)
                if cmd == _V:
                    c.sendall(_pkt(_V, b"\x01"))
                    c.sendall(_text_message(
                        f"{MARKER}: URSim {'.'.join(map(str, self.version[:3]))} "
                        f"+ {self.plant.words()}"))
                elif cmd == _v:
                    c.sendall(_pkt(_v, struct.pack(">IIII", *self.version)))
                elif cmd == _O:
                    names = body[8:].decode().split(",")
                    recipe = [(n, self.types.get(n, "NOT_FOUND")) for n in names]
                    c.sendall(_pkt(_O, b"\x01" + ",".join(t for _, t in recipe).encode()))
                    granted = [(n, t) for n, t in recipe if t != "NOT_FOUND"]
                elif cmd == _S:
                    c.sendall(_pkt(_S, b"\x01"))
                    # a client that stops reading is dropped, as the real
                    # controller does, rather than stalling the plant
                    c.settimeout(0.5)
                    with self._lock:
                        self._clients.append({"sock": c, "recipe": granted, "id": 1})
                    return
                elif cmd == _I:
                    self._hand_through(c, raw)
                    return
                elif cmd == _P:
                    c.sendall(_pkt(_P, b"\x01"))
        except Exception:       # noqa: BLE001
            try:
                c.close()
            except OSError:
                pass

    def _hand_through(self, c, first_raw: bytes):
        u = socket.create_connection((self.ursim, self.ursim_rtde_port), timeout=5)
        u.sendall(_pkt(_V, struct.pack(">H", 2)))
        while _recv_pkt(u)[0] != _V:        # its reply; ours was already sent
            pass
        u.settimeout(None)
        u.sendall(first_raw)
        self._pipe(c, u)

    # -- agent side: script, dashboard, primary/realtime -----------------------
    def _serve_relay(self, port):
        srv = self._listener(port)
        while not self._stop.is_set():
            try:
                c, _ = srv.accept()
            except OSError:
                return
            try:
                u = socket.create_connection((self.ursim, port), timeout=5)
                u.settimeout(None)
            except OSError as e:
                log.warning("URSim port %d: %s", port, e)
                c.close()
                continue
            self._pipe(c, u)

    @staticmethod
    def _pipe(a, b):
        def one(src, dst):
            try:
                while True:
                    d = src.recv(65536)
                    if not d:
                        break
                    dst.sendall(d)
            except OSError:
                pass
            for s in (src, dst):
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        threading.Thread(target=one, args=(a, b), daemon=True).start()
        threading.Thread(target=one, args=(b, a), daemon=True).start()

    def _listener(self, port):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.listen, port))
        s.listen(8)
        return s

    def start(self):
        threading.Thread(target=self._master, daemon=True, name="ursim").start()
        threading.Thread(target=self._serve_rtde, daemon=True).start()
        for p in self.relay_ports:
            threading.Thread(target=self._serve_relay, args=(p,), daemon=True).start()
        return self

    def stop(self):
        self._stop.set()


URSIM_IMAGE = "universalrobots/ursim_e-series:5.11.11"
URSIM_NAME = "sonair-ursim"


def ensure_ursim(image: str = URSIM_IMAGE) -> str:
    """
    Start URSim in Docker if it is not already running, with its ports on
    127.0.0.1 ONLY -- 127.0.0.2 is this cell's, and a container publishing
    on every address would take it. Returns "" or what went wrong, in words.
    """
    import shutil
    import subprocess
    if not shutil.which("docker"):
        return ("Docker is not installed. Install Docker Desktop, or start "
                "URSim yourself and run this with --no-start-ursim")
    try:
        up = subprocess.run(["docker", "ps", "-q", "-f", f"name={URSIM_NAME}"],
                            capture_output=True, text=True, timeout=20)
    except Exception as e:      # noqa: BLE001
        return f"Docker is not answering ({e}). Is Docker Desktop running?"
    if up.returncode != 0:
        return ("Docker is not running: " + (up.stderr or "").strip()
                + ". Start Docker Desktop and try again")
    if up.stdout.strip():
        return ""
    subprocess.run(["docker", "rm", "-f", URSIM_NAME], capture_output=True)
    ports = []
    for p in (29999, 30001, 30002, 30003, 30004, 6080):
        ports += ["-p", f"127.0.0.1:{p}:{p}"]
    for img in (image, "mirror.gcr.io/" + image):
        r = subprocess.run(["docker", "run", "-d", "--name", URSIM_NAME, *ports,
                            "-e", "ROBOT_MODEL=UR5", img],
                           capture_output=True, text=True)
        if r.returncode == 0:
            log.info("URSim started from %s", img)
            return ""
        subprocess.run(["docker", "rm", "-f", URSIM_NAME], capture_output=True)
    return ("URSim could not be started: " + (r.stderr or "").strip()
            + ". Run install_sim.py first, which downloads it")


def power_on(host: str = "127.0.0.1", wait_s: float = 180.0) -> str:
    """Power the simulated arm on and release its brakes, through the
    Dashboard Server -- what pressing ON then START on the pendant does."""
    def ask(cmd):
        with socket.create_connection((host, 29999), timeout=5) as d:
            d.recv(1024)
            d.sendall((cmd + "\n").encode())
            return d.recv(1024).decode().strip()
    end = time.time() + wait_s
    while True:
        try:
            mode = ask("robotmode")
            break
        except OSError:
            if time.time() > end:
                return "URSim's Dashboard Server did not answer"
            time.sleep(3)
    if "RUNNING" in mode:
        return ""
    ask("power on")
    for _ in range(30):
        time.sleep(1)
        if "IDLE" in ask("robotmode") or "RUNNING" in ask("robotmode"):
            break
    ask("brake release")
    for _ in range(30):
        time.sleep(1)
        if "RUNNING" in ask("robotmode"):
            return ""
    return "URSim did not reach RUNNING: " + ask("robotmode")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ursim", default="127.0.0.1", help="where URSim is")
    ap.add_argument("--listen", default="127.0.0.2",
                    help="the address the console connects to as the robot")
    ap.add_argument("--plant", choices=("mujoco", "ursim"), default="mujoco")
    ap.add_argument("--menagerie", default=None,
                    help="clone of google-deepmind/mujoco_menagerie")
    ap.add_argument("--imu-port", type=int, default=5005,
                    help="UDP port the simulated IMU is sent to (0 = off)")
    ap.add_argument("--no-imu-noise", action="store_true")
    ap.add_argument("--no-start-ursim", action="store_true",
                    help="URSim is already running somewhere; do not start it")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    if not a.no_start_ursim and a.ursim in ("127.0.0.1", "localhost"):
        why = ensure_ursim()
        if why:
            print(why)
            return 1
    print("Waiting for URSim to boot and powering the arm on "
          "(the first start takes a minute)...", flush=True)
    why = power_on(a.ursim)
    if why:
        print(why)
    plant = Plant(a.plant, a.menagerie, imu_noise=not a.no_imu_noise)
    cell = SimCell(a.ursim, a.listen, plant,
                   imu_to=("127.0.0.1", a.imu_port) if a.imu_port else None).start()
    if not cell.ready.wait(60):
        print(f"URSim is not answering RTDE at {a.ursim}:{RTDE_PORT}. Is it "
              f"running, and powered on? (docs/Simulated_Cell.md)")
    print(f"\nSimulated cell ready. In the console, set the robot address to "
          f"{a.listen} and connect.\n  controller: URSim at {a.ursim}"
          f"\n  plant:      {plant.words()}"
          + (f"  ({plant.why})" if plant.why else "")
          + (f"\n  IMU:        UDP 127.0.0.1:{a.imu_port} -- on the Sensors page "
             f"choose 'UDP', port {a.imu_port}" if a.imu_port and plant.arm else "")
          + "\nCtrl-C to stop.", flush=True)
    try:
        while True:
            time.sleep(5)
            log.debug("packets %d", cell.packets)
    except KeyboardInterrupt:
        cell.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
