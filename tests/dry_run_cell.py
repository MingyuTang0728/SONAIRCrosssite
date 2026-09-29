"""
Drive the real job runner against a simulated cell, with no hardware at all.

This is the rehearsal that is run before asking a person to stand next to a
moving arm. It exercises the actual `automation.Runner`, the actual
`BenchRecorder`, the actual pre-flight gate and the actual dataset export --
only the robot and the inertial unit are stand-ins -- and then replays the run
it produced through MuJoCo and scores the gap. Everything between the console's
buttons and a GCR number is touched once.

The inertial stand-in is a faithful LPMS-B2: it reproduces the three quirks the
real unit has, because those are what the ingestion path exists to cope with --
gyroscope in DEGREES per second, quaternion published WORLD-TO-SENSOR, and a
timestamp field that never advances. It is driven by the simulated arm's ACTUAL
joint motion rather than by a recording, so `settle_sensors` has to really
produce enough rotation for both detectors to reach a verdict. It caught two
real defects on its first run: a `carrier_com_m` the recorder would not accept,
and an operator preference reported under a verdict's name.

Run:  python tests/dry_run_cell.py
Needs mujoco and a menagerie checkout for the last stage; without them the
first five stages still run and the replay is skipped.
"""
import sys, math, time, json, tempfile, threading, types
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import automation, carrier, bench_agent, imu_link
from sonair_benchmark.attitude import GRAVITY

root = Path(tempfile.mkdtemp())

# Forward kinematics, borrowed from the simulator when it is installed.
FK = None
try:
    import os
    import sim_mujoco
    _men = Path(os.environ.get("MENAGERIE", "mujoco_menagerie"))
    if sim_mujoco.available()[0] and \
            (_men / "universal_robots_ur5e" / "scene.xml").exists():
        _arm = sim_mujoco.Arm(sim_mujoco.ensure_model(_men))
        # MuJoCo's MjData is not thread safe, and this is called from the
        # runner, the recorder and the stand-in RTDE feeder at once. Without
        # the lock the process segfaults part way through a job -- which it
        # did, the first time the feeder was added.
        _fk_lock = threading.Lock()
        def FK(q):                      # noqa: E301
            with _fk_lock:
                _arm.reset(list(q))
                return _arm.tcp_position()[:3]
except Exception as _e:      # noqa: BLE001
    FK = None
print("forward kinematics:", "from MuJoCo" if FK else "not available — the "
      "replay stage will stop at the frame check, which is correct")
carfile = root/'carrier.json'
car = carrier.save({"carrier_id":"carrier-v1","carrier_mass_kg":0.182,
                    "carrier_com_m":[0,0,0.035],"note":"LPMS-B2 + bracket"}, carfile)
assert car["ok"], car

# --- a cell that moves but has no hardware -------------------------------
Q0 = [0.867, -1.621, 2.034, -1.953, -1.598, 3.203]
class Cell:
    def __init__(self):
        self.q = list(Q0); self.qd = [0.0]*6; self.moving=False
        self.runs = root/'runs'; self.imu = root/'imu'
        bench_agent.RECORDER.out_dir = self.runs
        bench_agent.LOGGER.out_dir = self.imu
        bench_agent.RECORDER.state_fn = self.state
        self.t0 = time.time()
    def state(self):
        # A real commanded joint velocity, so the export's cell-label audit has
        # something to check. A stand-in that always reports zero would make
        # every run look mislabelled, which is the opposite of useful.
        return {"q": list(self.q), "tcp": self.tcp(), "qd": list(self.qd),
                "target_q": list(self.q), "target_qd": list(self.qd),
                "speed_scaling": 1.0}
    _pose = None
    def tcp(self):
        if self._pose is not None:
            return list(self._pose)
        # Forward kinematics from the same model the replay uses, when it is
        # available. A stand-in robot whose reported tool pose does not follow
        # its own joints is exactly the frame disagreement the replay refuses
        # to call a gap, so without this the last stage correctly stops and the
        # chain is never exercised end to end.
        if FK is not None:
            return list(FK(self.q)) + [-2.879, 1.153, 0.030]
        return [-0.180, -0.416, 0.326, -2.879, 1.153, 0.030]
    # ctx surface
    def robot_enabled(self): return True
    def robot_state(self): return {"robot_mode_text":"RUNNING","safety_mode_text":"NORMAL",
                                   "actual_q":list(self.q)}
    def robot_health(self): return {"rate_hz":125}
    def robot_host(self): return "sim"
    def tcp_pose(self): return self.tcp()
    def joints(self): return list(self.q)
    def max_joint_speed(self): return 1.5
    def max_linear_speed(self): return 0.25
    def camera_age_s(self): return None
    def camera_info(self): return {}
    def calibration(self): return None
    def imu_status(self): return bench_agent.unit_report()
    def clock_status(self): return bench_agent.MASTER.status()
    def channel_registry(self): return []
    def carrier(self): return carrier.load(carfile)
    def out_dir(self): return str(root)
    def move_to(self,*a,**k): return True, ""
    def halt(self): return True, ""
    def move_joints(self, target, speed, accel=1.2):
        """A trapezoidal joint move: ramp to `speed`, cruise, ramp down."""
        start = list(self.q)
        d = [b - a for a, b in zip(start, target)]
        dist = max(abs(x) for x in d)
        if dist < 1e-9:
            return True, ""
        ramp = speed / accel
        ramp_d = speed * speed / accel
        cruise_t = max(0.0, (dist - ramp_d) / speed)
        total = 2 * ramp + cruise_t
        t0 = time.perf_counter()
        while True:
            e = time.perf_counter() - t0
            if e >= total:
                break
            if e < ramp:
                v = accel * e
                travelled = 0.5 * accel * e * e
            elif e < ramp + cruise_t:
                v = speed
                travelled = 0.5 * speed * ramp + speed * (e - ramp)
            else:
                td = total - e
                v = accel * td
                travelled = dist - 0.5 * accel * td * td
            f = min(1.0, travelled / dist)
            self.q = [a + f * x * (1 if dist else 0) for a, x in zip(start, d)]
            self.qd = [v * (x / dist) for x in d]
            time.sleep(0.003)
        self.q = list(target)
        self.qd = [0.0] * 6
        return True, ""
    def is_recording(self): return bench_agent.RECORDER.is_recording()
    def record_start(self, args): return bench_agent.RECORDER.start(**args)
    def record_stop(self): return bench_agent.RECORDER.stop()
    def move_path(self, poses, speed, blend=0.004):
        """Walk the path at the right speed, so the timing is realistic."""
        import math
        for a, b in zip([self.tcp()] + list(poses), poses):
            d = math.dist(a[:3], b[:3])
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < d / max(speed, 1e-6):
                time.sleep(0.002)
            self._pose = list(b)
        return True, ""
    def ur_logging(self): return bench_agent.UR_LOGGER.status().get("running")
    def ur_log_start(self, path=None): return bench_agent.UR_LOGGER.start(path)
    def ur_log_stop(self): return bench_agent.UR_LOGGER.stop()
    def imu_logging(self): return bench_agent.LOGGER.status().get("running")
    def imu_log_start(self, path=None): return bench_agent.LOGGER.start(path)
    def imu_log_stop(self): return bench_agent.LOGGER.stop()
    def job_started_at(self): return self.t0
    def export_dataset(self, name):
        return automation.export_dataset(name, out_root=root/'datasets',
                                         runs_dir=self.runs, imu_dir=self.imu, ctx=self)

cell = Cell()

# --- a faithful LPMS-B2 emulator, driven by the cell's ACTUAL joint motion --
#
# Reproduces the three quirks the real unit has, because those are what the
# ingestion path has to cope with: gyroscope in DEGREES per second, quaternion
# published WORLD-TO-SENSOR, and a timestamp field that never advances.
# The carrier is rigid with the forearm, so its orientation follows joint 2.
FROZEN_NS = 1790608906450297800
stop = threading.Event()

class _FakeLink:
    """Just enough of a link for the registry: it owns the units verdict."""
    def __init__(self):
        self.units = imu_link.GyroUnits()
        self.n = 0
    def health(self):
        return {"running": True, "samples": self.n, "rate_hz": 190.0,
                "bad": 0, "age_s": 0.0, "format": "protobuf", "error": "",
                **self.units.status()}

FAKE = _FakeLink()
bench_agent.LINKS.get("ind0").link = FAKE

def feeder():
    units = FAKE.units
    prev_a, prev_t = cell.q[2], time.perf_counter()
    while not stop.is_set():
        time.sleep(1/190.0)
        now = time.perf_counter()
        a = cell.q[2]
        dt = max(now - prev_t, 1e-6)
        rate = (a - prev_a) / dt                     # rad/s about the elbow axis
        prev_a, prev_t = a, now
        # sensor->world for a rotation of `a` about x, then invert it, because
        # that is the sense this sensor publishes in.
        h = a / 2.0
        q_sw = [math.cos(h), math.sin(h), 0.0, 0.0]
        q_pub = [q_sw[0], -q_sw[1], -q_sw[2], -q_sw[3]]
        # gravity in the sensor frame under the TRUE sense, plus a little noise
        g = [0.0, math.sin(a) * GRAVITY, math.cos(a) * GRAVITY]
        rec = {"quat": q_pub,
               "gyro": [math.degrees(rate), 0.0, 0.0],      # DEGREES, like the real one
               "accel": g,
               "mag": [11.0, 6.4, 56.6]}
        rec = units.feed(rec, FROZEN_NS / 1e9, now)
        FAKE.n += 1
        bench_agent.HUB.push("ind0", bench_agent.MASTER.to_master("ind0", FROZEN_NS / 1e9), rec)

threading.Thread(target=feeder, daemon=True).start()

# --- a stand-in RTDE stream, so the robot log has something to log ----------
import ur_telemetry as _urt
_svc = _urt.URTelemetry("dry-run")
_fake_ur = types.ModuleType("ur_bridge_ext")
class _UR:
    enabled = True
    telemetry = _svc
_fake_ur.UR = _UR
sys.modules.setdefault("ur_bridge_ext", _fake_ur)
sys.modules["ur_bridge_ext"] = _fake_ur

def ur_feeder():
    i = 0
    while not stop.is_set():
        time.sleep(1 / 125.0)
        i += 1
        _svc._publish({
            "timestamp": i / 125.0,
            "actual_q": list(cell.q), "actual_qd": list(cell.qd),
            "target_q": list(cell.q), "target_qd": list(cell.qd),
            "actual_current": [0.3, -1.5, -1.9, -0.2, 0.02, -0.01],
            # A slow thermal drift, which is the sort of thing only a
            # continuous robot log ever captures.
            "joint_temperatures": [31.0 + i * 2e-4, 34.8, 38.1 + i * 3e-4,
                                   29.5, 29.4, 28.9],
            "actual_TCP_pose": cell.tcp(),
            "actual_TCP_force": [-1.1, 0.8, 1.4, 0.01, -0.04, -0.03],
            "actual_tool_accelerometer": [0.09, -0.03, -9.8],
            "speed_scaling": 1.0, "momentum": 0.42,
            "actual_main_voltage": 47.9, "tool_temperature": 30.1,
            "robot_mode": 7, "safety_mode": 1,
        }, "rtde-125")

threading.Thread(target=ur_feeder, daemon=True).start()
time.sleep(1.5)

R = automation.Runner(cell)
jobs = automation.builtin_jobs(tcp_pose=cell.tcp(), arm_config="mid_workspace")

print("=== 1. preflight BEFORE the sensors have settled ===")
pf = automation.preflight(cell, ["robot","imu"])
print("   ok:", pf["ok"], " blocking:", pf["blocking"])

print("\n=== 2. settle_sensors ===")
print("   start:", R.start(jobs["settle_sensors"]).get("ok"))
while R.status()["state"] in ("running","starting"): time.sleep(0.2)
print("   final:", R.status()["state"])

print("\n   unit report after settling:")
import pprint
_ur = bench_agent.unit_report()["ind0"]
for k in ("gyro_units","gyro_units_basis","gyro_units_evidence","gyro_peak_raw",
          "quat_convention","quat_convention_basis","units_pending","rate_hz"):
    print("     %-22s %s" % (k, _ur.get(k)))

print("\n=== 3. preflight AFTER ===")
pf = automation.preflight(cell, ["robot","imu"])
print("   ok:", pf["ok"], " blocking:", pf["blocking"])
for c in pf["checks"]:
    if c["key"] in ("imu_units","carrier","clock"):
        print(f"   [{c['state']}] {c['label']}: {c['detail'][:120]}")

print("\n=== 4. single_run ===")
print("   start:", R.start(jobs["single_run"]).get("ok"))
while R.status()["state"] in ("running","starting"): time.sleep(0.2)
st = R.status(); print("   final:", st["state"])
for e in st["log"][-8:]: print("     ", e["level"], "|", e["text"][:130])

print("\n=== 4b. arc_scan — the minute-long inspection path ===")
import scan_paths
_arc = [st for st in jobs["arc_scan"].steps if st["kind"] == "path"][0]
print("   ", scan_paths.describe(_arc["poses"], _arc["speed"]))
# Run a shortened version here: the geometry and the plumbing are what this
# rehearsal tests, and sixty seconds of wall clock proves nothing extra.
short = dict(jobs["arc_scan"].__dict__)
short["steps"] = [dict(x) for x in jobs["arc_scan"].steps]
for st in short["steps"]:
    if st["kind"] == "path":
        st["poses"] = st["poses"][::12]
        st["speed"] = 0.22
    if st["kind"] == "dwell":
        st["seconds"] = 0.2
J = automation.Job(**{k: v for k, v in short.items() if not k.startswith("_")})
print("   start:", R.start(J).get("ok"))
while R.status()["state"] in ("running", "starting"): time.sleep(0.2)
st = R.status(); print("   final:", st["state"])
for e in st["log"][-7:]: print("     ", e["level"], "|", e["text"][:145])

print("\n=== 5. export ===")
ex = cell.export_dataset("dryrun")
print("   ok:", ex.get("ok"), ex.get("path"))
stop.set()
man = json.loads((Path(ex["path"])/"manifest.json").read_text())
print("   carrier in manifest:", man["cell"]["carrier"]["carrier_id"],
      man["cell"]["carrier"]["carrier_mass_kg"], "kg")
print("   measured:", json.dumps(man["measured"], indent=4)[:600])
print("   clock:", json.dumps(man["clock"]["inertial_files_on"])[:200])
rf = sorted((Path(ex["path"])/"runs").iterdir())[0]
rm = json.loads(rf.read_text().splitlines()[0])["_manifest"]
print("   RUN manifest carrier:", rm["carrier_id"], rm["carrier_mass_kg"], rm["carrier_com_m"])
print("\nRUNFILE:", rf)


# --- 6. and the whole way through to a gap -------------------------------
print("\n=== 6. MuJoCo replay and gap ===")
try:
    import sim_mujoco
    ok, why = sim_mujoco.available()
    if not ok:
        print("   skipped:", why)
    else:
        import os
        men = Path(os.environ.get("MENAGERIE", "mujoco_menagerie"))
        if not (men / "universal_robots_ur5e" / "scene.xml").exists():
            print(f"   skipped: no menagerie at {men}. Clone it, or set "
                  f"MENAGERIE=<path>.")
        else:
            from sonair_benchmark.schema import read_dataset
            simdir = root / "sim"
            n = 0
            for r in read_dataset(Path(ex["path"]) / "runs", side="real"):
                res = sim_mujoco.replay(r, simdir, men, tcp_offset=[0, 0, 0])
                print("   replay:", res.get("ok"), res.get("error", "")[:120])
                n += 1 if res.get("ok") else 0
            if n:
                sm = json.loads(sorted(simdir.iterdir())[0]
                                .read_text().splitlines()[0])["_manifest"]
                print("   sim manifest says:", sm["notes"])
except Exception as e:      # noqa: BLE001
    print("   skipped:", e)
