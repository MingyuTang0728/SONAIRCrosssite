"""
sim_mujoco.py — the simulated side of the gap, on a machine with no GPU.

Reads a real run, drives a simulated UR5e with the SAME joint commands the real
controller issued, and writes the result in the same schema the real recorder
writes. Nothing downstream has to know which side a run came from.

WHY THE COMMANDS AND NOT THE OUTCOME. Each sample of a real run carries both
what the arm did (`q`) and what its controller told it to do (`target_q`).
Replaying `q` would reproduce `q`: the simulator would be handed the answer and
the measured gap would be zero for a reason that has nothing to do with
physics. Replaying `target_q` asks the simulated plant the same question the
real plant was asked, and the difference between the two answers is the thing
being measured.

Replaying the WAYPOINTS would not work either. A UR generates its own joint
trajectory from a Cartesian target, with its own blending, its own acceleration
limits and whatever the speed slider was set to; none of that survives in a
list of waypoints. `target_q` is that generated trajectory, already resolved,
at the control rate.

WHY MUJOCO FIRST. The contract names its solver in a field and the benchmark
never imports a simulator, so this is not a commitment. MuJoCo installs with
pip on the machine already wired to the robot, needs no GPU, and is the one
environment where the plant parameters — joint damping, friction, armature —
can be fitted to the runs you have just recorded rather than taken from a
datasheet. Isaac remains the declared simulator for the camera work; the same
run files and the same scorer serve both.

WHAT IS AND IS NOT SIMULATED. The arm's rigid-body dynamics, the position
controller, and an IMU at the tool flange reading orientation, angular rate and
proper acceleration — a MuJoCo accelerometer reads 9.81 at rest, like a real
one. Not simulated: drive flexibility, backlash, thermal drift, and the
structural ringing a real wrist-mounted accelerometer sees when the arm stops.
Those are most of the gap. That is the point.

Usage:

    python sim_mujoco.py --real data/real --out data/sim \\
        --menagerie ./mujoco_menagerie [--phase0 phase0/ind0.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

try:
    import mujoco
    import numpy as np
    _HAS_MJ = True
    _MJ_ERR = ""
except Exception as e:                      # noqa: BLE001
    mujoco = None
    np = None
    _HAS_MJ = False
    _MJ_ERR = str(e)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sonair_benchmark.schema import (          # noqa: E402
    RunManifest, RunWriter, Sample, read_dataset,
)

MODEL_DIR = "universal_robots_ur5e"
WRAPPER = "_sonair_imu.xml"

# The joints, in the order a UR reports them. Asserted against the model rather
# than assumed: a model whose joints are ordered differently would replay every
# run through a permuted arm and produce a large, plausible, meaningless gap.
UR_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

WRAPPER_XML = """<mujoco model="ur5e_sonair">
  <include file="scene.xml"/>
  <!-- An IMU at the tool flange, where the real one is bolted. MuJoCo's
       accelerometer reports PROPER acceleration, so it reads 9.81 at rest
       exactly as the real part does; subtracting gravity here as well would
       double-count it. -->
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="attachment_site"/>
    <gyro name="imu_gyro" site="attachment_site"/>
    <accelerometer name="imu_acc" site="attachment_site"/>
    <framepos name="tcp_pos" objtype="site" objname="attachment_site"/>
    <framequat name="tcp_quat" objtype="site" objname="attachment_site"/>
  </sensor>
</mujoco>
"""


def available() -> tuple[bool, str]:
    if not _HAS_MJ:
        return False, (f"mujoco not importable ({_MJ_ERR}) — "
                       "run: pip install mujoco")
    return True, ""


def ensure_model(menagerie: Path) -> Path:
    """
    Write the IMU wrapper next to the menagerie model and return its path.

    It has to live INSIDE that directory: the scene declares its mesh folder
    relative to itself, and an including file elsewhere makes every asset path
    resolve from the wrong place — which fails as a missing .obj rather than as
    anything that names the cause.
    """
    d = Path(menagerie) / MODEL_DIR
    if not (d / "scene.xml").exists():
        raise FileNotFoundError(
            f"no scene.xml under {d}. Clone the model library first:\n"
            "  git clone --depth 1 "
            "https://github.com/google-deepmind/mujoco_menagerie.git")
    path = d / WRAPPER
    if not path.exists() or path.read_text(encoding="utf-8") != WRAPPER_XML:
        path.write_text(WRAPPER_XML, encoding="utf-8")
    return path


class Arm:
    """The simulated cell: model, data, and the handful of indices we need."""

    def __init__(self, model_path: Path, carrier_mass_kg: float = 0.0):
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)

        names = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, i)
                 for i in range(self.m.njnt)]
        if tuple(names[:6]) != UR_JOINTS:
            raise ValueError(
                "this model's joints are not in UR order: " + str(names[:6]))
        self.qadr = [self.m.jnt_qposadr[
            mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in UR_JOINTS]

        if self.m.nu < 6:
            raise ValueError("this model has no position actuators to drive")

        self.sens = {}
        for i in range(self.m.nsensor):
            n = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_SENSOR, i)
            self.sens[n] = (int(self.m.sensor_adr[i]), int(self.m.sensor_dim[i]))
        for need in ("imu_quat", "imu_gyro", "imu_acc", "tcp_pos", "tcp_quat"):
            if need not in self.sens:
                raise ValueError(f"the model is missing the {need} sensor")

        # The carrier, as MEASURED in Phase 1. The wrist body carries the
        # bracket and the IMU, and leaving them out understates wrist inertia —
        # which shows up as a gap that is really a bookkeeping error.
        if carrier_mass_kg > 0:
            wid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                    "wrist_3_link")
            if wid >= 0:
                self.m.body_mass[wid] += float(carrier_mass_kg)

    def set_tool_offset(self, offset):
        """
        Move the measurement site out to the tool centre point, so the sim
        reports the same physical point the robot does.
        """
        sid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE,
                                "attachment_site")
        if sid < 0:
            return
        off = [float(v) for v in list(offset)[:3]]
        self.m.site_pos[sid] = np.asarray(off, dtype=float)
        if len(offset) >= 6:
            rv = np.asarray([float(v) for v in offset[3:6]], dtype=float)
            q = np.zeros(4)
            mujoco.mju_axisAngle2Quat(q, rv / (np.linalg.norm(rv) or 1.0),
                                      float(np.linalg.norm(rv)))
            self.m.site_quat[sid] = q
        mujoco.mj_forward(self.m, self.d)

    def tcp_position(self):
        mujoco.mj_forward(self.m, self.d)
        return self.read("tcp_pos")

    def reset(self, q):
        mujoco.mj_resetData(self.m, self.d)
        for i, a in enumerate(self.qadr):
            self.d.qpos[a] = float(q[i])
        self.d.qvel[:] = 0.0
        self.d.ctrl[:6] = [float(v) for v in q[:6]]
        mujoco.mj_forward(self.m, self.d)

    def settle(self, q, seconds=0.5):
        """Let the controller take up its own error before the run starts."""
        self.d.ctrl[:6] = [float(v) for v in q[:6]]
        for _ in range(int(seconds / self.m.opt.timestep)):
            mujoco.mj_step(self.m, self.d)

    def drive(self, target_q, seconds):
        """Hold one commanded configuration for `seconds` of simulated time."""
        self.d.ctrl[:6] = [float(v) for v in target_q[:6]]
        for _ in range(max(1, int(round(seconds / self.m.opt.timestep)))):
            mujoco.mj_step(self.m, self.d)

    def read(self, name):
        a, n = self.sens[name]
        return [float(v) for v in self.d.sensordata[a:a + n]]

    def joints(self):
        return [float(self.d.qpos[a]) for a in self.qadr]

    def joint_vels(self):
        return [float(self.d.qvel[self.m.jnt_dofadr[
            mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]])
            for n in UR_JOINTS]


def _quat_to_rotvec(q):
    """MuJoCo gives (w, x, y, z); the run schema stores a rotation vector."""
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    ang = 2.0 * math.atan2(n, w)
    if ang > math.pi:
        ang -= 2.0 * math.pi
    k = ang / n
    return [x * k, y * k, z * k]


def replay(run, out_dir: Path, menagerie: Path, degrader=None,
           carrier_mass_kg: float = 0.0, settle_s: float = 0.5,
           tcp_offset=None, frame_tol_m: float = 0.05) -> dict:
    """
    One real run in, one simulated run out, same cell, same rate.

    THE FRAME CHECK IS NOT OPTIONAL. A UR reports `actual_TCP_pose` at the
    TOOL CENTRE POINT configured on the pendant; MuJoCo's attachment_site is
    the bare flange. If a tool offset is set on the robot -- and it will be,
    once a bracket and a camera are bolted on -- the two describe points a
    long way apart, and the "gap" that comes out is that offset. It is
    constant, it is large, it varies with nothing, and it looks exactly like a
    result. This run measured 913 mm before the check existed.

    So the first sample is compared in both frames and a disagreement beyond
    `frame_tol_m` stops the run rather than producing a number.
    """
    ok, why = available()
    if not ok:
        return {"ok": False, "error": why}

    samples = [s for s in run.samples if s.get("target_q")]
    if len(samples) < 10:
        return {"ok": False, "error":
                f"{run.manifest.run_id}: only {len(samples)} samples carry "
                "target_q, so there is no commanded trajectory to replay. "
                "These runs predate the commanded-channel fix and cannot be "
                "used for a gap."}

    arm = Arm(ensure_model(menagerie), carrier_mass_kg=carrier_mass_kg)
    arm.reset(samples[0]["target_q"])
    arm.settle(samples[0]["target_q"], settle_s)
    if tcp_offset:
        arm.set_tool_offset(tcp_offset)

    first_real = samples[0].get("tcp_pos")
    if first_real and len(first_real) >= 3:
        sim_pos = arm.tcp_position()
        delta = math.dist(first_real[:3], sim_pos[:3])
        if delta > frame_tol_m:
            return {"ok": False, "frame_mismatch_mm": round(delta * 1000, 1),
                    "error":
                    f"{run.manifest.run_id}: the real and simulated tool "
                    f"positions differ by {delta * 1000:.0f} mm at the very "
                    "first sample, before anything has moved. That is a frame "
                    "disagreement, not a gap: the robot reports its TOOL "
                    "CENTRE POINT as configured on the pendant, and the model "
                    "reports the bare flange. Read the tool offset off the "
                    "pendant (Installation \u2192 TCP) and pass it as "
                    "--tcp-offset x,y,z[,rx,ry,rz] in metres and radians. "
                    "Without it the gap is that offset, which is constant, "
                    "large, and varies with nothing."}

    man = run.manifest
    sim = RunManifest(
        run_id=man.run_id, side="sim", calib_version=man.calib_version,
        joint_vel=man.joint_vel, arm_config=man.arm_config,
        traj_type=man.traj_type, repeat_idx=man.repeat_idx,
        carrier_id=man.carrier_id, carrier_mass_kg=carrier_mass_kg,
        sample_rate_hz=man.sample_rate_hz, started_utc=man.started_utc,
        operator="sim_mujoco",
        notes=f"mujoco {mujoco.__version__}; replayed from {man.run_id}")
    problems = sim.validate()
    if problems:
        return {"ok": False, "error": "; ".join(problems)}

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{man.run_id}.jsonl"
    n = 0
    with RunWriter(path, sim) as w:
        prev_t = samples[0]["t"]
        for s in samples:
            dt = float(s["t"]) - prev_t
            prev_t = float(s["t"])
            # Guarded the same way the attitude tracker guards its own step: a
            # sample whose timestamp jumped would otherwise be simulated for
            # however long the jump claimed.
            if not (0.0 < dt <= 0.2):
                dt = 1.0 / max(1.0, sim.sample_rate_hz)
            arm.drive(s["target_q"], dt)

            gyro = arm.read("imu_gyro")
            acc = arm.read("imu_acc")
            quat = arm.read("imu_quat")
            if degrader is not None:
                gyro = degrader.gyro(gyro)
                acc = degrader.accel(acc)
            w.write(Sample(
                t=float(s["t"]),
                q=arm.joints(),
                qd=arm.joint_vels(),
                tcp_pos=arm.read("tcp_pos"),
                tcp_rot=_quat_to_rotvec(arm.read("tcp_quat")),
                target_q=[float(v) for v in s["target_q"]],
                target_qd=s.get("target_qd"),
                speed_scaling=s.get("speed_scaling"),
                imu={"ind0": {"quat": quat, "gyro": gyro, "accel": acc}},
            ))
            n += 1
    return {"ok": True, "path": str(path), "samples": n,
            "cell": sim.cell_key(), "repeat": sim.repeat_idx}


def _degrader_from_phase0(phase0: Path, calib_version: str):
    """Phase 0 noise and bias, applied OUTSIDE the simulator."""
    from sonair_benchmark.isaac import SimContract, SensorDegrader
    d = json.loads(Path(phase0).read_text(encoding="utf-8"))["noise_floor"]
    c = SimContract(
        calib_version=calib_version,
        gyro_noise_rad_s=tuple(d.get("gyro_noise", (0, 0, 0))[:3]),
        gyro_bias_rad_s=tuple(d.get("gyro_bias", (0, 0, 0))[:3]),
        accel_noise_m_s2=tuple(d.get("accel_noise", (0, 0, 0))[:3]),
        accel_bias_m_s2=(0.0, 0.0, 0.0),   # the bias is gravity-dominated in
                                           # the measured figure; only the
                                           # noise is a property of the part
        solver=f"mujoco-{mujoco.__version__}" if _HAS_MJ else "mujoco")
    return SensorDegrader(c, seed=7), c


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Replay real runs through MuJoCo and write the sim side")
    ap.add_argument("--real", required=True, help="folder of real run files")
    ap.add_argument("--out", required=True, help="where to write the sim runs")
    ap.add_argument("--menagerie", default="./mujoco_menagerie",
                    help="clone of google-deepmind/mujoco_menagerie")
    ap.add_argument("--phase0", default="",
                    help="phase0/ind0.json — applies the measured noise floor")
    ap.add_argument("--carrier-mass-kg", type=float, default=0.0,
                    help="MEASURED bracket + IMU mass carried at the wrist")
    ap.add_argument("--tcp-offset", default="",
                    help="the pendant's tool offset, x,y,z[,rx,ry,rz] in "
                         "metres and radians — read it off Installation > TCP")
    ap.add_argument("--frame-tol-mm", type=float, default=50.0,
                    help="how far the real and simulated tool may sit apart "
                         "at the first sample before the replay refuses")
    ap.add_argument("--contract-out", default="",
                    help="where to write the SimContract actually used")
    args = ap.parse_args(argv)

    ok, why = available()
    if not ok:
        print(why, file=sys.stderr)
        return 2

    runs = read_dataset(args.real, side="real")
    if not runs:
        print(f"no real runs under {args.real}", file=sys.stderr)
        return 1

    degrader = contract = None
    if args.phase0:
        degrader, contract = _degrader_from_phase0(
            Path(args.phase0), runs[0].manifest.calib_version)
    else:
        print("WARNING: no --phase0, so the simulated sensors are ideal. The "
              "gap will include 'the simulator has no sensor noise', which is "
              "not a result.", file=sys.stderr)

    tcp_offset = None
    if args.tcp_offset:
        try:
            tcp_offset = [float(v) for v in args.tcp_offset.split(",")]
        except ValueError:
            print("--tcp-offset must be comma-separated numbers", file=sys.stderr)
            return 2

    out = Path(args.out)
    done = failed = 0
    for r in runs:
        res = replay(r, out, Path(args.menagerie), degrader=degrader,
                     carrier_mass_kg=args.carrier_mass_kg,
                     tcp_offset=tcp_offset,
                     frame_tol_m=args.frame_tol_mm / 1000.0)
        if res.get("ok"):
            done += 1
            print(f"  {r.manifest.run_id}  ->  {res['samples']} samples  "
                  f"[{res['cell']}]")
        else:
            failed += 1
            print(f"  {r.manifest.run_id}  FAILED: {res['error']}",
                  file=sys.stderr)

    if contract is not None and args.contract_out:
        Path(args.contract_out).parent.mkdir(parents=True, exist_ok=True)
        contract.save(args.contract_out)
        print(f"contract written to {args.contract_out}")

    print(f"{done} simulated run(s) written to {out}"
          + (f", {failed} failed" if failed else ""))
    return 0 if done else 1


if __name__ == "__main__":
    raise SystemExit(main())
