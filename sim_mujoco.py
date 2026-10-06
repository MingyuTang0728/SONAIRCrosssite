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
import os
import sys
import time
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


def wrap_model(model_xml: Path) -> Path:
    """
    A SUBMITTED model (Track A), wrapped with the same sensor block S0 has.

    The contract for a submission is small and checked by Arm itself: the six
    UR joints by their UR names, in UR order; six actuators whose ctrl is the
    commanded joint position; and a site named attachment_site at the tool
    flange. Everything else -- masses, friction, armature, gains, solver -- is
    the submission. The sensors are ours, so every model is measured the same
    way, at the same point.
    """
    model_xml = Path(model_xml).resolve()
    path = model_xml.parent / f"_sonair_{model_xml.stem}.xml"
    xml = WRAPPER_XML.replace('<include file="scene.xml"/>',
                              f'<include file="{model_xml.name}"/>')
    if not path.exists() or path.read_text(encoding="utf-8") != xml:
        path.write_text(xml, encoding="utf-8")
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

        # WHERE THE FLANGE IS, remembered before anything moves it.
        #
        # `attachment_site` is not at its parent body's origin: in the
        # menagerie UR5e it sits 100 mm out along the wrist. A tool offset
        # read off the pendant is measured FROM THE FLANGE, so it has to be
        # composed with that, not written over it. Overwriting it was a 100 mm
        # error in the opposite direction from the one the frame check exists
        # to catch -- and it fired on anyone who followed the check's own
        # advice and passed --tcp-offset. Keeping the base pose here also
        # makes repeated calls idempotent instead of compounding.
        self._site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE,
                                       "attachment_site")
        if self._site >= 0:
            self._site_pos0 = np.array(self.m.site_pos[self._site], dtype=float)
            self._site_quat0 = np.array(self.m.site_quat[self._site], dtype=float)

    def set_tool_offset(self, offset):
        """
        Move the measurement site out to the tool centre point, so the sim
        reports the same physical point the robot does.

        `offset` is x,y,z[,rx,ry,rz] in the FLANGE frame, metres and radians --
        exactly what the pendant shows under Installation > TCP -- and is
        composed onto the flange's own pose in its parent body, never
        substituted for it.
        """
        sid = getattr(self, "_site", -1)
        if sid < 0:
            return
        off = np.zeros(3)
        vals = [float(v) for v in list(offset)[:3]]
        off[:len(vals)] = vals

        R0 = np.zeros(9)
        mujoco.mju_quat2Mat(R0, self._site_quat0)
        self.m.site_pos[sid] = self._site_pos0 + R0.reshape(3, 3) @ off

        if len(offset) >= 6:
            rv = np.asarray([float(v) for v in offset[3:6]], dtype=float)
            ang = float(np.linalg.norm(rv))
            qt = np.array([1.0, 0.0, 0.0, 0.0])
            if ang > 1e-12:
                mujoco.mju_axisAngle2Quat(qt, rv / ang, ang)
            out = np.zeros(4)
            mujoco.mju_mulQuat(out, self._site_quat0, qt)
            self.m.site_quat[sid] = out
        else:
            self.m.site_quat[sid] = self._site_quat0
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


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def _mat_to_quat(R):
    """Rotation matrix -> (w, x, y, z)."""
    import numpy as np
    import ur_kin
    rv = ur_kin.rotvec(np.asarray(R, dtype=float))
    ang = math.sqrt(sum(float(v) * float(v) for v in rv))
    if ang < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    k = math.sin(ang / 2) / ang
    return [math.cos(ang / 2)] + [float(v) * k for v in rv]


class ImuMount:
    """
    The real IMU's mounting, applied to the simulated one.

    MuJoCo's IMU sits at attachment_site and reads in the flange's axes (the
    site's frame and the kinematic flange frame agree to 0.000 deg). The real
    IMU reads in its own axes, rotated by however the bracket holds it.
    imu_align measures that rotation; this puts the simulated readings into
    the same axes, so gyro x is compared with gyro x.
    """

    def __init__(self, cal: dict | None):
        self.cal = cal
        self.R = None
        if cal and cal.get("R_flange_imu"):
            self.R = [[float(v) for v in row] for row in cal["R_flange_imu"]]
            self.q = _mat_to_quat(self.R)
        # The real gyro's gain against the robot's own angular velocity, as
        # imu_align measured it (the pilot sessions read 0.98). A sensor gain
        # error is not a plant gap, so the simulated gyro is given the same
        # gain rather than charging 2% of every rotation to the simulator.
        # Applied only inside a plausible band: anything outside it is a units
        # or kinematics fault that imu_align reports, not a gain.
        self.gyro_scale = 1.0
        g = (cal or {}).get("gyro_scale")
        if isinstance(g, (int, float)) and 0.8 <= g <= 1.2:
            self.gyro_scale = float(g)

    def gyro(self, v):
        return [self.gyro_scale * x for x in self.vec(v)]

    def vec(self, v):
        if self.R is None:
            return v
        # R^T v: flange axes -> IMU axes
        return [sum(self.R[r][c] * float(v[r]) for r in range(3))
                for c in range(3)]

    def quat(self, q_site):
        if self.R is None:
            return q_site
        return _qmul([float(v) for v in q_site], self.q)

    def words(self) -> str:
        if self.R is None:
            return "IMU in flange axes (no mounting calibration)"
        g = (f", gyro gain {self.gyro_scale:.3f}"
             if abs(self.gyro_scale - 1.0) > 1e-9 else "")
        return (f"IMU rotated into the real IMU's axes "
                f"(imu_cal {self.cal.get('made_at', '?')}{g})")


def _no_display() -> str:
    """Why the viewer cannot open here, or "" if it can."""
    if sys.platform.startswith(("win", "darwin")):
        return ""
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return ""
    return ("this machine has no display (neither DISPLAY nor WAYLAND_DISPLAY "
            "is set), and the graphics layer would stop the whole replay "
            "rather than just the window")


def replay(run, out_dir: Path, menagerie: Path, degrader=None,
           carrier_mass_kg: float | None = None, settle_s: float = 0.5,
           tcp_offset=None, frame_tol_m: float = 0.05,
           view: bool = False, speed: float = 1.0,
           imu_cal: dict | None = None, model_xml: Path | None = None,
           model_name: str = "") -> dict:
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

    # THE PAYLOAD COMES FROM THE RUN, not from whether the operator remembered
    # a flag. The real arm carried a bracket, a sensor and a cable; the run
    # file records what they weigh. Defaulting the simulated arm to a bare
    # flange because nobody typed --carrier-mass-kg produces a difference in
    # exactly the joint torques, overshoot and settling that a campaign
    # sweeping elbow speed exists to observe, and charges it to the gap.
    # The flag stays, as an override for deliberately asking "what if".
    from_run = float(getattr(run.manifest, "carrier_mass_kg", 0.0) or 0.0)
    if carrier_mass_kg is None:
        carrier_mass_kg = from_run
        carrier_src = "from the run's own manifest"
    else:
        carrier_mass_kg = float(carrier_mass_kg)
        carrier_src = "overridden on the command line"
        if abs(carrier_mass_kg - from_run) > 1e-4:
            print(f"WARNING {run.manifest.run_id}: replaying with "
                  f"{carrier_mass_kg:.3f} kg on the flange, but the run was "
                  f"recorded with {from_run:.3f} kg. The gap this produces "
                  f"includes the difference.", file=sys.stderr)

    samples = [s for s in run.samples if s.get("target_q")]
    if len(samples) < 10:
        return {"ok": False, "error":
                f"{run.manifest.run_id}: only {len(samples)} samples carry "
                "target_q, so there is no commanded trajectory to replay. "
                "These runs predate the commanded-channel fix and cannot be "
                "used for a gap."}

    try:
        arm = Arm(wrap_model(model_xml) if model_xml else ensure_model(menagerie),
                  carrier_mass_kg=carrier_mass_kg)
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"the model could not be used: {e}"}
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

    mount = ImuMount(imu_cal)
    man = run.manifest
    sim = RunManifest(
        run_id=man.run_id, side="sim", calib_version=man.calib_version,
        joint_vel=man.joint_vel, arm_config=man.arm_config,
        traj_type=man.traj_type, repeat_idx=man.repeat_idx,
        carrier_id=man.carrier_id, carrier_mass_kg=carrier_mass_kg,
        sample_rate_hz=man.sample_rate_hz, started_utc=man.started_utc,
        operator="sim_mujoco",
        experiment=getattr(man, "experiment", "E2"),
        notes=(f"mujoco {mujoco.__version__}; "
               + (f"model {model_name or Path(model_xml).name}; " if model_xml
                  else "model S0 (menagerie UR5e); ")
               + f"replayed from {man.run_id}; "
               f"flange payload {carrier_mass_kg:.3f} kg {carrier_src}; "
               f"{mount.words()}"))
    problems = sim.validate()
    if problems:
        return {"ok": False, "error": "; ".join(problems)}

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{man.run_id}.jsonl"
    n = 0

    # Watching it and scoring it are THE SAME RUN.
    #
    # The obvious way to add a picture is a second loop that animates the
    # recorded joint angles -- and it would show a beautiful arm tracing
    # exactly the real trajectory, because it would be playing back the
    # measurement rather than simulating anything. What has to be on screen is
    # the simulated arm being driven by the commanded trajectory, diverging
    # from the real one by however much it diverges, because that divergence is
    # the entire measurement. So the viewer is attached to the loop below, not
    # given a loop of its own, and closing the window only stops the drawing.
    viewer = None
    if view:
        # Checked BEFORE launching, not caught afterwards. With no display,
        # GLFW prints "could not initialize GLFW" and exits the PROCESS from C
        # -- no Python handler runs, no run file is written, and the work is
        # simply gone. A try/except around the launch looks like it covers this
        # and does not. So the one condition that causes it is tested first.
        why = _no_display()
        if why:
            print(f"WARNING: not opening the viewer -- {why}. Replaying "
                  f"without it; the run is written and scored either way.",
                  file=sys.stderr)
        else:
            try:
                import mujoco.viewer as _mjv
                viewer = _mjv.launch_passive(arm.m, arm.d)
            except Exception as e:      # noqa: BLE001
                print(f"WARNING: could not open the viewer ({e}); "
                      f"replaying without it.", file=sys.stderr)

    wall0 = time.perf_counter()
    t_first = float(samples[0]["t"])
    try:
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

            gyro = mount.gyro(arm.read("imu_gyro"))
            acc = mount.vec(arm.read("imu_acc"))
            quat = mount.quat(arm.read("imu_quat"))
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

            if viewer is not None:
                if not viewer.is_running():
                    viewer = None
                    continue
                viewer.sync()
                # Paced against the run's own clock, so what is on screen moves
                # at the speed the arm actually moved. Running it as fast as
                # the solver goes makes a 0.2 rad/s sweep and a 0.9 rad/s sweep
                # look identical, which defeats the point of looking.
                if speed > 0:
                    ahead = (float(s["t"]) - t_first) / speed - \
                            (time.perf_counter() - wall0)
                    if ahead > 0:
                        time.sleep(min(ahead, 0.25))
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                pass
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
    ap.add_argument("--menagerie", default="",
                    help="clone of google-deepmind/mujoco_menagerie (found by "
                         "itself if install_sim.py put it in the usual place)")
    ap.add_argument("--model", default="",
                    help="a submitted MJCF model to replay instead of S0 "
                         "(Track A; see docs/Benchmark_Tracks.md)")
    ap.add_argument("--phase0", default="",
                    help="phase0/ind0.json — applies the measured noise floor")
    ap.add_argument("--carrier-mass-kg", type=float, default=None,
                    help="override the flange payload. By default it is taken "
                         "from each run's own manifest, which is where the "
                         "measured bracket + IMU mass is recorded; pass this "
                         "only to deliberately simulate a different payload")
    ap.add_argument("--view", action="store_true",
                    help="watch it: open the MuJoCo viewer and play the "
                         "simulated arm at the speed the real one moved. It is "
                         "the SAME simulation that gets written and scored, not "
                         "a playback of the recording")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="viewer playback speed multiplier (2 = twice as fast, "
                         "0.25 = quarter speed, 0 = as fast as it will go)")
    ap.add_argument("--tcp-offset", default="",
                    help="the pendant's tool offset, x,y,z[,rx,ry,rz] in "
                         "metres and radians — read it off Installation > TCP")
    ap.add_argument("--frame-tol-mm", type=float, default=50.0,
                    help="how far the real and simulated tool may sit apart "
                         "at the first sample before the replay refuses")
    ap.add_argument("--imu-cal", default="calib/imu_cal.json",
                    help="the IMU mounting and latency measured by the "
                         "imu_mount_cal job; the simulated IMU is written in "
                         "the real IMU's axes")
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

    imu_cal = None
    try:
        import imu_align
        imu_cal = imu_align.load(args.imu_cal)
    except ImportError:
        pass
    if imu_cal is None:
        print(f"WARNING: no IMU calibration at {args.imu_cal}, so the "
              "simulated IMU is written in the flange's axes and the real one "
              "is in its own. Their gyro and accelerometer cannot be compared "
              "axis by axis. Run the imu_mount_cal job first.", file=sys.stderr)

    out = Path(args.out)
    done = failed = 0
    men = Path(args.menagerie) if args.menagerie else None
    if men is None:
        import twin
        men = next((d for d in twin.menagerie_dirs()
                    if (d / MODEL_DIR / "scene.xml").exists()),
                   Path("./mujoco_menagerie"))
    for r in runs:
        res = replay(r, out, men, degrader=degrader,
                     model_xml=Path(args.model) if args.model else None,
                     carrier_mass_kg=args.carrier_mass_kg,
                     tcp_offset=tcp_offset,
                     frame_tol_m=args.frame_tol_mm / 1000.0,
                     view=args.view, speed=args.speed, imu_cal=imu_cal)
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
