"""
The measured IMU calibration, carried through the simulator and the gap.

Two "real" runs of the same commanded motion are made, both with the IMU
bolted on rotated (the -90 deg about the flange's z axis the real cell's
arc_scan showed): one whose IMU samples arrive on time, one whose samples
arrive 100 ms late (what that run's correlation gave). The simulator sees
only the commanded trajectory, so one replay serves both.

  * The late IMU, with its latency taken out, must score the same gyro gap
    as the on-time IMU. Whatever gap is left is the simulator's own.
  * The late IMU with its latency ignored must score differently.
  * With the mounting ignored the gap must be of the order of the signal
    itself, because gyro x is then being compared with gyro y.

The gap left over is NOT zero, and is not meant to be here: the menagerie
UR5e's position servo (kp 2000, kv 400) trails its command by kv/kp = 0.2 s,
where the real controller tracks within milliseconds. That is the simulator's
controller model and is what the benchmark exists to measure; it is
printed, not asserted away.

Run:  MENAGERIE=<path> python tests/test_imu_sim.py   (needs mujoco + menagerie)
"""
from __future__ import annotations

import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import ur_kin                                                 # noqa: E402
from sonair_benchmark.metrics import gap_between              # noqa: E402
from sonair_benchmark.schema import (RunManifest, RunWriter,  # noqa: E402
                                     Sample, read_run)

Q0 = [0.867, -1.621, 2.034, -1.953, -1.598, 3.203]
LAG = 0.100
R_MOUNT = ur_kin.rotmat([0.0, 0.0, -math.pi / 2])
RATE = 125.0


def real_run(path, lag):
    """Elbow and wrist in slow sinusoids; the IMU reads it late and rotated."""
    dt = 1.0 / RATE
    T = 12.0
    amp = np.radians([0, 0, 12, 15, 15, 25])
    w = np.array([0, 0, 0.9, 1.3, 1.1, 0.7])
    man = RunManifest(run_id="imu_sim_check", side="real", calib_version="t",
                      joint_vel=0.2, arm_config="mid_workspace",
                      traj_type="contour", repeat_idx=0)

    def at(t):
        t = max(0.0, t)
        q = np.asarray(Q0) + amp * (1 - np.cos(w * t))
        qd = amp * w * np.sin(w * t)
        return q, qd

    with RunWriter(path, man) as wr:
        for k in range(int(T * RATE)):
            t = k * dt
            q, qd = at(t)
            # what the IMU sample in hand at t actually describes
            ql, qdl = at(t - lag)
            gyro = R_MOUNT.T @ ur_kin.body_angular_velocity(ql, qdl)
            pose = ur_kin.fk_pose(q)
            wr.write(Sample(t=t, q=list(q), qd=list(qd), tcp_pos=pose[:3],
                            tcp_rot=pose[3:], target_q=list(q),
                            target_qd=list(qd),
                            imu={"ind0": {"gyro": list(gyro)}}))
    return read_run(path)


def main():
    import sim_mujoco
    ok, why = sim_mujoco.available()
    men = Path(os.environ.get("MENAGERIE", "mujoco_menagerie"))
    if not ok or not (men / "universal_robots_ur5e" / "scene.xml").exists():
        print("  skipped:", why or f"no menagerie at {men}")
        return
    root = Path(tempfile.mkdtemp())
    on_time = real_run(root / "a" / "imu_sim_check.jsonl", 0.0)
    late = real_run(root / "b" / "imu_sim_check.jsonl", LAG)
    cal = {"ok": True, "lag_s": LAG, "made_at": "test",
           "R_flange_imu": R_MOUNT.tolist()}

    res = sim_mujoco.replay(late, root / "sim", men, imu_cal=cal)
    assert res.get("ok"), res
    sim = read_run(res["path"])
    res = sim_mujoco.replay(late, root / "sim_nomount", men, imu_cal=None)
    assert res.get("ok"), res
    sim_nomount = read_run(res["path"])

    ref = gap_between(on_time, sim, imu_lag_s=0.0)
    good = gap_between(late, sim, imu_lag_s=LAG)
    nolag = gap_between(late, sim, imu_lag_s=0.0)
    nomount = gap_between(late, sim_nomount, imu_lag_s=LAG)
    for tag, r in (("on-time IMU", ref), ("late, corrected", good),
                   ("late, uncorrected", nolag), ("mounting ignored", nomount)):
        print(f"        {tag:<18} gyro gap median {r.gyro_median_rad_s:.4f} "
              f"rad/s, p95 {r.gyro_p95_rad_s:.4f}, relative "
              f"{r.gyro_rel_rms:.3f}")
    assert good.gyro_n > 1000
    assert abs(good.gyro_rel_rms - ref.gyro_rel_rms) < 0.01, (good, ref)
    assert abs(nolag.gyro_rel_rms - ref.gyro_rel_rms) > 0.05
    assert nomount.gyro_rel_rms > 0.8
    assert "real IMU's axes" in sim.manifest.notes
    assert "flange axes" in sim_nomount.manifest.notes
    print(f"  pass  a 100 ms late IMU, corrected, scores what an on-time one "
          f"does ({good.gyro_rel_rms:.3f} vs {ref.gyro_rel_rms:.3f})")
    print("  pass  without the mounting, the gyro gap is the size of the "
          "signal: the calibration is what makes the comparison mean anything")
    print("  note  the remainder is the simulator's own: the menagerie "
          "servo trails its command by about 0.2 s (see docstring)")
    shutil.rmtree(root, ignore_errors=True)


if __name__ == "__main__":
    main()
