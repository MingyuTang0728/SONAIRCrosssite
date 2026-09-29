"""
The imu_mount_cal job, end to end, on a simulated cell.

The stand-in robot moves at the commanded speed; a stand-in IMU is bolted to
its flange at a known rotation and reports 60 ms late, through the same hub
the real sensor feeds. The job must drive the calibration motion, record both
logs, and save a calibration that finds the rotation and the latency.

Run:  python tests/test_imu_job.py
"""
from __future__ import annotations

import collections
import math
import os
import shutil
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import automation                                     # noqa: E402
import bench_agent                                    # noqa: E402
import imu_align                                      # noqa: E402
import ur_kin                                         # noqa: E402
import ur_telemetry as urt                            # noqa: E402
from test_campaign import ROOT, Cell                  # noqa: E402

LAG = 0.060
R = ur_kin.rotmat([0.4, -2.2, 0.9])


def main():
    os.chdir(ROOT)                  # the logs go where the agent runs
    cell = Cell()
    svc = urt.URTelemetry("sim", use_process=False)
    mod = types.ModuleType("ur_bridge_ext")
    mod.UR = type("UR", (), {"enabled": True, "telemetry": svc})
    sys.modules["ur_bridge_ext"] = mod
    stop = threading.Event()
    hist = collections.deque()

    def feed():
        k = 0
        rng = np.random.default_rng(3)
        while not stop.is_set():
            now = bench_agent.MASTER.now()
            st = cell.state()
            hist.append((now, st["q"], st["qd"]))
            if k % 3 == 0:
                svc._publish({"actual_q": st["q"], "actual_qd": st["qd"],
                              "target_q": st["q"], "target_qd": st["qd"],
                              "actual_TCP_pose": st["tcp"], "robot_mode": 7,
                              "safety_mode": 1}, "rtde")
            # the IMU describes the flange as it was LAG ago
            while len(hist) > 1 and hist[1][0] <= now - LAG:
                hist.popleft()
            _, q, qd = hist[0]
            g = R.T @ ur_kin.body_angular_velocity(q, qd) + rng.normal(0, 0.003, 3)
            up = ur_kin.fk(q)[:3, :3].T @ np.array([0, 0, imu_align.G])
            a = R.T @ up + rng.normal(0, 0.02, 3)
            bench_agent.HUB.push("ind0", now, {"gyro": list(g), "accel": list(a)})
            k += 1
            time.sleep(0.003)
    threading.Thread(target=feed, daemon=True).start()
    time.sleep(0.5)

    import campaign_runner as cr
    import carrier
    assert "not been measured" in cr.imu_cal_note(cell.carrier(), ROOT)

    R_ = automation.Runner(cell)
    job = automation.builtin_jobs(cell.tcp_pose())["imu_mount_cal"]
    job.requires = ["robot"]
    t0 = time.time()
    assert R_.start(job).get("ok")
    while R_.status()["state"] in ("running", "starting", "stopping"):
        time.sleep(0.2)
    st = R_.status()
    log = [e["text"] for e in st["log"]]
    assert st["state"] == "done", log[-8:]
    cal = imu_align.load(ROOT / imu_align.CAL_PATH)
    assert cal, log[-8:]
    err_ms = abs(cal["lag_s"] - LAG) * 1000
    err_deg = math.degrees(np.linalg.norm(
        ur_kin.rotvec(np.asarray(cal["R_flange_imu"]).T @ R)))
    assert err_ms < 5, cal["lag"]
    assert err_deg < 1.5, cal["mount"]
    assert max(abs(a - b) for a, b in zip(cell.joints(), cell.q)) < 1e-9
    print(f"  pass  imu_mount_cal ran in {time.time() - t0:.0f} s and found the "
          f"latency within {err_ms:.1f} ms and the mounting within "
          f"{err_deg:.2f} deg")
    for line in log:
        if "IMU" in line or "gyro" in line or "gravity" in line:
            print("        " + line)

    assert cr.imu_cal_note(cell.carrier(), ROOT) == ""
    time.sleep(1.1)
    carrier.save({**cell.carrier(), "carrier_id": "carrier-v2"}, cell.carfile)
    assert "re-described" in cr.imu_cal_note(cell.carrier(), ROOT)
    print("  pass  the campaign preview asks for a calibration when there is "
          "none, and again after the carrier is re-described")

    stop.set()
    os.chdir(HERE)
    shutil.rmtree(ROOT, ignore_errors=True)
    print("all passed")


if __name__ == "__main__":
    main()
