"""
Two faults found in Session 1, and their fixes.

  1. The contour's commanded acceleration was a staircase. Its velocity was
     stepped every 8 ms and the controller met each step at the full speedj
     acceleration, so a 0.9 rad/s sinusoid (true peak 2.3 rad/s^2) was
     commanded at 5 rad/s^2 in bursts, and the current of two identical runs
     differed by up to 1.3 A. The program is executed here against a model
     of speedj on a 2 ms control tick: stepped every tick, the commanded
     acceleration is the sinusoid's own.
  2. Only contour runs are re-recorded for that (protocol 3); point-to-point
     and stop-start runs from protocol 2 still count.
  3. The session was ended by the inertial sensor going off the air for more
     than 30 s between runs. A job now pauses -- the arm standing still --
     until the sensor is back, and a run spoiled by a dropout part-way
     through is recorded again at once.

Run:  python tests/test_session_robustness.py
"""
from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import automation                                     # noqa: E402
import bench_agent                                    # noqa: E402
import campaign_runner as cr                          # noqa: E402
import ur_control                                     # noqa: E402
import test_campaign as tc                            # noqa: E402

TICK = 0.002            # e-series control period


def commanded_accel(prog: str) -> list[float]:
    """Run a joint_sine program against speedj on a 2 ms tick; per-tick qdd."""
    v = 0.0
    out = []

    def speedj(vec, a, t):
        nonlocal v
        target = [x for x in vec if x != 0] or [0.0]
        target = target[0]
        for _ in range(max(1, round(t / TICK))):
            dv = max(-a * TICK, min(a * TICK, target - v))
            v += dv
            out.append(dv / TICK)
    import re
    py = "\n".join(l for l in prog.splitlines() if l.strip() != "end")
    py += "\n" + re.match(r"def (\w+)\(\):", prog.strip()).group(1) + "()\n"
    exec(compile(py, "<urscript>", "exec"),
         {"sin": math.sin, "speedj": speedj, "stopj": lambda a: None,
          "get_target_joint_positions": lambda: [0.0] * 6,
          "movej": lambda q, a=1.2, v=0.3: None})
    return out


def test_contour_acceleration():
    sent = []
    c = ur_control.URController.__new__(ur_control.URController)
    c.script = type("S", (), {"send": lambda self, p: (sent.append(p), (True, ""))[1]})()
    c.envelope = ur_control.Envelope()
    A = math.radians(20.0)
    v = 0.9
    w = v / A
    true_peak = A * w * w
    c.joint_sine(2, A, w, 2)
    new = max(abs(x) for x in commanded_accel(sent[-1]))
    c.joint_sine(2, A, w, 2, dt=0.008)
    old = max(abs(x) for x in commanded_accel(sent[-1]))
    assert "0.002000" in sent[0]
    assert new < 1.05 * true_peak, (new, true_peak)
    assert old > 1.8 * true_peak, (old, true_peak)
    print(f"  pass  contour at {v} rad/s: commanded acceleration peaks at "
          f"{new:.2f} rad/s^2 stepped every 2 ms (true {true_peak:.2f}), "
          f"against {old:.2f} stepped every 8 ms")


def test_protocol_3_only_redoes_contour():
    st = {"configs": {}, "rejected": {}, "sessions": {}, "done": {
        "v0p50_near_singular_contour_r00": {"protocol": 2},
        "v0p50_near_singular_point_to_point_r00": {"protocol": 2},
        "v0p50_near_singular_stop_start_r00": {"protocol": 2},
        "v0p50_near_singular_contour_r03": {"protocol": 3},
        "v0p50_near_singular_stop_start_r03": {}}}
    got = cr.done_ids(st)
    assert got == {"v0p50_near_singular_point_to_point_r00",
                   "v0p50_near_singular_stop_start_r00",
                   "v0p50_near_singular_contour_r03"}, got
    print("  pass  protocol 3: protocol-2 contour runs are recorded again; "
          "point-to-point and stop-start runs keep counting; pilot runs do not")


def test_pause_for_imu_and_rerecord():
    import os
    os.chdir(tc.ROOT)
    state = tc.ROOT / "state_pause.json"
    cell = tc.Cell()
    # a sensor whose scales were settled earlier (see test_quat_convention)
    cell.imu_status = lambda: {
        u: {**v, "gyro_units": "rad", "quat_convention": "direct"}
        for u, v in bench_agent.unit_report().items()}

    # the sensor: on, off for 2 s from t=1.0 (during the first recording and
    # across the start of the next), then on again
    t_start = [None]
    stop = threading.Event()

    def imu():
        k = 0
        while not stop.is_set():
            el = time.monotonic() - t_start[0] if t_start[0] else 0.0
            if not (1.0 <= el < 3.0):
                bench_agent.HUB.push("ind0", bench_agent.MASTER.now(), {
                    "gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.0, 9.81],
                    "quat": [1.0, 0.0, 0.0, 0.0]})
                k += 1
            time.sleep(0.01)
    threading.Thread(target=imu, daemon=True).start()
    time.sleep(1.5)                                     # the sensor is live

    rid = "v0p20_mid_workspace_point_to_point_r00"
    job = automation.Job(name="pause_test", requires=["robot", "imu"], steps=[
        {"kind": "goto_joints", "q": list(cell.q), "speed": 0.4,
         "run_start": True},
        {"kind": "record_start", "run_id_exact": rid, "joint_vel": 0.0,
         "arm_config": "mid_workspace", "traj_type": "point_to_point",
         "repeat_idx": 0},
        {"kind": "dwell", "seconds": 1.5},
        {"kind": "record_stop"},
        {"kind": "campaign_mark", "run_id": rid, "state_path": str(state)},
    ])
    R = automation.Runner(cell)
    t_start[0] = time.monotonic()
    out = tc.run_job(R, job)
    stop.set()
    log = [e["text"] for e in out["log"]]
    assert out["state"] == "done", log[-6:]
    assert any("dropped out during this run" in l for l in log), log
    assert any(l.startswith("Paused: the inertial sensor") for l in log), log
    assert any("is back after" in l for l in log), log
    st = cr.load_state(state)
    assert rid in st["done"] and rid not in st["rejected"], st
    kept = list((tc.ROOT / "runs").glob(rid + ".jsonl.*.superseded"))
    assert kept, "the spoiled recording was not kept"
    waited = [l for l in log if "is back after" in l][0]
    print(f"  pass  sensor lost mid-run: the run was recorded again after a "
          f"pause ({waited.split('after ')[1].split(';')[0]}), the spoiled "
          f"file kept as {kept[0].name[-30:]}, and the job finished")


def main():
    test_contour_acceleration()
    test_protocol_3_only_redoes_contour()
    test_pause_for_imu_and_rerecord()
    print("all passed")


if __name__ == "__main__":
    main()
