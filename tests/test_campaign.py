"""
Run the campaign executor end to end against a simulated cell.

A reduced plan -- one speed, all three arm configurations, all three
trajectory types, one repeat: nine runs -- so every code path runs in about
a minute of real time: teaching, the envelope check, each motion, the
read-back check, resuming, a rejected run being re-recorded, and the refit
gate.

Run:  python tests/test_campaign.py
"""
from __future__ import annotations

import math
import shutil
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import automation                                     # noqa: E402
import bench_agent                                    # noqa: E402
import campaign_runner as cr                          # noqa: E402
import carrier                                        # noqa: E402
import ur_telemetry as urt                            # noqa: E402
from sonair_benchmark.campaign import PlannedRun      # noqa: E402

ROOT = Path(tempfile.mkdtemp())
STATE = ROOT / "state.json"
V = 0.9


class Cell:
    """A robot that moves at the commanded speed and reports it honestly."""

    def __init__(self):
        self.q = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.qd = [0.0] * 6
        self.lock = threading.Lock()
        self.carfile = ROOT / "carrier.json"
        carrier.save({"carrier_id": "carrier-v1", "carrier_mass_kg": 0.19,
                      "carrier_com_m": [-0.012, -0.037, 0.008]}, self.carfile)
        bench_agent.RECORDER.out_dir = ROOT / "runs"
        bench_agent.LOGGER.out_dir = ROOT / "imu"
        bench_agent.UR_LOGGER.out_dir = ROOT / "ur"
        bench_agent.RECORDER.state_fn = self.state

    # -- what the recorder and the UR log see --------------------------------
    def state(self):
        with self.lock:
            return {"q": list(self.q), "tcp": self.tcp_pose(), "qd": list(self.qd),
                    "target_q": list(self.q), "target_qd": list(self.qd),
                    "speed_scaling": 1.0, "robot_age_s": 0.004}

    # -- the capability bundle ------------------------------------------------
    def robot_enabled(self): return True
    def robot_state(self): return {"robot_mode_text": "RUNNING",
                                   "safety_mode_text": "NORMAL"}
    def robot_health(self): return {"rate_hz": 125, "degraded": "",
                                    "fields": ["x"] * 43, "mode": "process"}
    def robot_host(self): return "sim"
    def tcp_pose(self):
        import ur_kin
        return ur_kin.fk_pose(self.q)
    def joints(self):
        with self.lock:
            return list(self.q)
    def max_joint_speed(self): return 1.0
    def max_linear_speed(self): return 0.25
    def camera_age_s(self): return None
    def camera_info(self): return {}
    def calibration(self): return None
    def imu_status(self): return {}
    def clock_status(self): return {}
    def channel_registry(self): return []
    def carrier(self): return carrier.load(self.carfile)
    def out_dir(self): return str(ROOT)
    def halt(self): return True, ""
    def pose_allowed(self, pose):
        # A table top at z = 0.05 m in the base frame.
        return (pose[2] >= 0.05, f"z={pose[2]:.3f} below the table")

    def move_joints(self, target, speed, accel=None):
        a = float(accel or 1.2)
        start = self.joints()
        d = [b - s for s, b in zip(start, target)]
        dist = max(abs(x) for x in d)
        if dist < 1e-9:
            return True, ""
        ramp, ramp_d = speed / a, speed * speed / a
        cruise = max(0.0, (dist - ramp_d) / speed)
        total = 2 * ramp + cruise
        t0 = time.perf_counter()
        while (e := time.perf_counter() - t0) < total:
            if e < ramp:
                v, s = a * e, 0.5 * a * e * e
            elif e < ramp + cruise:
                v, s = speed, 0.5 * speed * ramp + speed * (e - ramp)
            else:
                td = total - e
                v, s = a * td, dist - 0.5 * a * td * td
            with self.lock:
                self.q = [p + x * min(1.0, s / dist) for p, x in zip(start, d)]
                self.qd = [v * x / dist for x in d]
            time.sleep(0.003)
        with self.lock:
            self.q, self.qd = list(target), [0.0] * 6
        return True, ""

    def joint_contour(self, joint, amp, w, cycles):
        q0 = self.joints()
        T = cycles * 2 * math.pi / w

        def run():
            t0 = time.perf_counter()
            while (t := time.perf_counter() - t0) < T:
                with self.lock:
                    self.q[joint] = q0[joint] + amp * (1 - math.cos(w * t))
                    self.qd[joint] = amp * w * math.sin(w * t)
                time.sleep(0.003)
            with self.lock:
                self.q[joint], self.qd[joint] = q0[joint], 0.0
        threading.Thread(target=run, daemon=True).start()
        return True, ""

    def is_recording(self): return bench_agent.RECORDER.is_recording()
    def record_start(self, args): return bench_agent.RECORDER.start(**args)
    def record_stop(self): return bench_agent.RECORDER.stop()
    def imu_logging(self): return bench_agent.LOGGER.status().get("running")
    def imu_log_start(self, path=None): return bench_agent.LOGGER.start(path)
    def imu_log_stop(self): return bench_agent.LOGGER.stop()
    def ur_logging(self): return bench_agent.UR_LOGGER.status().get("running")
    def ur_log_start(self, path=None): return bench_agent.UR_LOGGER.start(path)
    def ur_log_stop(self): return bench_agent.UR_LOGGER.stop()
    def job_started_at(self): return 0.0


def small_plan():
    runs = []
    for cfg in ("near_singular", "mid_workspace", "extended"):
        for tt in ("point_to_point", "contour", "stop_start"):
            runs.append(PlannedRun(
                run_id=f"v{V:.2f}_{cfg}_{tt}_r00".replace(".", "p"),
                joint_vel=V, arm_config=cfg, traj_type=tt, repeat_idx=0,
                session=0))
    runs.append(PlannedRun(run_id="post_refit_run", joint_vel=V,
                           arm_config="mid_workspace", traj_type="contour",
                           repeat_idx=2, session=2, refit_before=True))
    held = {runs[1].cell_key()}
    return runs, held


def run_job(R, job):
    res = R.start(job)
    assert res.get("ok"), res
    while R.status()["state"] in ("running", "starting", "stopping"):
        time.sleep(0.2)
    return R.status()


def main():
    import os
    os.chdir(ROOT)                  # the logs go where the agent runs
    cr.the_plan = small_plan
    cell = Cell()

    # a stand-in RTDE stream for the robot log
    svc = urt.URTelemetry("sim", use_process=False)
    mod = types.ModuleType("ur_bridge_ext")
    mod.UR = type("UR", (), {"enabled": True, "telemetry": svc})
    sys.modules["ur_bridge_ext"] = mod
    stop = threading.Event()

    def feed():
        while not stop.is_set():
            st = cell.state()
            svc._publish({"actual_q": st["q"], "actual_qd": st["qd"],
                          "target_q": st["q"], "target_qd": st["qd"],
                          "actual_TCP_pose": st["tcp"], "robot_mode": 7,
                          "safety_mode": 1}, "rtde")
            time.sleep(0.008)
    threading.Thread(target=feed, daemon=True).start()

    R = automation.Runner(cell)

    # 1. nothing taught: refused, and says what to do
    pv = cr.preview(0, cr.load_state(STATE), envelope_ok=cell.pose_allowed,
                    tcp_now=cell.tcp_pose(), q_now=cell.joints())
    assert not pv["ok"] and any("has not been taught" in p for p in pv["problems"])
    print("  pass  refuses a session with configurations not taught")

    # 2. a configuration whose excursion reaches the table is refused
    low = [0.0, -0.25, 1.9, -1.65, -1.57, 0.0]          # tool near the table
    for cfg in ("near_singular", "mid_workspace", "extended"):
        cr.teach(cfg, low if cfg == "extended" else cell.q, -1, STATE)
    pv = cr.preview(0, cr.load_state(STATE), envelope_ok=cell.pose_allowed,
                    tcp_now=cell.tcp_pose(), q_now=cell.joints())
    assert not pv["ok"] and any("safe envelope" in p for p in pv["problems"]), pv
    print("  pass  refuses a configuration whose elbow travel leaves the envelope")

    # teach three sound configurations
    for cfg, dq in (("near_singular", -0.3), ("mid_workspace", 0.0),
                    ("extended", 0.4)):
        q = list(cell.q)
        q[1] += dq
        cr.teach(cfg, q, -1, STATE)
    pv = cr.preview(0, cr.load_state(STATE), envelope_ok=cell.pose_allowed,
                    tcp_now=cell.tcp_pose(), q_now=cell.joints())
    assert pv["ok"], pv["problems"]
    assert pv["runs"] == 9 and pv["held_out_cells"] == 1
    print(f"  pass  preview: {pv['runs']} runs, ~{pv['minutes']} min, "
          f"{pv['held_out_cells']} held out")

    # 3. run the session
    job = cr.build_job(0, cr.load_state(STATE), STATE)
    job.requires = ["robot"]
    t0 = time.time()
    st = run_job(R, job)
    assert st["state"] == "done", [e["text"] for e in st["log"][-6:]]
    state = cr.load_state(STATE)
    done = state["done"]
    assert len(done) == 9, (sorted(done), state.get("rejected"))
    for run_id, e in done.items():
        assert Path(e["path"]).name == run_id + ".jsonl", e
    audit = automation._audit_runs(ROOT / "runs")
    for name, blk in audit.items():
        assert not [n for n in blk["notes"] if "declares" not in n], (name, blk)
        assert blk["peak_commanded_joint_vel"] >= 0.95 * V, (name, blk)
    print(f"  pass  9 planned runs recorded under their plan ids in "
          f"{time.time() - t0:.0f}s, every one reaching {V} rad/s, all marked done")

    # 4. resume: nothing left
    runs, _ = cr.session_runs(0, cr.load_state(STATE))
    assert runs == []
    print("  pass  a finished session has nothing left to run")

    # 5. a rejected run is re-recorded, and only that one
    state = cr.load_state(STATE)
    victim = sorted(state["done"])[0]
    state["rejected"][victim] = state["done"].pop(victim)
    cr.save_state(state, STATE)
    runs, _ = cr.session_runs(0, cr.load_state(STATE))
    assert [r.run_id for r in runs] == [victim]
    job = cr.build_job(0, cr.load_state(STATE), STATE)
    job.requires = ["robot"]
    st = run_job(R, job)
    assert st["state"] == "done"
    assert victim in cr.load_state(STATE)["done"]
    print("  pass  a rejected run is recorded again, and nothing else")

    # 6. the refit gate
    why = cr.refit_problem(2, cr.load_state(STATE), cell.carrier())
    assert "refit" in why, why
    carrier.save({"carrier_id": "carrier-v2", "carrier_mass_kg": 0.19,
                  "carrier_com_m": [-0.012, -0.037, 0.008]}, cell.carfile)
    time.sleep(1.1)
    carrier.save({"carrier_id": "carrier-v2", "carrier_mass_kg": 0.191,
                  "carrier_com_m": [-0.012, -0.037, 0.008]}, cell.carfile)
    assert cr.refit_problem(2, cr.load_state(STATE), cell.carrier()) == ""
    print("  pass  the post-refit session is refused until the carrier is re-described")

    # 7. held-out label is in the run's own manifest
    import json
    held_run = small_plan()[0][1].run_id
    man = json.loads((ROOT / "runs" / (held_run + ".jsonl")).read_text()
                     .splitlines()[0])["_manifest"]
    assert "HELD-OUT" in man["notes"], man["notes"]
    print("  pass  a held-out cell's run says so in its own manifest")

    stop.set()
    os.chdir(HERE)
    shutil.rmtree(ROOT, ignore_errors=True)
    print("all passed")


if __name__ == "__main__":
    main()
