"""
What made the console feel slow on the real cell, held fixed.

  1. the live IMU message says how old each unit's reading is -- the
     header lamp goes by it, and without it read "No data" over a sensor
     streaming at 100 Hz;
  2. the camera works at full rate only on demand: idle by default, full
     rate while asked for or while a job that needs it runs, and a camera
     handler waits for a frame taken after it was asked;
  3. status questions and the pendant's program list are answered beside the
     command queue, never in it, so a slow FTP refusal cannot hold up
     freedrive or a move;
  4. the spare 30003 connection gives way to the UR service instead of
     sitting in a 30 s receive;
  5. a packet subscriber (the twin, the robot log, a recording) keeps
     receiving after the robot link is restarted -- the twin used to freeze
     on the reader that had been replaced.

Run:  python tests/test_responsiveness.py
"""
from __future__ import annotations

import asyncio
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import bench_agent                                     # noqa: E402
import multimodal_bridge as mb                         # noqa: E402


def test_imu_age():
    hub = bench_agent.HUB
    hub.push("t_resp", bench_agent.MASTER.now(),
             {"gyro": [0, 0, 0], "accel": [0, 0, 9.81]})
    rec = hub.latest()["t_resp"]
    assert "age_s" in rec and 0 <= rec["age_s"] < 1.0, rec
    print(f"  pass  the live IMU message carries age_s ({rec['age_s']} s)")


def test_camera_demand():
    mb._cam_demand_until = 0.0
    assert not mb._camera_active()
    assert mb.camera_wanted(0.3) is True           # it was idle
    assert mb._camera_active()
    assert mb.camera_wanted(0.3) is False          # already wanted
    time.sleep(0.35)
    assert not mb._camera_active()

    class J:                                        # a job that needs the camera
        requires = ["robot", "camera"]

    class R:
        job, state = J(), "running"
    old = mb.RUNNER
    mb.RUNNER = R()
    try:
        assert mb._camera_active()
        R.job = type("K", (), {"requires": ["robot", "imu"]})()
        assert not mb._camera_active()
    finally:
        mb.RUNNER = old

    # a handler on an idle camera waits for a frame taken after it asked
    mb._cam_demand_until = 0.0
    mb._cam_frame_mono = 0.0
    vis = mb._HAS_VISION
    mb._HAS_VISION = True

    async def go():
        async def frame_later():
            await asyncio.sleep(0.2)
            mb._cam_frame_mono = time.monotonic()
        t = asyncio.create_task(frame_later())
        t0 = time.monotonic()
        await mb.camera_fresh(timeout_s=1.5)
        await t
        return time.monotonic() - t0
    try:
        waited = asyncio.run(go())
    finally:
        mb._HAS_VISION = vis
    assert 0.15 < waited < 1.0, waited
    print(f"  pass  camera: idle by default, full rate on demand or for a "
          f"camera job; a handler waited {waited*1000:.0f} ms for a fresh frame")


def test_queries_beside_the_queue():
    for m in ("get_urp_list", "ur_service_status", "bench_status", "jog_status",
              "auto_status"):
        assert m in mb.READ_ONLY_MESSAGES, m
    for m in ("freedrive_start", "ur_freedrive", "camp_goto", "camp_release",
              "camp_suggest", "jog_halt", "ur_service_start", "auto_run"):
        assert m not in mb.READ_ONLY_MESSAGES, m
    # a refused program list is remembered, not retried on every visit
    mb._URP_FAIL.update(host=None, until=0.0)
    old = mb.UR_IP
    mb.UR_IP = "127.0.0.9"
    try:
        t0 = time.monotonic(); mb.fetch_urp_list(); first = time.monotonic() - t0
        t0 = time.monotonic(); mb.fetch_urp_list(); second = time.monotonic() - t0
    finally:
        mb.UR_IP = old
    assert second < 0.01, (first, second)
    print(f"  pass  status questions and the program list skip the command "
          f"queue; a refused FTP list is not asked again "
          f"({first*1000:.0f} ms, then {second*1000:.1f} ms)")


def test_spare_30003_gives_way():
    src = (HERE.parent / "multimodal_bridge.py").read_text(encoding="utf-8")
    i = src.index("def ur_io_thread"); body = src[i:src.index("def _parse_res", i)]
    assert "s.settimeout(1.0)" in body and "spare 30003 connection" in body
    assert "settimeout(30.0)" not in body
    print("  pass  the spare 30003 connection closes once the UR service reads "
          "the robot")


def test_subscribers_survive_a_restart():
    sys.path.insert(0, str(HERE))
    import fake_ur
    import ur_telemetry as urt
    import ur_bridge_ext
    fake = fake_ur.FakeRTDE()
    old = urt.RTDE_PORT
    urt.RTDE_PORT = fake.port
    svc = ur_bridge_ext.URService()
    got = []
    svc.subscribe(lambda st: got.append(time.monotonic()))
    try:
        assert svc.start("127.0.0.1")["ok"]
        end = time.time() + 8
        while time.time() < end and len(got) < 50:
            time.sleep(0.05)
        n1 = len(got)
        assert svc.start("127.0.0.1")["ok"]          # what "Connect" does
        t_restart = time.monotonic()
        end = time.time() + 8
        while time.time() < end and sum(1 for t in got if t > t_restart) < 50:
            time.sleep(0.05)
        after = sum(1 for t in got if t > t_restart)
    finally:
        svc.stop()
        urt.RTDE_PORT = old
    assert n1 >= 50 and after >= 50, (n1, after)
    print(f"  pass  a subscriber keeps receiving across a link restart "
          f"({n1} packets before, {after} after)")


def main():
    test_subscribers_survive_a_restart()
    test_imu_age()
    test_camera_demand()
    test_queries_beside_the_queue()
    test_spare_30003_gives_way()
    print("all passed")


if __name__ == "__main__":
    main()
