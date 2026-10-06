"""
The OpenZen link: an LPMS sensor read without FusionHub, kept alive, and
timed by its own clock -- tested against a stand-in `openzen` module that
uses the real binding's names and delivers like Bluetooth does.

  1. it streams, in SI units, and timestamps come from the sensor's clock:
     evenly spaced although the radio delivers in 40 ms bursts;
  2. a Bluetooth disconnect is followed by a reconnect, by itself, and the
     outage is counted and timed;
  3. a sensor that goes silent without saying so is noticed and reconnected;
  4. a reader process that dies is started again;
  5. readings lost on the radio are counted from the sensor's frame counter;
  6. no OpenZen installed: the link says so, plainly, and does not loop;
  7. the Find button's search returns the sensor;
  8. end to end through the agent: a run recorded across a dropout is
     rejected by the read-back, a clean one is not.

Run:  python tests/test_openzen_link.py
"""
from __future__ import annotations

import os
import statistics
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import imu_link                                       # noqa: E402

FAKE = str(HERE / "fake_openzen")
TMP = Path(tempfile.mkdtemp())


def env(**kw):
    for k in list(os.environ):
        if k.startswith("FAKE_OZ_"):
            del os.environ[k]
    for k, v in kw.items():
        os.environ[k] = str(v)


def make(**kw):
    got = []
    link = imu_link.make_link(
        "openzen", "ind0", on_sample=lambda t, r: got.append(
            (t, time.perf_counter(), r)),
        python=sys.executable, zen_dir=FAKE, **kw)
    return link, got


def wait_for(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_streams_in_si_on_the_sensor_clock():
    env(FAKE_OZ_BURST_S=0.04)
    link, got = make()
    assert link.start()["ok"]
    assert wait_for(lambda: len(got) > 300, 10), link.health()
    link.stop()
    h = link.health()
    t = [g[0] for g in got[50:]]
    arr = [g[1] for g in got[50:]]
    dt = [b - a for a, b in zip(t, t[1:])]
    da = [b - a for a, b in zip(arr, arr[1:])]
    _, _, rec = got[-1]
    acc = sum(x * x for x in rec["accel"]) ** 0.5
    assert abs(acc - 9.80665) < 1e-3, rec
    assert abs(rec["gyro"][2] - 0.5235988) < 1e-4, rec       # 30 deg/s
    assert len(rec["quat"]) == 4
    assert statistics.pstdev(dt) < 0.0005, statistics.pstdev(dt)
    assert statistics.pstdev(da) > 0.005                      # arrival is bursty
    assert all(a >= b - 1e-6 for (b, a) in zip(t, arr))       # never after arrival
    assert h["timebase"] == "sensor" and h["gyro_units"] == "rad"
    print(f"  pass  streams in m/s^2 and rad/s; sensor-clock spacing jitter "
          f"{statistics.pstdev(dt) * 1000:.3f} ms against "
          f"{statistics.pstdev(da) * 1000:.1f} ms by arrival")


def test_reconnects_after_a_drop():
    state = TMP / "drop"
    env(FAKE_OZ_DROP_AT=1.0, FAKE_OZ_DOWN_S=1.5, FAKE_OZ_STATE=state)
    link, got = make()
    link.start()
    assert wait_for(lambda: link.health()["outages_10min"] >= 1, 20), link.health()
    n = len(got)
    assert wait_for(lambda: len(got) > n + 100, 5)
    h = link.health()
    link.stop()
    assert h["outages_10min"] == 1 and 1.4 < h["longest_outage_s"] < 6, h
    print(f"  pass  Bluetooth drop: reconnected by itself after "
          f"{h['longest_outage_s']:.1f} s, outage counted")


def test_reconnects_after_a_silent_stall():
    state = TMP / "stall"
    env(FAKE_OZ_STALL_AT=1.0, FAKE_OZ_STATE=state)
    link, got = make(stall_s=0.6)
    link.start()
    assert wait_for(lambda: link.health()["outages_10min"] >= 1, 15), link.health()
    h = link.health()
    link.stop()
    assert h["outages_10min"] == 1 and h["reconnects"] == 1, h
    assert h["longest_outage_s"] >= 0.6, h      # from the last reading, not
                                                # from when it was noticed
    print(f"  pass  silent stall noticed and reconnected "
          f"({h['longest_outage_s']:.1f} s without data)")


def test_respawns_a_dead_reader():
    env()
    link, got = make()
    link.start()
    assert wait_for(lambda: len(got) > 50, 10)
    link._proc.kill()
    n = len(got)
    assert wait_for(lambda: len(got) > n + 100, 15), link.health()
    h = link.health()
    link.stop()
    assert h["reader_restarts"] == 1, h
    print("  pass  a reader process that dies is started again")


def test_counts_frames_lost_on_the_radio():
    env(FAKE_OZ_LOSE_EVERY=25, FAKE_OZ_FC_STEP=4)
    link, got = make()
    link.start()
    assert wait_for(lambda: len(got) > 700, 12)
    link.stop()
    pct = link.health()["frames_lost_pct"]
    assert 3.0 < pct < 5.0, pct                      # 1 in 25 = 4%
    print(f"  pass  {pct:.1f}% of frames lost on the radio, counted from the "
          f"sensor's frame counter (which steps by 4)")


def test_uneven_frame_steps_are_not_losses():
    """The real LPMS-B2 case: full rate arriving, frame steps not all 4."""
    env(FAKE_OZ_FC_JITTER=1, FAKE_OZ_FC_STEP=4)
    link, got = make()
    link.start()
    assert wait_for(lambda: len(got) > 700, 12)
    link.stop()
    pct = link.health()["frames_lost_pct"]
    assert pct < 0.5, pct
    print(f"  pass  uneven frame-counter steps with nothing lost read {pct:.1f}% "
          f"lost (by the sensor clock), not a false loss")


def test_not_installed_is_said_plainly():
    env()
    link = imu_link.make_link("openzen", "ind0", python=sys.executable,
                              zen_dir=str(TMP / "nothing-here"))
    link.start()
    assert wait_for(lambda: link.state == "failed", 10), link.health()
    h = link.health()
    link.stop()
    assert "OpenZen could not be loaded" in h["error"], h
    assert h["reader_restarts"] == 0
    print("  pass  without OpenZen: one plain error, no retry loop")


def test_find_lists_the_sensor():
    env()
    os.environ["SONAIR_OPENZEN_PYTHON"] = sys.executable
    os.environ["SONAIR_OPENZEN_DIR"] = FAKE
    try:
        res = imu_link.list_openzen(seconds=2)
    finally:
        del os.environ["SONAIR_OPENZEN_PYTHON"], os.environ["SONAIR_OPENZEN_DIR"]
    assert res["ok"] and res["sensors"][0]["name"].startswith("LPMSB2"), res
    assert res["sensors"][0]["identifier"] == "00:04:3E:4B:31:41"
    print("  pass  Find lists the sensor with its Bluetooth address")


def test_dropout_rejects_the_run():
    """Through the agent: the hub, the recorder, and the read-back."""
    import automation
    import bench_agent
    state = TMP / "rundrop"
    runs = TMP / "runs"
    bench_agent.RECORDER.out_dir = runs
    def robot():                    # an elbow turning at the labelled 0.25 rad/s
        q = [0.0, -1.57, 0.25 * time.perf_counter() % 1.0, -1.57, -1.57, 0.0]
        qd = [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]
        return {"q": q, "tcp": [0.4, 0, 0.3, 0, 3.14, 0], "qd": qd,
                "target_q": q, "target_qd": qd, "speed_scaling": 1.0,
                "robot_age_s": 0.004}
    bench_agent.RECORDER.state_fn = robot
    os.environ["SONAIR_OPENZEN_PYTHON"] = sys.executable
    os.environ["SONAIR_OPENZEN_DIR"] = FAKE
    results = {}
    try:
        for name, kw in (("clean", {}),
                         ("dropped", dict(FAKE_OZ_DROP_AT=2.5, FAKE_OZ_DOWN_S=1.0,
                                          FAKE_OZ_STATE=state))):
            env(**kw)
            link = bench_agent.LINKS.get("ind0")
            assert link.start("openzen", {}, "auto"), link.status()
            assert wait_for(lambda: (bench_agent.unit_report().get("ind0") or {})
                            .get("rate_hz", 0) > 50, 10)
            res = bench_agent.RECORDER.start(
                run_id=f"oz_{name}", joint_vel=0.2, arm_config="mid_workspace",
                traj_type="contour", repeat_idx=0, calib_version="t",
                rate_hz=50.0)
            assert res.get("ok"), res
            time.sleep(5.0)
            stop = bench_agent.RECORDER.stop()
            link.stop()
            p = Path(stop["path"])
            results[name] = automation._audit_runs(p.parent, only=p.name)[p.name]
    finally:
        del os.environ["SONAIR_OPENZEN_PYTHON"], os.environ["SONAIR_OPENZEN_DIR"]
    clean, dropped = results["clean"], results["dropped"]
    assert not [n for n in clean["notes"] if "inertial" in n], clean
    assert clean["imu_age_max_s"]["ind0"] < automation.IMU_GAP_S, clean
    bad = [n for n in dropped["notes"] if "inertial sensor ind0 stopped" in n]
    assert bad, dropped
    import campaign_runner as cr
    m = cr.mark(TMP / "state.json", "oz_dropped", 0, "x", dropped)
    assert not m["ok"]
    m = cr.mark(TMP / "state.json", "oz_clean", 0, "x", clean)
    assert m["ok"], m
    print(f"  pass  clean run accepted (worst IMU age "
          f"{clean['imu_age_max_s']['ind0'] * 1000:.0f} ms); run across a "
          f"dropout rejected: {bad[0][:58]}...")


def main():
    test_streams_in_si_on_the_sensor_clock()
    test_reconnects_after_a_drop()
    test_reconnects_after_a_silent_stall()
    test_respawns_a_dead_reader()
    test_counts_frames_lost_on_the_radio()
    test_uneven_frame_steps_are_not_losses()
    test_not_installed_is_said_plainly()
    test_find_lists_the_sensor()
    test_dropout_rejects_the_run()
    print("all passed")


if __name__ == "__main__":
    main()
