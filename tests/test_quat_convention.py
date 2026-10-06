"""
Which way round a sensor publishes its orientation, decided from its own
accelerometer -- for all four conventions met in practice.

The LPMS-B2 through FusionHub publishes world->sensor with z up; the same
sensor through OpenZen publishes sensor->world with z DOWN. With only the
first two candidates the second case chose the less wrong of two wrong
answers and ADDED gravity: 19.75 m/s^2 of "linear acceleration" on a unit
standing still. The numbers below are the ones that real sensor reported.

Run:  python tests/test_quat_convention.py
"""
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sonair_benchmark.attitude import (GRAVITY, QuatConvention as QC,  # noqa: E402
                                       gravity_from_quat, q_conjugate,
                                       q_multiply, q_normalise)


def residual(acc, q):
    g = gravity_from_quat(q)
    n = math.sqrt(sum(a * a for a in acc))
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(
        a / n * b for a, b in zip(acc, g))))))


def test_the_real_lpms_b2_over_openzen():
    raw = [0.0160, 0.3276, 0.9445, -0.0161]     # as the sensor sent it
    acc = [-0.18, 0.41, 9.96]                    # lying still, m/s^2
    c = QC()
    for _ in range(40):
        c.feed(raw, acc)
    assert c.decided == "conjugate_zdown", c.status()
    lin = [a - GRAVITY * g for a, g in zip(acc, gravity_from_quat(c.apply(raw)))]
    assert max(abs(v) for v in lin) < 0.25, lin  # what is left: its 1.5% scale
    print(f"  pass  LPMS-B2 over OpenZen: {c.decided}, still-arm linear "
          f"acceleration {max(abs(v) for v in lin):.2f} m/s^2 (was 19.75)")


def test_every_convention_from_tilted_poses():
    random.seed(1)
    right = undecided = 0
    for _ in range(200):
        ax = [random.gauss(0, 1) for _ in range(3)]
        n = math.sqrt(sum(v * v for v in ax))
        ang = random.uniform(0.3, 2.5)
        qt = q_normalise([math.cos(ang / 2)] + [math.sin(ang / 2) * v / n for v in ax])
        acc = [GRAVITY * v for v in gravity_from_quat(qt)]
        for name in QC.CANDIDATES:
            pub = qt
            if name.endswith("_zdown"):
                pub = q_multiply([0.0, -1.0, 0.0, 0.0], pub)
            if name.startswith("conjugate"):
                pub = q_conjugate(pub)
            c = QC()
            for _ in range(40):
                c.feed(pub, acc)
            if not c.decided:
                undecided += 1
                continue
            assert c.decided == name, (name, c.status())
            assert residual(acc, c.apply(pub)) < 0.01
            right += 1
    assert undecided < 10
    print(f"  pass  {right} of 800 decided correctly, {undecided} left open "
          f"(nearly symmetric poses), none decided wrongly")


def test_remembered_then_checked():
    raw = [0.0160, 0.3276, 0.9445, -0.0161]
    acc = [-0.18, 0.41, 9.96]
    # remembered right: usable at once, then confirmed by the data
    c = QC()
    c.remember("conjugate_zdown", "2026-10-01")
    assert c.decided == "conjugate_zdown" and c.status()["quat_convention_remembered"]
    assert not c.status()["quat_convention_confirmed"]
    for _ in range(40):
        c.feed(raw, acc)
    assert c.decided == "conjugate_zdown" and c.status()["quat_convention_confirmed"]
    assert not c.revised
    # remembered wrong (the FusionHub answer, for the OpenZen stream): the data wins
    c = QC()
    c.remember("conjugate")
    for _ in range(40):
        c.feed(raw, acc)
    assert c.decided == "conjugate_zdown" and c.revised, c.status()
    print("  pass  a remembered convention is used at once and re-checked; "
          "a wrong one is overruled by the data")


def test_agent_remembers_across_restarts():
    import os
    import tempfile
    os.chdir(tempfile.mkdtemp())
    import bench_agent
    raw = [0.0160, 0.3276, 0.9445, -0.0161]
    acc = [-0.18, 0.41, 9.96]
    hub = bench_agent.ImuHub()
    hub.set_source("ind0", "openzen")
    for k in range(40):
        hub.push("ind0", k * 0.01, {"quat": raw, "accel": acc,
                                     "gyro": [0.0, 0.0, 0.0]})
    st = hub.tracker_status()["ind0"]
    assert st["quat_convention"] == "conjugate_zdown", st
    book = bench_agent.json.loads(hub.CONV_PATH.read_text())
    assert book["ind0"]["openzen"]["convention"] == "conjugate_zdown", book
    # the agent restarts: known at once, before the arm has moved
    hub2 = bench_agent.ImuHub()
    hub2.set_source("ind0", "openzen")
    st = hub2.tracker_status()["ind0"]
    assert st["quat_convention"] == "conjugate_zdown" and \
        st["quat_convention_remembered"], st
    # a different connection starts again
    hub2.set_source("ind0", "udp-listen")
    assert hub2.tracker_status()["ind0"]["quat_convention"] == "deciding"
    print("  pass  the agent remembers the convention per sensor and "
          "connection across a restart; a new connection decides afresh")


if __name__ == "__main__":
    test_the_real_lpms_b2_over_openzen()
    test_every_convention_from_tilted_poses()
    test_remembered_then_checked()
    test_agent_remembers_across_restarts()
    print("all passed")
