"""
Does this console report what FusionHub reports?

The operator asked the plainest possible question of this rig: the numbers on
FusionHub's own Overview page and the numbers on the console's Sensors page
disagreed, so were they the same data or not? They were the same data --
sample for sample, to six decimal places -- being INTERPRETED differently in
three places, and all three interpretations were the console's fault:

  1. the gyroscope        FusionHub publishes DEGREES per second; the console
                          wrote the same numbers into columns named rad/s and
                          then multiplied by 57.3 to display "deg/s", so a
                          22.9 deg/s elbow sweep was reported as 1425 deg/s;
  2. the quaternion       published WORLD-TO-SENSOR, consumed as though it
                          were sensor-to-world, which flips the sign of pitch
                          and leaves 1.9 m/s^2 of fictitious linear
                          acceleration on a motionless arm;
  3. the timestamp        FusionHub's `timecode` field is bit-identical on
                          every one of 4280 messages, and it was trusted, so
                          an exported log of 1425 rows carried one instant.

The accelerometer (g -> m/s^2), the magnetometer and the quaternion values
themselves were already exact and must stay that way.

The fixture is a real FusionHub MCAP recording taken on the cell during a
25 deg elbow sweep at 0.4 rad/s, including FusionHub's own Euler angles for
each sample, which is what the console's orientation is checked against.

Run:  python -m pytest tests/test_fusionhub_agreement.py -q
 or:  python tests/test_fusionhub_agreement.py
"""
from __future__ import annotations

import json
import math
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import imu_link                                     # noqa: E402
from sonair_benchmark.attitude import (              # noqa: E402
    AttitudeTracker, GRAVITY, q_to_euler_deg,
)

FIXTURE = Path(__file__).parent / "fixtures" / "fusionhub_lpms_b2.json"

# The elbow was commanded 25 deg at 0.4 rad/s. 0.4 rad/s is 22.9 deg/s, and
# the carrier is rigid with the link, so nothing in this recording may report
# an angular rate anywhere near a radian-per-second reading of the raw numbers.
COMMANDED_RATE_DEG_S = math.degrees(0.4)


def load():
    d = json.loads(FIXTURE.read_text())
    return d["rows"]


def replay(rows, pin_units="auto"):
    """
    Push the fixture through the real ingestion path: the link's unit
    decision, then the attitude tracker, exactly as a live packet goes.
    """
    units = imu_link.GyroUnits(pin_units)
    tracker = AttitudeTracker("ind0")
    out = []
    for t_arr, timecode, quat, gyro, accel_g, mag, fh_euler in rows:
        rec = {
            "quat": list(quat),
            "gyro": list(gyro),
            "accel": [v * GRAVITY for v in accel_g],
            "mag": list(mag),
        }
        # The source timestamp is handed over exactly as FusionHub sends it:
        # the same frozen integer every time.
        rec = units.feed(rec, timecode / 1e9, t_arr)
        derived = tracker.update(t_arr, rec)
        out.append({"t": t_arr, "fh_euler": fh_euler, **rec, **derived})
    return units, tracker, out


# ---------------------------------------------------------------------------
# 1. the raw channels that were already right must stay right
# ---------------------------------------------------------------------------

def test_accelerometer_is_g_converted_to_m_s2():
    rows = load()
    units, tracker, out = replay(rows)
    for r, row in zip(out, rows):
        for i in range(3):
            assert abs(r["accel"][i] - row[4][i] * 9.80665) < 1e-9
    norms = [r["accel_norm"] for r in out if "accel_norm" in r]
    # A near-stationary carrier reads one g. This is the check that would have
    # caught a wrong gravity constant or a double conversion.
    assert 9.5 < st.median(norms) < 10.2, st.median(norms)


def test_quaternion_and_magnetometer_pass_through_untouched():
    rows = load()
    _, _, out = replay(rows)
    for r, row in zip(out, rows):
        assert r["mag"] == row[5]
        # The stored quaternion may be re-expressed in this module's sense, so
        # compare the rotation it represents, not the four numbers.
        a = r["quat"]
        b = row[2]
        dot = abs(sum(x * y for x, y in zip(a, b)))
        conj = abs(a[0] * b[0] - sum(x * y for x, y in zip(a[1:], b[1:])))
        assert max(dot, conj) > 0.999999, (a, b)


# ---------------------------------------------------------------------------
# 2. the gyroscope's units
# ---------------------------------------------------------------------------

def test_gyro_units_are_found_to_be_degrees():
    units, _, out = replay(load())
    assert units.decided == "deg", (units.decided, units.basis, units.ratios)
    assert units.n_revised == 0, ("the verdict should be reached from evidence, "
                                  "not corrected after the fact: " + units.basis)


def test_reported_angular_rate_matches_the_commanded_elbow_sweep():
    _, _, out = replay(load())
    rates = [r["gyro_norm_deg_s"] for r in out if "gyro_norm_deg_s" in r]
    assert rates, "no angular rate was reported at all"
    peak = max(rates)
    # Before the fix this peak was 1425 deg/s -- a UR5e elbow doing four
    # revolutions a second. It must now sit just above the commanded rate.
    assert COMMANDED_RATE_DEG_S <= peak < 3 * COMMANDED_RATE_DEG_S, peak


def test_reported_angular_rate_matches_the_unit_s_own_orientation_change():
    """
    The absolute check, and the one that pins the units rather than bounding
    them: how fast the unit says it is turning must match how fast its own
    orientation is actually turning. The quaternion is an independent witness
    -- it comes out of the sensor's own fusion and is not scaled by whatever
    the gyroscope reports in -- so if the two agree, the units are right.
    A factor of 57 is not something this test can miss.
    """
    _, _, out = replay(load())
    have = [r for r in out if "gyro_norm_deg_s" in r and r.get("quat")]
    assert len(have) > 200, len(have)
    # Compared over a window, not between neighbours. This link arrives in
    # Bluetooth bursts -- several packets in the same millisecond, then a gap
    # -- so a neighbour-to-neighbour dt is mostly transport noise.
    ratios = []
    j = 0
    for i, a in enumerate(have):
        while j < len(have) and have[j]["t"] - a["t"] < 0.15:
            j += 1
        if j >= len(have):
            break
        b = have[j]
        dt = b["t"] - a["t"]
        d = abs(sum(x * y for x, y in zip(a["quat"], b["quat"])))
        turn = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d)))) / dt
        if turn < 5.0:
            continue            # too slow to be clear of the noise
        span = [r["gyro_norm_deg_s"] for r in have[i:j + 1]]
        ratios.append((sum(span) / len(span)) / turn)
    assert len(ratios) > 20, len(ratios)
    assert 0.8 < st.median(ratios) < 1.25, st.median(ratios)


def test_rows_taken_before_the_verdict_carry_no_rad_s_value():
    """
    The pre-verdict rows are the subtle half of this bug: a number whose unit
    is not yet known must not be written into a column that asserts one.
    """
    _, _, out = replay(load())
    pending = [r for r in out if r.get("_units_pending")]
    assert pending, "expected some rows before the units were established"
    for r in pending:
        assert "gyro" not in r
        assert r["gyro_raw"]


# ---------------------------------------------------------------------------
# 3. the quaternion's sense
# ---------------------------------------------------------------------------

def test_quaternion_sense_is_found_to_be_world_to_sensor():
    _, tracker, _ = replay(load())
    assert tracker.convention.decided == "conjugate", \
        tracker.convention.status()
    assert tracker.convention.residual_deg < 1.0, tracker.convention.status()


def test_euler_angles_agree_with_fusionhub():
    _, _, out = replay(load())
    errs = []
    undecided = 0
    for r in out:
        if r.get("quat_source") != "device":
            continue
        if r.get("quat_convention") != "conjugate":
            # Still measuring which way round the unit publishes. These rows
            # are passed through unchanged and SAY SO, which is the intended
            # behaviour -- but there must only be a handful of them.
            undecided += 1
            continue
        got = r["euler_deg"]
        want = r["fh_euler"]
        errs.append(max(abs(_wrap(g - w)) for g, w in zip(got, want)))
    assert errs
    # Same convention, same data: the only difference left is the rounding in
    # the fixture. Before the fix pitch was out by 11 degrees on every row.
    assert max(errs) < 0.05, (max(errs), st.median(errs))
    # The undecided window is the first fraction of a second, no more.
    assert undecided <= 40, undecided


def test_no_fictitious_linear_acceleration_on_a_still_carrier():
    """
    `linear_accel` is accelerometer minus gravity, and it is a channel the
    benchmark is scored on. Getting the quaternion's sense wrong pointed
    gravity 11 degrees off, which left 9.81*sin(11) = 1.9 m/s^2 behind on a
    carrier that was standing perfectly still.
    """
    _, _, out = replay(load())
    still = [r["linear_accel_norm"] for r in out
             if "linear_accel_norm" in r and _is_still(r)
             and r.get("quat_convention") == "conjugate"]
    assert len(still) > 100, len(still)
    # 0.35 m/s^2 is the noise of the part itself. 1.9 m/s^2 was the bug.
    assert st.median(still) < 0.35, st.median(still)


# ---------------------------------------------------------------------------
# 4. the clock
# ---------------------------------------------------------------------------

def test_fusionhub_timecode_really_is_frozen():
    """Guards the premise of the clock fix, so the fixture cannot drift."""
    rows = load()
    assert len({r[1] for r in rows}) == 1, "the fixture no longer shows a frozen clock"


def test_master_clock_refuses_the_frozen_timestamp():
    import bench_agent
    master = bench_agent.TimeMaster()
    rows = load()
    ts = [master.to_master("ind0", r[1] / 1e9) for r in rows]
    assert len(set(ts)) > 0.9 * len(ts), \
        "master time must advance even when the sensor's own clock does not"
    assert ts == sorted(ts), "the master timeline must be monotonic"
    assert max(ts) < 1e6, "an epoch-scale value leaked through as master time"
    block = master.clock_report()["ind0"]
    assert block["clock"] == "stalled", block
    assert block["timebase"] == "arrival", block


def test_master_clock_resolves_better_than_a_sample_interval():
    """
    The clock every recorded sample is stamped with has to resolve finer than
    the interval between samples. On Windows `time.monotonic()` does not: it
    is the 15.6 ms scheduler tick, while the inertial unit on this rig arrives
    every 5.3 ms and the recorder is asked for a sample every 8 ms. A
    smoke-test run stamped with it held 303 samples at only 195 distinct
    times, reported intervals of exactly zero, and delivered 43 Hz of the
    125 Hz it declared. This test fails on that clock and passes on
    `perf_counter`, so it fails on the platform the console actually runs on
    if the change is ever reverted.
    """
    import bench_agent
    master = bench_agent.TimeMaster()
    seen = set()
    t_end = master.now() + 0.05
    while master.now() < t_end:
        seen.add(master.now())
    steps = sorted(seen)
    gaps = [b - a for a, b in zip(steps, steps[1:]) if b > a]
    assert gaps, "the master clock did not advance at all"
    # A 125 Hz grid needs a clock far finer than 8 ms; 1 ms is a generous bar
    # that the 15.6 ms Windows tick still fails.
    assert min(gaps) < 1e-3, f"coarsest usable step {min(gaps) * 1e3:.2f} ms"


def test_attitude_filters_still_run_with_a_frozen_source_clock():
    """
    dt came from the source clock, so a frozen clock meant every integration
    step was exactly zero and the cross-check between the two filters could
    never say anything. Timed by arrival, both filters run and agree.
    """
    _, tracker, out = replay(load())
    assert tracker.seeded
    dis = [r["filter_disagreement_deg"] for r in out
           if "filter_disagreement_deg" in r]
    assert dis and max(dis) > 0.0, \
        "the two orientation filters produced identical output, which means " \
        "neither of them integrated anything"


def _is_still(r):
    """Stationary judged from the accelerometer, which needs no gyro units."""
    n = r.get("accel_norm")
    return n is not None and abs(n / GRAVITY - 1.0) < 0.02


def _wrap(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


if __name__ == "__main__":
    import traceback
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print(f"  pass  {name}")
        except Exception:
            fails += 1
            print(f"  FAIL  {name}")
            traceback.print_exc()
    print(("all passed" if not fails else f"{fails} failed"))
    sys.exit(1 if fails else 0)
