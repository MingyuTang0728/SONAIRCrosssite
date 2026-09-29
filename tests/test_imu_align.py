"""
The IMU time offset and mounting estimate, against IMUs whose answer is known.

A robot log is synthesised from the calibration motion itself (the same joint
targets the job drives), at the controller's 125 Hz. An IMU log is made from
it by rotating the flange's true angular velocity and gravity into a known
mounting, stamping it a known time late, and adding the noise, bias, jitter
and dropouts of the real LPMS-B2 over Bluetooth. The estimate must find both.

Then the ways it must refuse rather than guess: a log with too little motion,
a robot log full of holes, a gyro in the wrong units, a base that is not
upright.

Run:  python tests/test_imu_align.py
"""
from __future__ import annotations

import csv
import math
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import imu_align as ia                                # noqa: E402
import ur_kin                                         # noqa: E402

ROOT = Path(tempfile.mkdtemp())
Q0 = [0.3, -1.4, 1.5, -1.7, -1.57, 0.2]
RNG = np.random.default_rng(7)


def trapezoid(q_from, q_to, v, a, dt):
    d = np.asarray(q_to) - np.asarray(q_from)
    dist = float(np.abs(d).max())
    if dist < 1e-12:
        return [], []
    if dist > v * v / a:
        tr, tc = v / a, (dist - v * v / a) / v
    else:
        tr, tc = math.sqrt(dist / a), 0.0
    vp = a * tr
    T = 2 * tr + tc
    qs, qds = [], []
    for t in np.arange(dt, T + dt, dt):
        t = min(t, T)
        if t < tr:
            s, sv = 0.5 * a * t * t, a * t
        elif t < tr + tc:
            s, sv = 0.5 * vp * tr + vp * (t - tr), vp
        else:
            td = T - t
            s, sv = dist - 0.5 * a * td * td, a * td
        qs.append(np.asarray(q_from) + d * (s / dist))
        qds.append(d / dist * sv)
    return qs, qds


def motion(targets, v=ia.EXCITE_SPEED, pause=ia.EXCITE_PAUSE_S, lead=2.0,
           dt=0.008):
    q = np.asarray(Q0, float)
    Q, QD = [], []
    for _ in range(int(lead / dt)):
        Q.append(q.copy())
        QD.append(np.zeros(6))
    for tgt in targets:
        qs, qds = trapezoid(q, tgt, v, ia.EXCITE_ACCEL, dt)
        Q += qs
        QD += qds
        q = np.asarray(tgt, float)
        for _ in range(int(pause / dt)):
            Q.append(q.copy())
            QD.append(np.zeros(6))
    t = 100.0 + dt * np.arange(len(Q))
    return t, np.array(Q), np.array(QD)


def write_ur(path, t, Q, QD, holes=()):
    cols = ["t_s"] + [f"actual_q_{j}" for j in ia.JOINTS] + \
           [f"actual_qd_{j}" for j in ia.JOINTS]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for ti, q, qd in zip(t, Q, QD):
            if any(a <= ti < b for a, b in holes):
                continue
            # the controller's packets arrive with a millisecond of jitter
            w.writerow([f"{ti + RNG.normal(0, 0.0008):.6f}", *q, *qd])


def write_imu(path, t, Q, QD, R, lag, rate=300.0, scale=1.0,
              up=np.array([0.0, 0.0, 1.0])):
    """The IMU a flange with mounting R reads, stamped `lag` late."""
    w_f = ia.flange_omega(Q, QD)
    ti = np.arange(t[0], t[-1], 1.0 / rate)
    ti = ti + RNG.normal(0, 0.0004, len(ti))
    # Bluetooth: bursts where a few samples arrive together, and dropouts
    keep = RNG.random(len(ti)) > 0.02
    ti = ti[keep]
    bias = np.array([0.004, -0.003, 0.002])
    rows = []
    w_at = np.column_stack([np.interp(ti, t, w_f[:, c]) for c in range(3)])
    for tk, wk in zip(ti, w_at):
        k = min(len(t) - 1, max(0, int(round((tk - t[0]) / (t[1] - t[0])))))
        g = R.T @ wk * scale + bias + RNG.normal(0, 0.004, 3)
        Rbf = ur_kin.fk(Q[k])[:3, :3]
        acc = R.T @ (Rbf.T @ (up * ia.G)) + RNG.normal(0, 0.02, 3)
        rows.append((tk + lag, g, acc))
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "unit"] + [f"gyro_{a}_rad_s" for a in "xyz"] +
                   [f"accel_{a}_m_s2" for a in "xyz"])
        for tk, g, a in rows:
            w.writerow([f"{tk:.6f}", "ind0", *g, *a])


def case(name, targets, R, lag, **kw):
    holes = kw.pop("holes", ())
    t, Q, QD = motion(targets)
    ur, imu = ROOT / f"{name}_ur.csv", ROOT / f"{name}_imu.csv"
    write_ur(ur, t, Q, QD, holes)
    write_imu(imu, t, Q, QD, R, lag, **kw)
    return ia.calibrate(ur, imu)


def angle_deg(Ra, Rb):
    return math.degrees(np.linalg.norm(ur_kin.rotvec(np.asarray(Ra).T @ Rb)))


def main():
    targets = ia.excitation_targets(Q0)
    secs = ia.excitation_seconds()

    # 1. the calibration motion, three different mountings and offsets
    for i, (rv, lag) in enumerate((([0.0, math.pi, 0.0], 0.137),
                                   ([1.2, -0.4, 2.1], 0.043),
                                   ([0.0, 0.0, math.pi / 2], -0.021))):
        R = ur_kin.rotmat(rv)
        cal = case(f"good{i}", targets, R, lag)
        assert cal["ok"], ia.summary(cal)
        err_ms = abs(cal["lag_s"] - lag) * 1000
        err_deg = angle_deg(cal["R_flange_imu"], R)
        assert err_ms < 2.0, (err_ms, cal["lag"])
        assert err_deg < 1.0, (err_deg, cal["mount"])
        print(f"  pass  mounting {np.degrees(np.linalg.norm(rv)):.0f} deg, "
              f"offset {lag * 1000:+.0f} ms: found within {err_ms:.2f} ms and "
              f"{err_deg:.2f} deg ({secs:.0f} s of motion)")

    # 2. an elbow-only campaign run with its still dwells is enough, because
    #    gravity pins what one axis of rotation leaves free
    R = ur_kin.rotmat([0.5, 0.9, -0.3])
    elbow = [t for t in targets[:3]] * 3
    cal = case("elbow", elbow, R, 0.09)
    assert cal["ok"], ia.summary(cal)
    assert angle_deg(cal["R_flange_imu"], R) < 1.5, cal["mount"]
    print(f"  pass  elbow-only motion with stops: still determined "
          f"(conditioning {cal['mount']['conditioning']})")

    # 3. elbow only and never still: one axis, no gravity -> refused
    t, Q, QD = motion(elbow, pause=0.0, lead=0.0)
    write_ur(ROOT / "e1_ur.csv", t, Q, QD)
    write_imu(ROOT / "e1_imu.csv", t, Q, QD, R, 0.09)
    ur = ia.load_ur(ROOT / "e1_ur.csv")
    imu = ia.load_imu(ROOT / "e1_imu.csv")
    m = ia.estimate_mount(ur, imu, 0.09)
    assert not m["ok"] and "undetermined" in m["error"], m
    print("  pass  one-axis motion without stops is refused, not guessed")

    # 4. a robot log full of holes is refused and says why
    cal = case("holes", targets, R, 0.1,
               holes=[(104 + 6 * k, 109 + 6 * k) for k in range(6)])
    assert not cal["ok"] and "missing" in cal["error"], cal["error"]
    print("  pass  a robot log with holes is refused: " + cal["error"][:60] + "...")

    # 5. a gyro reporting deg/s where rad/s was expected
    cal = case("units", targets, R, 0.05, scale=math.degrees(1.0))
    assert not cal["ok"], ia.summary(cal)
    print("  pass  a gyro in the wrong units is caught: "
          + cal["error"][:70] + "...")

    # 6. a base that is not upright (wall-mounted): gravity will not agree
    cal = case("wall", targets, R, 0.05, up=np.array([1.0, 0.0, 0.0]))
    assert not cal["ok"] and "upright" in cal["error"], ia.summary(cal)
    print("  pass  a base that is not upright is caught by the gravity check")

    # 7. no motion at all
    cal = case("still", [], R, 0.05)
    assert not cal["ok"]
    print("  pass  a log with no motion is refused")

    # 8. the file round-trip, and the vector convention
    cal = case("good0b", targets, ur_kin.rotmat([0.0, math.pi, 0.0]), 0.137)
    p = ia.save(cal, ROOT / "calib" / "imu_cal.json")
    back = ia.load(p)
    v = ia.flange_to_imu(back, [0.0, 0.0, 1.0])
    assert abs(v[2] + 1.0) < 0.02, v
    print("  pass  the saved calibration loads and maps flange axes to IMU axes")

    shutil.rmtree(ROOT, ignore_errors=True)
    print("all passed")


if __name__ == "__main__":
    main()
