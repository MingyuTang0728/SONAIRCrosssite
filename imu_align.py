"""
imu_align.py -- when the IMU's samples happened, and which way it is bolted on.

Two numbers stand between an IMU log and a comparison with the simulator, and
neither can be read off a datasheet:

  THE TIME OFFSET. The robot log and the IMU log are both stamped on the host's
  clock, but each stamp is the moment a packet ARRIVED, and the two packets
  took different roads: the controller's RTDE stream on one side, the sensor's
  Bluetooth link and FusionHub on the other. The difference is tens to
  hundreds of milliseconds and it is not zero. At 0.9 rad/s, 100 ms is 5 deg
  of wrist rotation -- a "gap" the simulator would be blamed for.

  THE MOUNTING ROTATION. The IMU reports in its own axes. The simulator's IMU
  sits at the flange and reports in the flange's axes. Until the rotation
  between them is known, the two gyro traces cannot be compared axis by axis,
  and a bracket rotated 90 deg looks like a sensor that measures the wrong
  thing.

Both come from data the cell already records: the robot log's joint
positions and velocities and the IMU log's gyro and accelerometer.

  Time offset: the flange's angular velocity follows from the joint
  velocities through the arm's kinematics (ur_kin). Its MAGNITUDE does not
  depend on how the IMU is mounted, so it can be cross-correlated with the
  gyro's magnitude before the mounting is known. That is done in short
  windows, and their agreement is reported with the offset itself: a single
  number from one correlation looks the same whether it is solid or noise.

  Mounting rotation: with the offset removed, each moving sample gives a pair
  (angular velocity in flange axes, gyro in IMU axes), and each still sample
  gives another (gravity in flange axes, accelerometer in IMU axes). The
  rotation that best maps one onto the other is a least-squares problem with
  a closed-form answer (Kabsch). Gravity is what makes a single-joint motion
  sufficient: an elbow sweep turns the flange about one axis only, which pins
  one axis of the rotation and leaves the other two free. If the data still
  cannot pin the rotation, the estimate is refused, not guessed.

Assumption, stated because it matters: the robot base is mounted upright
(base z axis pointing up). The gravity residual reports whether that held.

Run:
    python imu_align.py --ur ur_logs/ur_X.csv --imu imu_logs/imu_X.csv
    python imu_align.py ... --save calib/imu_cal.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import ur_kin

JOINTS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
G = 9.80665
CAL_PATH = Path("calib") / "imu_cal.json"

GRID_S = 0.005          # common time grid for the correlation
GAP_S = 0.10            # a stream silent longer than this is a hole, not data
MAX_LAG_S = 0.50        # search range for the time offset, either direction
WINDOW_S = 6.0
STEP_S = 2.0
MIN_WINDOW_CORR = 0.6
MOVING_RAD_S = 0.10     # both streams must agree the flange is turning
STILL_RAD_S = 0.01      # and that it is not
MIN_S2_RATIO = 0.05     # below this the rotation is not determined

# The calibration motion: each of these joints out by this much, back past
# the start by the same, and home, stopping after every move. Elbow, both
# wrist bends and the wrist roll, so the flange turns about three independent
# axes and comes to rest at nine different attitudes.
EXCITATION = ((2, 20.0), (3, 20.0), (4, 25.0), (5, 40.0))
EXCITE_SPEED = 0.5      # rad/s
EXCITE_ACCEL = 1.2      # rad/s^2
EXCITE_PAUSE_S = 1.5


def excitation_targets(q0) -> list:
    """The joint targets of the calibration motion, from where the arm is."""
    out = []
    for j, deg in EXCITATION:
        for k in (1, -1, 0):
            q = [float(v) for v in q0]
            q[j] += k * math.radians(deg)
            out.append(q)
    return out


def excitation_seconds(v=EXCITE_SPEED, pause=EXCITE_PAUSE_S,
                       a=EXCITE_ACCEL) -> float:
    total = 0.0
    for _, deg in EXCITATION:
        for dist in (deg, 2 * deg, deg):
            d = math.radians(dist)
            total += (2 * v / a + max(0.0, d - v * v / a) / v
                      if d > v * v / a else 2 * math.sqrt(d / a)) + pause
    return total


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def _f(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return math.nan
    return x


def load_ur(path) -> dict:
    """t, q (N,6), qd (N,6) from a robot log written by the console."""
    t, q, qd = [], [], []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            qs = [_f(row.get(f"actual_q_{j}")) for j in JOINTS]
            qds = [_f(row.get(f"actual_qd_{j}")) for j in JOINTS]
            ts = _f(row.get("t_s"))
            if math.isnan(ts) or any(math.isnan(v) for v in qs + qds):
                continue
            t.append(ts)
            q.append(qs)
            qd.append(qds)
    if len(t) < 10:
        raise ValueError(f"{path}: fewer than 10 usable robot rows")
    t = np.asarray(t)
    order = np.argsort(t, kind="stable")
    return {"t": t[order], "q": np.asarray(q)[order], "qd": np.asarray(qd)[order]}


def load_imu(path, unit: str | None = None) -> dict:
    """t, gyro (N,3) rad/s, accel (N,3) m/s^2 from an IMU log, one unit."""
    rows = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            u = row.get("unit") or ""
            g = [_f(row.get(f"gyro_{a}_rad_s")) for a in "xyz"]
            a = [_f(row.get(f"accel_{a}_m_s2")) for a in "xyz"]
            ts = _f(row.get("t_s"))
            if math.isnan(ts) or any(math.isnan(v) for v in g + a):
                continue
            rows.setdefault(u, []).append((ts, g, a))
    if not rows:
        raise ValueError(f"{path}: no usable IMU rows")
    if unit is None:
        unit = max(rows, key=lambda k: len(rows[k]))
    if unit not in rows:
        raise ValueError(f"{path}: no rows for unit {unit!r} "
                         f"(present: {', '.join(sorted(rows))})")
    r = sorted(rows[unit], key=lambda x: x[0])
    return {"unit": unit, "t": np.array([x[0] for x in r]),
            "gyro": np.array([x[1] for x in r]),
            "accel": np.array([x[2] for x in r])}


def flange_omega(q, qd) -> np.ndarray:
    """Flange angular velocity in flange axes for every robot row."""
    return np.array([ur_kin.body_angular_velocity(a, b) for a, b in zip(q, qd)])


def flange_rotation(q) -> np.ndarray:
    return np.array([ur_kin.fk(a)[:3, :3] for a in q])


# ---------------------------------------------------------------------------
# resampling with holes
# ---------------------------------------------------------------------------

def _on_grid(t_src, values, grid, gap_s=GAP_S):
    """
    Linear interpolation onto `grid`, and a mask that is False wherever the
    nearest source samples are further apart than `gap_s`: interpolating
    across a 13 s hole in the robot log would invent 13 s of motion.
    """
    values = np.asarray(values, dtype=float)
    idx = np.searchsorted(t_src, grid)
    ok = (idx > 0) & (idx < len(t_src))
    lo = np.clip(idx - 1, 0, len(t_src) - 1)
    hi = np.clip(idx, 0, len(t_src) - 1)
    ok &= (t_src[hi] - t_src[lo]) <= gap_s
    if values.ndim == 1:
        out = np.interp(grid, t_src, values)
    else:
        out = np.column_stack([np.interp(grid, t_src, values[:, k])
                               for k in range(values.shape[1])])
    return out, ok


# ---------------------------------------------------------------------------
# time offset
# ---------------------------------------------------------------------------

def _xcorr(a, am, b, bm, n_lag):
    """
    Pearson correlation of a[i] with b[i + k] for k in [-n_lag, n_lag], over
    samples valid in both. b[i + k] with k > 0 means b happens later.
    """
    n = len(a)
    out = np.full(2 * n_lag + 1, np.nan)
    for j, k in enumerate(range(-n_lag, n_lag + 1)):
        if k >= 0:
            x, xm, y, ym = a[:n - k], am[:n - k], b[k:], bm[k:]
        else:
            x, xm, y, ym = a[-k:], am[-k:], b[:n + k], bm[:n + k]
        m = xm & ym
        if m.sum() < 50:
            continue
        xs, ys = x[m], y[m]
        xs = xs - xs.mean()
        ys = ys - ys.mean()
        d = math.sqrt(float((xs * xs).sum() * (ys * ys).sum()))
        if d > 0:
            out[j] = float((xs * ys).sum()) / d
    return out


def _peak(c, dt, n_lag):
    """Best lag with a parabolic refinement between grid points."""
    if np.all(np.isnan(c)):
        return None, None
    j = int(np.nanargmax(c))
    lag = (j - n_lag) * dt
    if 0 < j < len(c) - 1 and not (np.isnan(c[j - 1]) or np.isnan(c[j + 1])):
        den = c[j - 1] - 2 * c[j] + c[j + 1]
        if den < 0:
            lag += 0.5 * (c[j - 1] - c[j + 1]) / den * dt
    return lag, float(c[j])


def estimate_lag(ur: dict, imu: dict, w_flange=None,
                 max_lag_s=MAX_LAG_S, window_s=WINDOW_S, step_s=STEP_S) -> dict:
    """
    How much later the IMU's stamps are than the robot's for the same motion.

    Positive: the IMU is late. A sample stamped t in the IMU log happened at
    t - lag on the robot log's clock.
    """
    if w_flange is None:
        w_flange = flange_omega(ur["q"], ur["qd"])
    t0 = max(ur["t"][0], imu["t"][0])
    t1 = min(ur["t"][-1], imu["t"][-1])
    if t1 - t0 < window_s:
        return {"ok": False, "error": (
            f"the two logs overlap for {max(0.0, t1 - t0):.1f} s; at least "
            f"{window_s:.0f} s of both is needed")}
    grid = np.arange(t0, t1, GRID_S)
    r, rm = _on_grid(ur["t"], np.linalg.norm(w_flange, axis=1), grid)
    g, gm = _on_grid(imu["t"], np.linalg.norm(imu["gyro"], axis=1), grid)
    n_lag = int(round(max_lag_s / GRID_S))

    # the whole record, once
    lag_all, corr_all = _peak(_xcorr(r, rm, g, gm, n_lag), GRID_S, n_lag)

    # and in windows, to see whether the answer holds up
    wins = []
    nw, ns = int(window_s / GRID_S), int(step_s / GRID_S)
    for s in range(0, len(grid) - nw + 1, ns):
        sl = slice(s, s + nw)
        if rm[sl].mean() < 0.9 or gm[sl].mean() < 0.9:
            continue
        if np.nanmax(r[sl][rm[sl]]) < MOVING_RAD_S or np.std(r[sl][rm[sl]]) < 0.02:
            continue                    # nothing moved: nothing to line up
        lag, c = _peak(_xcorr(r[sl], rm[sl], g[sl], gm[sl], n_lag), GRID_S, n_lag)
        if lag is None or c < MIN_WINDOW_CORR:
            continue
        if abs(lag) >= max_lag_s - 2 * GRID_S:
            continue                    # pinned at the edge of the search
        wins.append((float(grid[s] + window_s / 2), lag, c))

    res = {"ok": False, "lag_s": None, "whole_record_lag_s": lag_all,
           "whole_record_corr": corr_all, "windows_used": len(wins),
           "overlap_s": round(float(t1 - t0), 1),
           "robot_coverage": round(float(rm.mean()), 3)}
    if len(wins) < 3:
        if rm.mean() < 0.5:
            res["error"] = (
                f"the robot log is missing {100 * (1 - rm.mean()):.0f}% of the "
                f"time both logs were running -- the robot link was not "
                f"delivering when it was recorded. Record again now the link "
                f"is healthy")
        else:
            res["error"] = (
                f"only {len(wins)} stretch(es) of motion could be lined up "
                f"(3 are needed). Record at least 30 s with the arm moving "
                f"through several starts and stops, with both logs running")
        return res
    lags = np.array([w[1] for w in wins])
    med = float(np.median(lags))
    mad = float(np.median(np.abs(lags - med)))
    res.update({
        "ok": True, "lag_s": med,
        "spread_s": 1.4826 * mad,                   # robust standard deviation
        "window_lags_s": [round(float(x), 4) for x in lags],
        "window_corr": [round(float(w[2]), 3) for w in wins],
    })
    # a lag that moves through the record is a clock drifting, not an offset
    if len(wins) >= 5:
        tt = np.array([w[0] for w in wins])
        if np.ptp(tt) > 0:
            slope = float(np.polyfit(tt - tt.mean(), lags, 1)[0])
            res["drift_ms_per_min"] = round(slope * 60_000.0, 2)
    if res["spread_s"] > 0.02:
        res["warning"] = (
            f"the windows disagree by about {res['spread_s'] * 1000:.0f} ms; "
            f"the offset is not stable through this record")
    return res


# ---------------------------------------------------------------------------
# mounting rotation
# ---------------------------------------------------------------------------

def _kabsch(a, b, w):
    """R minimising sum w |a - R b|^2. Returns R and the singular values."""
    H = (b * w[:, None]).T @ a
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, S


def estimate_mount(ur: dict, imu: dict, lag_s: float, w_flange=None,
                   use_gravity: bool = True) -> dict:
    """
    R_flange_imu: rotates a vector in IMU axes into flange axes.

    omega_flange = R @ gyro_imu    and, at rest,    up_flange = R @ accel_imu/|.|
    """
    if w_flange is None:
        w_flange = flange_omega(ur["q"], ur["qd"])
    t_imu = imu["t"] - lag_s                    # onto the robot's clock
    t0 = max(ur["t"][0], t_imu[0])
    t1 = min(ur["t"][-1], t_imu[-1])
    grid = np.arange(t0, t1, 0.01)
    wr, rm = _on_grid(ur["t"], w_flange, grid)
    qd_max, _ = _on_grid(ur["t"], np.abs(ur["qd"]).max(axis=1), grid)
    gy, gm = _on_grid(t_imu, imu["gyro"], grid)
    ac, _ = _on_grid(t_imu, imu["accel"], grid)
    ok = rm & gm

    # still: the controller says no joint moves, and so does the gyro
    nr = np.linalg.norm(wr, axis=1)
    ng = np.linalg.norm(gy, axis=1)
    still = ok & (qd_max < 1e-3) & (ng < STILL_RAD_S * 3)
    # a gyro reads its bias when still: remove it before pairing the motion
    bias = np.median(gy[still], axis=0) if still.sum() >= 20 else np.zeros(3)
    gyc = gy - bias
    moving = ok & (nr > MOVING_RAD_S) & (np.linalg.norm(gyc, axis=1) > MOVING_RAD_S)

    a_list, b_list, w_list, kinds = [], [], [], []
    if moving.sum() >= 20:
        a = wr[moving]
        b = gyc[moving]
        a_list.append(a)
        b_list.append(b)
        w_list.append(np.full(len(a), 1.0 / len(a)))
        kinds.append("gyro")
    n_grav = 0
    up_f = None
    if use_gravity and still.sum() >= 20:
        # gravity in flange axes, from the pose; the accelerometer at rest
        # reads the reaction to gravity, which points UP
        qs, _ = _on_grid(ur["t"], ur["q"], grid[still])
        Rbf = flange_rotation(qs)
        up_f = np.einsum("nji,j->ni", Rbf, np.array([0.0, 0.0, 1.0]))
        acc = ac[still]
        na = np.linalg.norm(acc, axis=1)
        keep = np.abs(na - G) < 0.5             # not mid-bump
        if keep.sum() >= 20:
            up_f, acc, na = up_f[keep], acc[keep], na[keep]
            b = acc / na[:, None]
            # a moving-sample pair has magnitude ~0.1-1 rad/s; bring the
            # gravity pairs to a comparable scale so neither drowns the other
            scale = float(np.median(np.linalg.norm(a_list[0], axis=1))) \
                if a_list else 1.0
            a_list.append(up_f * scale)
            b_list.append(b * scale)
            w_list.append(np.full(len(b), 1.0 / len(b)))
            kinds.append("gravity")
            n_grav = int(keep.sum())
            acc_unit = b
    if not a_list:
        return {"ok": False, "error": (
            "neither enough motion nor enough standing still was recorded to "
            "estimate the mounting")}

    A, B, W = np.vstack(a_list), np.vstack(b_list), np.concatenate(w_list)
    R, S = _kabsch(A, B, W)
    ratio = float(S[1] / S[0]) if S[0] > 0 else 0.0
    res = {"n_moving": int(moving.sum()), "n_still": n_grav,
           "used": kinds, "gyro_bias_rad_s": [round(float(x), 5) for x in bias],
           "conditioning": round(ratio, 3),
           "R_flange_imu": [[round(float(x), 6) for x in row] for row in R],
           "rotvec_deg": [round(float(x), 2)
                          for x in np.degrees(ur_kin.rotvec(R))]}

    # how well does it explain the data
    if "gyro" in kinds:
        a, b = wr[moving], gyc[moving]
        err = a - b @ R.T
        res["gyro_rms_rad_s"] = round(float(np.sqrt((err ** 2).sum(1).mean())), 4)
        res["gyro_rel_err"] = round(float(
            np.sqrt((err ** 2).sum(1).mean() / (a ** 2).sum(1).mean())), 3)
        # the scale of one against the other: a gyro in deg/s read as rad/s,
        # or a kinematic model that is not this robot, both show here
        res["gyro_scale"] = round(float(
            np.linalg.norm(b, axis=1).mean() / np.linalg.norm(a, axis=1).mean()), 3)
    if "gravity" in kinds:
        pred = acc_unit @ R.T
        ang = np.degrees(np.arccos(np.clip((pred * up_f).sum(1), -1, 1)))
        res["gravity_median_deg"] = round(float(np.median(ang)), 2)
        res["gravity_max_deg"] = round(float(np.max(ang)), 2)

    problems = []
    if ratio < MIN_S2_RATIO:
        problems.append(
            "the recorded motion turned the flange about a single axis and "
            "gravity did not pin the rest; the rotation about that axis is "
            "undetermined. Run the mounting-calibration motion, which turns "
            "the wrist about three axes and stops between each")
    if 45 < res.get("gyro_scale", 1.0) < 70:
        problems.append(
            f"the gyro reads {res['gyro_scale']:.0f} times the robot's angular "
            f"velocity -- it is reporting degrees per second where radians "
            f"per second were expected")
    elif res.get("gyro_rel_err", 0) > 0.25:
        problems.append(
            f"the rotated gyro misses the robot's angular velocity by "
            f"{res['gyro_rel_err'] * 100:.0f}% -- the time offset, the sensor "
            f"units, or the log pairing is wrong")
    if res.get("gravity_median_deg", 0) > 5:
        problems.append(
            f"gravity disagrees by {res['gravity_median_deg']:.1f} deg after "
            f"rotation -- is the robot base mounted level and upright?")
    res["ok"] = not problems
    if problems:
        res["error"] = "; ".join(problems)
    return res


# ---------------------------------------------------------------------------
# the whole calibration, and its file
# ---------------------------------------------------------------------------

def calibrate(ur_csv, imu_csv, unit: str | None = None) -> dict:
    ur = load_ur(ur_csv)
    imu = load_imu(imu_csv, unit)
    w = flange_omega(ur["q"], ur["qd"])
    lag = estimate_lag(ur, imu, w)
    out = {"ok": False, "unit": imu["unit"], "ur_log": str(ur_csv),
           "imu_log": str(imu_csv), "lag": lag,
           "made_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
    if not lag.get("ok"):
        out["error"] = "time offset: " + lag.get("error", "not found")
        return out
    mount = estimate_mount(ur, imu, lag["lag_s"], w)
    out["mount"] = mount
    if not mount.get("ok"):
        out["error"] = "mounting: " + mount.get("error", "not determined")
        return out
    out["ok"] = True
    out["lag_s"] = lag["lag_s"]
    out["R_flange_imu"] = mount["R_flange_imu"]
    return out


def save(cal: dict, path=CAL_PATH) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(cal, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


def load(path=CAL_PATH) -> dict | None:
    try:
        cal = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not cal.get("ok") or "R_flange_imu" not in cal:
        return None
    return cal


def flange_to_imu(cal: dict | None, v):
    """A vector in flange axes, expressed in the IMU's axes."""
    if not cal:
        return list(v)
    R = np.asarray(cal["R_flange_imu"], dtype=float)
    return [float(x) for x in R.T @ np.asarray(v, dtype=float)]


def summary(cal: dict) -> str:
    """What the operator reads: plain sentences, no arrays."""
    lag = cal.get("lag", {})
    lines = []
    if lag.get("ok"):
        ms = lag["lag_s"] * 1000
        lines.append(
            f"IMU timing: its samples arrive {abs(ms):.1f} ms "
            f"{'after' if ms >= 0 else 'before'} the robot's for the same "
            f"motion, consistent to within {lag['spread_s'] * 1000:.1f} ms "
            f"over {lag['windows_used']} stretches of motion.")
        drift = lag.get("drift_ms_per_min")
        if drift is not None and abs(drift) >= 1.0:
            lines.append(f"The offset drifts by {drift:+.1f} ms a minute; "
                         f"repeat this calibration before each session.")
    else:
        lines.append("IMU timing: not found. " + lag.get("error", ""))
    m = cal.get("mount")
    if m:
        if "rotvec_deg" in m:
            c = m.get("conditioning", 0)
            how = ("well determined" if c >= 0.2 else "determined"
                   if c >= MIN_S2_RATIO else "NOT determined")
            lines.append(
                f"IMU mounting: turned {float(np.linalg.norm(m['rotvec_deg'])):.1f}"
                f" deg from the flange's axes ({how}).")
        if "gyro_rel_err" in m:
            lines.append(
                f"Check: the rotated gyro matches the robot's own turning "
                f"rate to {max(0.0, 100 - m['gyro_rel_err'] * 100):.0f}% over "
                f"{m['n_moving']} moving samples.")
        if "gravity_median_deg" in m:
            lines.append(
                f"Check: gravity agrees to {m['gravity_median_deg']:.2f} deg "
                f"over {m['n_still']} still samples.")
        if not m.get("ok"):
            lines.append("Not usable: " + m.get("error", ""))
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ur", required=True, help="robot log CSV")
    ap.add_argument("--imu", required=True, help="IMU log CSV")
    ap.add_argument("--unit", default=None, help="IMU unit (default: the busiest)")
    ap.add_argument("--save", default="", help="write the calibration here")
    args = ap.parse_args(argv)
    cal = calibrate(args.ur, args.imu, args.unit)
    print(summary(cal))
    if not cal["ok"]:
        print("\nnot saved: " + cal.get("error", ""), file=sys.stderr)
        return 1
    if args.save:
        print(f"\nsaved to {save(cal, args.save)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
