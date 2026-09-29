"""
Six-face accelerometer calibration.

An accelerometer at rest measures one g, whichever way up it is. This one does
not. Over a two-minute arc scan the LPMS-B2 on the carrier read, on samples
where it was demonstrably still, anything from 8.31 to 10.09 m/s^2 -- an 18%
spread -- and the value tracked the carrier's ATTITUDE: about 10.00 m/s^2 with
the tool pitched near -15 degrees, about 9.43 near +60.

That pattern is per-axis bias and scale, and it matters here more than it would
elsewhere. The benchmark scores acceleration. It also removes gravity from the
accelerometer using the orientation, so a scale error on one axis leaks
gravity into `linear_accel` as a function of pose -- a systematic that looks
exactly like a pose-dependent sim-to-real gap and survives every average.

WHY IT CANNOT BE FITTED FROM A RUN. The obvious shortcut is to fit the
correction from a recording that already exists, using the still samples. It
was tried on the arc scan and it returns nothing usable: over the whole scan
the sensor's Y axis saw gravity only between -0.54 and +0.99 m/s^2, because
the tool never rolls far in that direction. An axis that is never presented to
gravity carries no information about its own scale, and the least-squares
solution for it is a division by approximately zero.

So the six faces are not ceremony; they are the minimum set of orientations
that constrains all six unknowns. Each axis must see +1 g and -1 g.

The model is deliberately the simple one:

    corrected = (raw - bias) / scale        per axis

No cross-axis misalignment term. Six static poses constrain six parameters and
no more; solving for nine from six measurements would produce three numbers
invented by the solver, and on a part whose spread is 18% the diagonal terms
are what matter anyway.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

GRAVITY = 9.80665
DEFAULT_PATH = Path("accel_cal.json")

# The six faces, as the dominant axis and its sign.
FACES = (
    ("x+", 0, +1), ("x-", 0, -1),
    ("y+", 1, +1), ("y-", 1, -1),
    ("z+", 2, +1), ("z-", 2, -1),
)

# How dominant the axis has to be for a pose to count as that face. cos(25 deg)
# -- generous enough to hold a part against a bench by hand, tight enough that
# two faces can never be confused.
FACE_TOL = math.cos(math.radians(25.0))


def identify_face(accel) -> str:
    """Which face a still reading represents, or "" if it is not square enough."""
    n = math.sqrt(sum(float(v) * float(v) for v in accel))
    if n < 1e-6:
        return ""
    u = [float(v) / n for v in accel]
    for name, axis, sign in FACES:
        if u[axis] * sign >= FACE_TOL:
            return name
    return ""


def fit(faces: dict) -> dict:
    """
    Solve per-axis bias and scale from the six face averages.

    `faces` maps a face name to its mean [ax, ay, az] while still.

    For axis i, the +face reads bias_i + scale_i*g and the -face reads
    bias_i - scale_i*g, so bias and scale come straight out of the sum and the
    difference. No iteration, nothing to converge, and each number traceable to
    two measurements the operator actually made.
    """
    missing = [n for n, _, _ in FACES if n not in faces]
    if missing:
        return {"ok": False, "error":
                "these faces have not been captured yet: " + ", ".join(missing)
                + ". Every axis has to see gravity both ways round, or its "
                  "scale is not determined by anything."}
    bias, scale = [0.0] * 3, [1.0] * 3
    for axis in range(3):
        pos = float(faces[FACES[axis * 2][0]][axis])
        neg = float(faces[FACES[axis * 2 + 1][0]][axis])
        bias[axis] = (pos + neg) / 2.0
        s = (pos - neg) / (2.0 * GRAVITY)
        if s <= 0.1:
            return {"ok": False, "error":
                    f"axis {'xyz'[axis]} gives a scale of {s:.3f}, which cannot "
                    "be right — the two faces for it were probably the same "
                    "way up. Re-capture them."}
        scale[axis] = s

    # What the correction actually achieves, on the captures themselves.
    before, after = [], []
    for name, _a, _s in FACES:
        raw = [float(v) for v in faces[name]]
        before.append(math.sqrt(sum(v * v for v in raw)))
        cor = [(raw[i] - bias[i]) / scale[i] for i in range(3)]
        after.append(math.sqrt(sum(v * v for v in cor)))
    return {
        "ok": True,
        "bias": [round(v, 5) for v in bias],
        "scale": [round(v, 6) for v in scale],
        "faces": {k: [round(float(x), 5) for x in v] for k, v in faces.items()},
        "before_spread_pct": round(100.0 * (max(before) - min(before)) / GRAVITY, 2),
        "after_spread_pct": round(100.0 * (max(after) - min(after)) / GRAVITY, 2),
        "worst_after_err_pct": round(
            100.0 * max(abs(v - GRAVITY) for v in after) / GRAVITY, 3),
        "saved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def apply(accel, cal) -> list:
    """Correct one reading. Returns it unchanged if there is no calibration."""
    if not cal or not cal.get("ok"):
        return [float(v) for v in accel]
    b, s = cal["bias"], cal["scale"]
    return [(float(accel[i]) - b[i]) / s[i] for i in range(3)]


def save(result: dict, path=DEFAULT_PATH) -> dict:
    if not result.get("ok"):
        return result
    path = Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not write {path}: {e}"}
    return {**result, "path": str(path.resolve())}


def load(path=DEFAULT_PATH) -> dict:
    path = Path(path)
    if not path.exists():
        return {"ok": False, "error": "this unit has not been calibrated"}
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"{path} is not readable: {e}"}
    if not (d.get("bias") and d.get("scale")):
        return {"ok": False, "error": f"{path} holds no calibration"}
    d["ok"] = True
    return d


def describe(cal: dict) -> str:
    if not cal or not cal.get("ok"):
        return "not calibrated — one g reads differently depending on which way up it is"
    s = cal["scale"]
    worst = max(abs(v - 1.0) for v in s) * 100.0
    return (f"calibrated: the worst axis was {worst:.1f}% out and now reads one "
            f"g to {cal.get('worst_after_err_pct', 0):.2f}%")
