"""
The carrier: what is actually bolted to the flange, and what it weighs.

This file exists because a whole smoke-test run was recorded with
`carrier_mass_kg: 0.0` in its manifest and nothing anywhere said so, and
because that zero is not a cosmetic default -- it is an input to the thing the
project measures.

The benchmark compares a real arm against a simulated one. The simulated one is
built from the run manifest, and the payload on its flange comes from
`carrier_mass_kg` and `carrier_com_m`. Leave them at zero and the simulator
swings a bare flange while the real arm swings a bare flange plus an IMU, a
bracket and a cable. The two then differ -- in exactly the joint torques,
overshoot and settling that a campaign sweeping elbow SPEED is built to
observe -- and the difference lands in the sim-to-real gap, where it is
indistinguishable from the simulator fidelity the gap is supposed to measure.
It inflates Gate B, it varies with speed so it survives Gate C, and nothing
downstream can subtract it out afterwards because nothing downstream knows.

So the mass is asked for once, stored beside the calibration, stamped into
every run, and pre-flight refuses to record until somebody has actually
answered. Zero is a perfectly good answer when the flange really is bare; what
is not acceptable is zero because nobody was asked.

`carrier_id` matters for the same reason at a different timescale: the campaign
design calls for a deliberate refit partway through, and the runs either side
of it are only comparable if the file says which side they are on.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

DEFAULT_PATH = Path("carrier.json")


def blank() -> dict:
    return {
        "carrier_id": "carrier-v1",
        "carrier_mass_kg": 0.0,
        "carrier_com_m": [0.0, 0.0, 0.0],
        "measured": False,
        "note": "",
        "saved_utc": "",
    }


def _clean(data: dict) -> dict:
    out = blank()
    out["carrier_id"] = str(data.get("carrier_id") or "carrier-v1").strip() or "carrier-v1"
    try:
        out["carrier_mass_kg"] = max(0.0, float(data.get("carrier_mass_kg") or 0.0))
    except (TypeError, ValueError):
        out["carrier_mass_kg"] = 0.0
    com = data.get("carrier_com_m") or [0.0, 0.0, 0.0]
    try:
        out["carrier_com_m"] = [float(v) for v in com][:3]
    except (TypeError, ValueError):
        out["carrier_com_m"] = [0.0, 0.0, 0.0]
    while len(out["carrier_com_m"]) < 3:
        out["carrier_com_m"].append(0.0)
    out["note"] = str(data.get("note") or "")[:400]
    out["measured"] = bool(data.get("measured"))
    out["saved_utc"] = str(data.get("saved_utc") or "")
    return out


def validate(data: dict) -> list[str]:
    """Problems worth stopping for. A bare flange is fine; a silent one is not."""
    problems = []
    m = data.get("carrier_mass_kg")
    if m is None:
        problems.append("no mass given")
    elif m > 5.0:
        problems.append(f"{m:g} kg is more than the UR5e's 5 kg payload")
    com = data.get("carrier_com_m") or []
    if any(abs(float(v)) > 0.5 for v in com):
        problems.append("the centre of mass is more than 500 mm from the tool "
                        "flange, which is almost certainly millimetres entered "
                        "as metres")
    if not str(data.get("carrier_id") or "").strip():
        problems.append("no carrier id")
    return problems


def save(data: dict, path: str | Path = DEFAULT_PATH) -> dict:
    out = _clean(data)
    problems = validate(out)
    if problems:
        return {"ok": False, "error": "; ".join(problems), **out}
    # Saving IS the measurement being confirmed. There is no separate tick box,
    # because a tick box next to a number is a tick box people tick.
    out["measured"] = True
    out["saved_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    path = Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not write {path}: {e}", **out}
    return {"ok": True, "path": str(path.resolve()), **out}


def load(path: str | Path = DEFAULT_PATH) -> dict:
    path = Path(path)
    if not path.exists():
        return {**blank(), "ok": False,
                "error": "the carrier on the flange has not been described yet"}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:      # noqa: BLE001
        return {**blank(), "ok": False, "error": f"{path} is not readable: {e}"}
    out = _clean(data)
    return {**out, "ok": bool(out["measured"]), "path": str(path.resolve())}


def describe(data: dict) -> str:
    """One sentence for the operator and for the run's notes."""
    if not data.get("measured"):
        return "not described yet"
    m = data.get("carrier_mass_kg") or 0.0
    com = data.get("carrier_com_m") or [0, 0, 0]
    where = (f", centre of mass {com[0] * 1000:.0f}/{com[1] * 1000:.0f}/"
             f"{com[2] * 1000:.0f} mm from the flange" if any(com) else "")
    if m <= 0.0:
        return f"{data.get('carrier_id')}: nothing on the flange"
    return f"{data.get('carrier_id')}: {m:.3f} kg{where}"
