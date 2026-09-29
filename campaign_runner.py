"""
Run the benchmark campaign as planned, cell by cell, and pick up where it
stopped.

`sonair_benchmark.campaign.plan_campaign` defines the campaign: 6 elbow speeds
x 3 arm configurations x 3 trajectory types = 54 condition cells, 5 repeats
each spread over 3 sessions, with one deliberate carrier refit. Until now
nothing could EXECUTE that plan. The built-in jobs swept four speeds of the
six, had no stop-start trajectory, treated the arm configuration as a label
typed into a box rather than a place the arm actually was, and the arc scan
was labelled joint_vel=0, which matches no cell at all. A campaign collected
with them would have covered a fraction of the plan under cell keys the
analysis could not join.

This module turns one session of the plan into one automation Job:

  * the arm configurations are TAUGHT, not typed: the operator jogs the arm to
    each of near_singular / mid_workspace / extended and saves it, and every
    run starts by driving there, so a run's `arm_config` is where the arm was;
  * each trajectory type is a definite joint-space motion of the elbow at the
    cell's speed, sized so the elbow actually reaches and holds that speed;
  * every run is recorded under the PLAN's run id, so real and simulated runs
    pair by construction;
  * a run is marked done only once its file has been read back and found
    sound, so a session that stops -- e-stop, dropped link, a bad run -- is
    resumed by running it again: done runs are skipped, bad ones re-run;
  * the held-out cells are fixed by the plan's own split and recorded, so the
    test set is decided before any data exists rather than after.

The motions are all ELBOW-ONLY in joint space. That is deliberate: the
campaign's factor is the elbow's angular velocity, and a motion that also
moved other joints would mix their dynamics into every cell.
"""
from __future__ import annotations

import json
import math
import random
import time
from pathlib import Path

try:
    from sonair_benchmark.campaign import plan_campaign, split_cells
    from sonair_benchmark.schema import ARM_CONFIGS
except Exception:       # noqa: BLE001
    plan_campaign = split_cells = None
    ARM_CONFIGS = ("near_singular", "mid_workspace", "extended")

try:
    import ur_kin
except Exception:       # noqa: BLE001
    ur_kin = None

STATE_PATH = Path("campaign") / "state.json"
ELBOW = 2

# Motion parameters, in one place so the audit, the preview and the runner
# all describe the same motion.
PTP_ACCEL = 1.2          # rad/s^2, as every joint_move is commanded
PTP_PLATEAU_S = 0.5      # cruise held at the cell's speed, per leg
CONTOUR_AMP_DEG = 20.0   # half the excursion of the sinusoid
CONTOUR_CYCLES = 2
SS_ACCEL = 3.0           # stop-start: sharper, because transients are the point
SS_CRUISE_S = 0.2
SS_STEPS = 2
SS_DWELL_S = 0.4
SS_MIN_STEP_DEG = 8.0


# ---------------------------------------------------------------------------
# state
# ---------------------------------------------------------------------------

def load_state(path=STATE_PATH) -> dict:
    p = Path(path)
    if p.exists():
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            d.setdefault("configs", {})
            d.setdefault("done", {})
            d.setdefault("rejected", {})
            d.setdefault("sessions", {})
            return d
        except Exception:       # noqa: BLE001
            pass
    return {"configs": {}, "done": {}, "rejected": {}, "sessions": {}}


def save_state(state: dict, path=STATE_PATH) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2), encoding="utf-8")
    tmp.replace(p)          # never a half-written state file


def teach(config: str, q, direction: int = 1, path=STATE_PATH) -> dict:
    if config not in ARM_CONFIGS:
        return {"ok": False, "error": f"{config!r} is not one of {list(ARM_CONFIGS)}"}
    if not q or len(q) < 6:
        return {"ok": False, "error": "the robot is not reporting joint angles"}
    st = load_state(path)
    st["configs"][config] = {
        "q": [round(float(v), 6) for v in q[:6]],
        "direction": 1 if int(direction) >= 0 else -1,
        "taught_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    save_state(st, path)
    return {"ok": True, "config": config, **st["configs"][config]}


# ---------------------------------------------------------------------------
# the motions
# ---------------------------------------------------------------------------

def motion(traj_type: str, v: float, direction: int = 1) -> dict:
    """
    One run's motion: the automation steps, the elbow positions it passes
    through (relative to the start, radians), and roughly how long it takes.
    """
    d = 1 if direction >= 0 else -1
    if traj_type == "point_to_point":
        amp = v * v / PTP_ACCEL + v * PTP_PLATEAU_S
        leg = 2 * v / PTP_ACCEL + (amp - v * v / PTP_ACCEL) / v
        return {"steps": [{"kind": "joint_move", "joint": ELBOW, "joint_vel": v,
                           "plateau_s": PTP_PLATEAU_S, "direction": d}],
                "excursion": [0.0, d * amp], "seconds": 2 * leg}
    if traj_type == "contour":
        # A continuous sinusoid: q = q0 + A(1 - cos wt), so the velocity peaks
        # at A*w = v and never stops until the end. Steady motion, the
        # counterpart of the transients the other two types are made of.
        A = math.radians(CONTOUR_AMP_DEG)
        w = v / A
        secs = CONTOUR_CYCLES * 2 * math.pi / w
        return {"steps": [{"kind": "joint_contour", "joint": ELBOW, "joint_vel": v,
                           "amplitude_deg": CONTOUR_AMP_DEG,
                           "cycles": CONTOUR_CYCLES, "direction": d}],
                "excursion": [0.0, d * 2 * A], "seconds": secs}
    if traj_type == "stop_start":
        step = max(math.radians(SS_MIN_STEP_DEG),
                   v * v / SS_ACCEL + v * SS_CRUISE_S)
        leg = 2 * v / SS_ACCEL + (step - v * v / SS_ACCEL) / v
        return {"steps": [{"kind": "joint_stop_start", "joint": ELBOW,
                           "joint_vel": v, "step_deg": math.degrees(step),
                           "steps": SS_STEPS, "dwell_s": SS_DWELL_S,
                           "accel": SS_ACCEL, "direction": d}],
                "excursion": [0.0, d * step * SS_STEPS],
                "seconds": 2 * SS_STEPS * (leg + SS_DWELL_S)}
    raise ValueError(f"unknown trajectory type {traj_type!r}")


# ---------------------------------------------------------------------------
# the plan
# ---------------------------------------------------------------------------

def the_plan():
    if plan_campaign is None:
        raise RuntimeError("sonair_benchmark is not importable")
    runs = plan_campaign()
    _pub, held = split_cells(runs)
    return runs, held


def session_runs(session: int, state: dict | None = None, seed: int = 0):
    """
    The planned runs of one session, in execution order, done runs removed.

    Grouped by arm configuration so the arm makes three configuration moves
    rather than ninety, and SHUFFLED within each group with a fixed seed so
    speed is not confounded with time: run the speeds in ascending order and a
    thermal drift over the session looks exactly like a speed-dependent gap.
    """
    runs, held = the_plan()
    done = set((state or {}).get("done", {}))
    mine = [r for r in runs if r.session == session and r.run_id not in done]
    rng = random.Random(seed * 1000 + session)
    out = []
    for cfg in ARM_CONFIGS:
        block = [r for r in mine if r.arm_config == cfg]
        rng.shuffle(block)
        out += block
    return out, held


def preview(session: int, state: dict, envelope_ok=None, tcp_now=None,
            q_now=None) -> dict:
    """
    What this session will do, checked before the arm moves at all.

    Every configuration it needs must be taught, and every run's excursion is
    pushed through the kinematics and checked against the cell's safe
    envelope at its extremes. The tool position along the excursion is taken
    as the TAUGHT configuration's tool position plus the flange displacement
    the kinematics predict, so it needs no TCP offset to be known.
    """
    runs, held = session_runs(session, state)
    configs = state.get("configs", {})
    problems, cells, secs = [], {}, 0.0
    need = sorted({r.arm_config for r in runs})
    for c in need:
        if c not in configs:
            problems.append(f"the {c} configuration has not been taught: jog the "
                            f"arm there and press Teach on the Automate page")
    for r in runs:
        cfg = configs.get(r.arm_config)
        m = motion(r.traj_type, r.joint_vel, (cfg or {}).get("direction", 1))
        secs += m["seconds"] + 6.0          # + config move, dwells, file I/O
        key = r.cell_key()
        c = cells.setdefault(key, {"cell": key, "held_out": key in held,
                                   "runs": 0, "excursion_deg": round(
                                       math.degrees(max(abs(e) for e in m["excursion"])), 1),
                                   "seconds": round(m["seconds"], 1)})
        c["runs"] += 1
        if cfg and ur_kin is not None and envelope_ok is not None:
            q0 = list(cfg["q"])
            base = ur_kin.fk(q0)[:3, 3]
            ref = None
            if tcp_now is not None and q_now is not None:
                ref = [float(tcp_now[i]) - float(ur_kin.fk(q_now)[i, 3])
                       for i in range(3)]
            for e in m["excursion"] + [m["excursion"][-1] / 2.0]:
                q = list(q0)
                q[ELBOW] += e
                p = ur_kin.fk(q)[:3, 3]
                pose = [p[i] + (ref[i] if ref else 0.0) for i in range(3)]
                ok, why = envelope_ok(pose + [0.0, 0.0, 0.0])
                if not ok:
                    msg = (f"{r.arm_config}: moving the elbow "
                           f"{math.degrees(e):+.0f} deg for {key} leaves the safe "
                           f"envelope ({why})")
                    if msg not in problems:
                        problems.append(msg)
    refit = any(r.refit_before for r in runs)
    return {"ok": not problems, "session": session, "runs": len(runs),
            "cells": sorted(cells.values(), key=lambda c: c["cell"]),
            "held_out_cells": sum(1 for c in cells.values() if c["held_out"]),
            "minutes": round(secs / 60.0, 1), "refit_before": refit,
            "problems": problems}


def imu_cal_note(carrier: dict, root=".") -> str:
    """
    Whether the IMU's timing and mounting have been measured for THIS carrier.

    Advisory, not a refusal: the runs are good data either way, and the
    calibration can be taken afterwards -- but only while the carrier is still
    the one that was on the arm. After a refit the IMU sits differently and a
    calibration taken then describes the new mounting, not the runs.
    """
    try:
        import imu_align
    except ImportError:
        return ""
    cal = imu_align.load(Path(root) / imu_align.CAL_PATH)
    if cal is None:
        return ("The IMU's timing and mounting have not been measured yet. Run "
                "the imu mount cal job before this session, so the simulator "
                "can compare its IMU with this one.")
    if carrier.get("saved_utc") and \
            cal.get("carrier_saved_utc") != carrier.get("saved_utc"):
        return ("The carrier was re-described after the IMU was last "
                "calibrated. If it was refitted, run the imu mount cal job "
                "again before this session.")
    return ""


def refit_problem(session: int, state: dict, carrier: dict) -> str:
    """
    The plan calls for one deliberate carrier refit, so that refit error is
    MEASURED rather than assumed away. A post-refit session run on a carrier
    that was never re-described is just another pre-refit session with the
    wrong label, and the one thing it existed to show goes unshown.
    """
    runs, _ = session_runs(session, state)
    if not any(r.refit_before for r in runs):
        return ""
    used = {s.get("carrier_saved_utc") for k, s in state.get("sessions", {}).items()
            if int(k) != session}
    if carrier.get("saved_utc") and carrier["saved_utc"] in used:
        return ("this session comes after the planned carrier refit, but the "
                "carrier on the flange is still the one described before "
                "earlier sessions. Take the carrier off, refit it, weigh it and "
                "save it again on the Automate page (with a new name, e.g. "
                "carrier-v2), then start this session")
    return ""


def build_job(session: int, state: dict, state_path=STATE_PATH):
    """One session of the plan as an automation Job."""
    from automation import Job
    runs, held = session_runs(session, state)
    configs = state["configs"]
    steps = [{"kind": "preflight"},
             {"kind": "campaign_session", "session": session,
              "state_path": str(state_path)},
             {"kind": "ur_log_start"}, {"kind": "imu_log_start"}]
    for r in runs:
        cfg = configs[r.arm_config]
        m = motion(r.traj_type, r.joint_vel, cfg.get("direction", 1))
        key = r.cell_key()
        steps += [
            {"kind": "goto_joints", "q": list(cfg["q"]), "speed": 0.4,
             "label": r.arm_config},
            {"kind": "dwell", "seconds": 1.0},
            {"kind": "record_start", "run_id_exact": r.run_id,
             "joint_vel": r.joint_vel, "arm_config": r.arm_config,
             "traj_type": r.traj_type, "repeat_idx": r.repeat_idx,
             "notes": (f"campaign session {session}; cell {key}"
                       + ("; HELD-OUT cell" if key in held else ""))},
            {"kind": "dwell", "seconds": 1.0},
            *[dict(s) for s in m["steps"]],
            {"kind": "dwell", "seconds": 1.0},
            {"kind": "record_stop"},
            {"kind": "campaign_mark", "run_id": r.run_id, "session": session,
             "state_path": str(state_path)},
        ]
    steps += [{"kind": "imu_log_stop"}, {"kind": "ur_log_stop"}]
    return Job(name=f"campaign_session_{session}", steps=steps, repeats=1,
               requires=["robot", "imu"],
               notes=f"Campaign session {session}: {len(runs)} planned runs.")


def mark(state_path, run_id: str, session: int, path: str, audit: dict) -> dict:
    """
    Record a finished run -- as done if its file is sound, as rejected if not.

    Read back rather than assumed: the audit that flags a stale robot link, a
    rate far below the declared one, or a joint that never reached the speed
    its cell is named after is run on the file the moment it closes. A
    rejected run is NOT marked done, so running the session again re-records
    exactly the runs that need it and nothing else.
    """
    st = load_state(state_path)
    notes = [n for n in (audit or {}).get("notes", [])
             if "declares" not in n]     # a slow rate is noted, not fatal
    entry = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             "session": session, "path": path,
             "samples": (audit or {}).get("samples"),
             "achieved_rate_hz": (audit or {}).get("achieved_rate_hz")}
    if notes:
        st["rejected"][run_id] = {**entry, "why": notes}
        st["done"].pop(run_id, None)
    else:
        st["done"][run_id] = entry
        st["rejected"].pop(run_id, None)
    save_state(st, state_path)
    return {"ok": not notes, "notes": notes}


def progress(state: dict) -> dict:
    runs, held = the_plan()
    by_session = {}
    for r in runs:
        s = by_session.setdefault(r.session, {"planned": 0, "done": 0})
        s["planned"] += 1
        if r.run_id in state.get("done", {}):
            s["done"] += 1
    return {"planned": len(runs), "done": len(state.get("done", {})),
            "rejected": len(state.get("rejected", {})),
            "cells": len({r.cell_key() for r in runs}),
            "held_out_cells": len(held),
            "sessions": {str(k): v for k, v in sorted(by_session.items())},
            "configs": {k: {"taught_utc": v.get("taught_utc"),
                            "direction": v.get("direction", 1)}
                        for k, v in state.get("configs", {}).items()}}
