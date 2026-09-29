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
import threading
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


CONFIG_WORDS = {"near_singular": "Near singular", "mid_workspace": "Mid workspace",
                "extended": "Extended"}


def _tool_offset(q_now, tcp_now):
    """
    Where the tool point sits in the FLANGE's frame, from one reading.

    The robot reports its tool centre point; the kinematics give the flange.
    The difference is fixed to the flange and turns with it, so it is stored
    in flange axes and rotated with every planned pose. Treating it as a
    fixed shift in the base frame -- what the first version did -- puts a
    15 cm tool offset up to 16 cm wrong after a 64 deg elbow move.
    """
    if ur_kin is None or tcp_now is None or q_now is None:
        return None
    T = ur_kin.fk(q_now)
    return T[:3, :3].T @ (ur_kin.np.asarray(tcp_now[:3], dtype=float) - T[:3, 3])


def _tool_at(q, off):
    T = ur_kin.fk(q)
    p = T[:3, 3] if off is None else T[:3, 3] + T[:3, :3] @ off
    return [float(v) for v in p]


def _elbow_path(q0, travel, n=None):
    """The elbow's positions from the start to `travel` rad, every ~4 deg."""
    n = n or max(2, int(abs(travel) / math.radians(4.0)) + 1)
    out = []
    for k in range(n + 1):
        q = list(q0)
        q[ELBOW] += travel * k / n
        out.append(q)
    return out


def max_travel(direction: int = 1, session: int | None = None) -> float:
    """The largest elbow travel (rad, signed) any planned run makes."""
    runs, _ = the_plan()
    kinds = {(r.traj_type, r.joint_vel) for r in runs
             if session is None or r.session == session}
    return max((abs(e) for tt, v in kinds
                for e in motion(tt, v, direction)["excursion"]),
               default=0.0) * (1 if direction >= 0 else -1)


def preview(session: int, state: dict, envelope_ok=None, tcp_now=None,
            q_now=None) -> dict:
    """
    What this session will do, checked before the arm moves at all.

    Every configuration it needs must be taught, and every run's elbow travel
    is pushed through the kinematics every few degrees and the tool point
    checked against the cell's safe envelope. Problems are reported per
    configuration, worst case first, in words.
    """
    runs, held = session_runs(session, state)
    configs = state.get("configs", {})
    problems, cells, secs = [], {}, 0.0
    need = sorted({r.arm_config for r in runs})
    for c in need:
        if c not in configs:
            problems.append(f"{CONFIG_WORDS.get(c, c)} has not been taught: jog "
                            f"the arm there and press Teach here")
    off = _tool_offset(q_now, tcp_now)
    bad = {}         # config -> [conditions failing, worst travel, why]
    total = {}
    checked = {}
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
        if not cfg or ur_kin is None or envelope_ok is None:
            continue
        total.setdefault(r.arm_config, set()).add(key)
        travel = max(m["excursion"], key=abs)
        memo = (r.arm_config, round(travel, 6))
        if memo not in checked:
            checked[memo] = None
            for q in _elbow_path(cfg["q"], travel):
                ok, why = envelope_ok(_tool_at(q, off) + [0.0, 0.0, 0.0])
                if not ok:
                    checked[memo] = (math.degrees(q[ELBOW] - cfg["q"][ELBOW]), why)
                    break
        if checked[memo]:
            b = bad.setdefault(r.arm_config, [set(), 0.0, ""])
            b[0].add(key)
            if not b[2] or abs(checked[memo][0]) < abs(b[1]):
                b[1], b[2] = checked[memo]
    for cfg_name, (keys, deg, why) in sorted(bad.items()):
        problems.append(
            f"{CONFIG_WORDS.get(cfg_name, cfg_name)}: {len(keys)} of "
            f"{len(total.get(cfg_name, keys))} conditions take the tool out of "
            f"the safe envelope, the first after {deg:+.0f} deg of elbow travel "
            f"({why}). Teach it with the tool higher, choose the other elbow "
            f"direction, or use Suggest configurations")
    refit = any(r.refit_before for r in runs)
    return {"ok": not problems, "session": session, "runs": len(runs),
            "cells": sorted(cells.values(), key=lambda c: c["cell"]),
            "held_out_cells": sum(1 for c in cells.values() if c["held_out"]),
            "minutes": round(secs / 60.0, 1), "refit_before": refit,
            "problems": problems}


# ---------------------------------------------------------------------------
# suggesting the three configurations
# ---------------------------------------------------------------------------

# How bent the elbow is at the start of each configuration's runs, in deg
# (0 = arm straight, the elbow singularity). Set by how far the shoulder-to-
# wrist distance reaches of its maximum: 97% (near singular), 85% (extended)
# and 60% (mid workspace).
CONFIG_BEND_DEG = {"near_singular": 28.0, "extended": 64.0, "mid_workspace": 106.0}
MIN_BEND_DEG = 10.0      # never closer to straight than this during a run
MAX_BEND_DEG = 155.0     # never folded tighter than this
MARGIN_M = 0.03          # extra clearance inside the envelope for a suggestion
MIN_RADIUS_M = 0.20      # keep the wrist this far from the base's own axis


def _inside(p, bounds, margin):
    return (bounds["x_min"] + margin <= p[0] <= bounds["x_max"] - margin and
            bounds["y_min"] + margin <= p[1] <= bounds["y_max"] - margin and
            bounds["z_min"] + margin <= p[2] <= bounds["z_max"] - margin)


def suggest(q_now, tcp_now, bounds: dict, envelope_ok=None) -> dict:
    """
    Three configurations, found from where the operator has put the arm.

    Only the shoulder, elbow and first wrist joint change: the base, the
    other two wrist joints and the tool's tilt stay as the operator left
    them, so each suggestion is the same arm, reaching nearer or further in
    the direction it already faces. For each configuration the shoulder angle
    closest to the current one is chosen for which
      * every run's elbow travel, checked every 4 deg, keeps the tool point at
        least 3 cm inside the safe envelope,
      * the elbow never comes within 10 deg of straight,
      * the wrist stays 20 cm from the base's axis, and
      * the move there from here, as movej makes it, stays inside too.
    These are proposals. The operator drives to each one slowly and decides.
    """
    if ur_kin is None:
        return {"ok": False, "error": "the kinematics module is not available"}
    if not q_now or len(q_now) < 6:
        return {"ok": False, "error": "the robot is not reporting joint angles"}
    np = ur_kin.np
    q_now = [float(v) for v in q_now[:6]]
    off = _tool_offset(q_now, tcp_now)
    sign = 1.0 if q_now[ELBOW] >= 0 else -1.0
    tilt = q_now[1] + q_now[2] + q_now[3]      # tool pitch in the arm's plane
    trav = {d: max_travel(d) for d in (1, -1)}
    here = _tool_at(q_now, off)
    if not _inside(here, bounds, 0.0) or (
            envelope_ok is not None and not envelope_ok(here + [0.0] * 3)[0]):
        why = envelope_ok(here + [0.0] * 3)[1] if envelope_ok else ""
        return {"ok": False, "configs": {}, "problems": [
            "The arm is outside the safe envelope where it is now"
            + (f" ({why})" if why else "") + ". Hand-guide or jog it back "
            "inside, nearer the middle of the table, and ask again"]}

    def ok_at(q, margin, radius=True):
        p = _tool_at(q, off)
        if not _inside(p, bounds, margin):
            return False
        if envelope_ok is not None and not envelope_ok(p + [0.0, 0.0, 0.0])[0]:
            return False
        if not radius:
            return True
        w = ur_kin.frames(q)[4][:3, 3]           # wrist centre
        return math.hypot(w[0], w[1]) >= MIN_RADIUS_M

    def search(cfg, base):
        bend = math.radians(CONFIG_BEND_DEG[cfg])
        best = None
        for d in (1, -1):
            travel = trav[d]
            end = sign * bend + travel
            if end * sign < 0 or not (MIN_BEND_DEG <= math.degrees(abs(end))
                                      <= MAX_BEND_DEG):
                continue                    # would straighten through 0
            for sh_deg in np.arange(-180.0, 0.1, 2.0):
                q = list(q_now)
                q[0] = base
                q[1] = math.radians(sh_deg)
                q[2] = sign * bend
                q[3] = tilt - q[1] - q[2]
                cost = max(abs(a - b) for a, b in zip(q, q_now))
                if best is not None and cost >= best[0]:
                    continue
                if not all(ok_at(p, MARGIN_M) for p in _elbow_path(q, travel)):
                    continue
                transit = [list(np.asarray(q_now) + (np.asarray(q) - q_now) * k / 20)
                           for k in range(21)]
                if not all(ok_at(p, 0.0, radius=False) for p in transit):
                    continue
                zs = [_tool_at(p, off)[2] for p in _elbow_path(q, travel)]
                best = (cost, q, d, min(zs), max(zs))
        return best

    out, problems = {}, []
    for cfg in ("near_singular", "mid_workspace", "extended"):
        # First with the base where the operator left it; only if nothing
        # fits there, turned a little further each way, up to 90 deg.
        best = None
        for k in range(0, 10):
            for sgn in ((1,) if k == 0 else (1, -1)):
                b = search(cfg, q_now[0] + sgn * math.radians(10.0 * k))
                if b and (best is None or b[0] < best[0]):
                    best = b
            if best:
                break
        if best is None:
            problems.append(
                f"{CONFIG_WORDS[cfg]}: nothing safe found from this position. "
                f"Start with the arm higher and nearer the middle of the table")
            continue
        cost, q, d, zlo, zhi = best
        out[cfg] = {"q": [round(v, 5) for v in q], "direction": d,
                    "bend_deg": CONFIG_BEND_DEG[cfg],
                    "travel_deg": round(math.degrees(abs(trav[d])), 0),
                    "move_deg": round(math.degrees(cost), 0),
                    "tool_low_cm": round(zlo * 100, 1),
                    "tool_high_cm": round(zhi * 100, 1)}
    return {"ok": not problems, "configs": out, "problems": problems}


# ---------------------------------------------------------------------------
# driving to a configuration slowly, only while the operator holds the button
# ---------------------------------------------------------------------------

GUIDED_SPEED = 0.15     # rad/s at the fastest joint: about 9 deg a second
GUIDED_ACCEL = 0.3      # rad/s^2
HOLD_TIMEOUT_S = 0.4    # this long without a heartbeat and the arm stops


class HoldToMove:
    """
    A move that happens only while a person is holding the button.

    The browser sends a heartbeat every 150 ms for as long as the button is
    held. Letting go sends a stop; so does losing the window, the network, the
    browser or this program's attention for HOLD_TIMEOUT_S, because a watchdog
    stops the arm whenever the heartbeats stop arriving -- whatever the
    reason. Pressing again carries on from wherever the arm stopped.

    This is the software half. The physical e-stop is the other half, and it
    stays within reach.
    """

    def __init__(self, move_fn, stop_fn, joints_fn, path_ok=None):
        self.move_fn, self.stop_fn, self.joints_fn = move_fn, stop_fn, joints_fn
        self.path_ok = path_ok
        self._lock = threading.Lock()
        self._target = None
        self._label = ""
        self._last_beat = 0.0
        self._moving = False
        self.stops = 0
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def remaining_deg(self):
        q, t = self.joints_fn(), self._target
        if not q or not t:
            return None
        return math.degrees(max(abs(a - b) for a, b in zip(q, t)))

    def press(self, target, label="") -> dict:
        """Start (or resume) the move. Returns what the button should show."""
        q = self.joints_fn()
        if not q or len(q) < 6:
            return {"ok": False, "error": "the robot is not reporting joint angles"}
        target = [float(v) for v in target[:6]]
        if self.path_ok is not None:
            ok, why = self.path_ok(q, target)
            if not ok:
                return {"ok": False, "error": why}
        with self._lock:
            self._target, self._label = target, label
            self._last_beat = time.monotonic()
        left = self.remaining_deg()
        if left is not None and left < 0.3:
            return {"ok": True, "arrived": True, "remaining_deg": round(left, 1)}
        ok, why = self.move_fn(target, GUIDED_SPEED, GUIDED_ACCEL)
        if not ok:
            return {"ok": False, "error": why or "the move was not accepted"}
        with self._lock:
            self._moving = True
        return {"ok": True, "arrived": False, "remaining_deg": round(left or 0, 1)}

    def beat(self) -> dict:
        with self._lock:
            self._last_beat = time.monotonic()
            moving = self._moving
        left = self.remaining_deg()
        arrived = left is not None and left < 0.3
        if arrived:
            with self._lock:
                self._moving = False
        return {"ok": True, "moving": moving and not arrived, "arrived": arrived,
                "remaining_deg": None if left is None else round(left, 1),
                "label": self._label}

    def release(self) -> dict:
        with self._lock:
            was = self._moving
            self._moving = False
        if was:
            self.stop_fn()
            self.stops += 1
        left = self.remaining_deg()
        return {"ok": True, "arrived": left is not None and left < 0.3,
                "remaining_deg": None if left is None else round(left, 1),
                "label": self._label}

    def stop_now(self) -> dict:
        """The Stop button: stop whatever the arm is doing, moving or not."""
        with self._lock:
            self._moving = False
        self.stop_fn()
        self.stops += 1
        left = self.remaining_deg()
        return {"ok": True, "arrived": False,
                "remaining_deg": None if left is None else round(left, 1),
                "label": self._label}

    def _watch(self):
        while True:
            time.sleep(0.05)
            with self._lock:
                late = (self._moving and
                        time.monotonic() - self._last_beat > HOLD_TIMEOUT_S)
                if late:
                    self._moving = False
            if late:
                try:
                    self.stop_fn()
                finally:
                    self.stops += 1


def transit_ok(q_from, q_to, envelope_ok, tcp_now=None, q_now=None, n=24):
    """Every point of a movej from q_from to q_to keeps the tool inside."""
    if ur_kin is None or envelope_ok is None:
        return True, ""
    off = _tool_offset(q_now or q_from, tcp_now)
    for k in range(n + 1):
        q = [a + (b - a) * k / n for a, b in zip(q_from, q_to)]
        ok, why = envelope_ok(_tool_at(q, off) + [0.0, 0.0, 0.0])
        if not ok:
            return False, f"the way there leaves the safe envelope: {why}"
    return True, ""


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
