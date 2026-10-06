"""
E1, the identification set: every joint excited, recorded as TRAINING data.

The evaluation sweep (E2) moves only the elbow, on purpose, and its runs are
what submissions are scored on. A simulator tuned on those runs has been
tuned on the answers. E1 is the separate training set the benchmark
publishes so that a submission can be identified -- friction, armature,
servo gains, delay -- without touching the evaluation runs, and it follows
what identification benchmarks do (Weigand et al.; PACE):

  * a CHIRP on each joint in turn, 0.05 to 2 Hz, logarithmic so every octave
    gets the same time: slow sweeps for friction at low speed, fast ones for
    the drive's bandwidth;
  * multi-joint FOURIER trajectories (Swevers): every joint at once, each a
    sum of harmonics with its own phases, so the couplings between joints
    are excited as well;
  * all of it at two payloads -- the bare carrier, then with a known added
    mass -- so the payload is something a model can learn rather than a
    constant it can absorb.

HOW IT RUNS. Each excitation is one URScript program on the controller: a
loop that computes the joint velocities from the time and hands them to
speedj every control tick (2 ms), then stops and drives back to the start. Nothing is
streamed from this side, so the motion does not depend on the network or on
the agent keeping up -- the same reason the campaign's sinusoid runs this way.

HOW IT IS KEPT SAFE. The program is a closed-form function of time, so the
whole path is known before anything is sent. `profile` integrates exactly
what the controller will execute, tick by tick, and `check` pushes every
pose of it through the kinematics: the tool point must stay inside the
cell's safe envelope, the elbow must not come near straight or fold
tighter than the campaign allows, and the wrist must keep clear of the
base's axis. An excitation that fails is tried the other way round, then
smaller; one that cannot be made to fit is not run, and the operator is told
why in words. Speeds and accelerations are capped below the cell's limits.
"""
from __future__ import annotations

import json
import math
import random
import time
from pathlib import Path

try:
    import ur_kin
except Exception:       # noqa: BLE001
    ur_kin = None

import campaign_runner as cr

# One speedj step per e-series control tick. At 8 ms the controller met each
# new velocity at full acceleration and then held it, and the commanded
# acceleration became a staircase (see ur_control.joint_sine).
DT = 0.002
TAPER_S = 2.0           # velocity eased in and out over this long
V_MAX = 0.8             # rad/s at any joint, before the cell's own limit
A_MAX = 2.5             # rad/s^2 at any joint
SPEEDJ_ACCEL = 4.0      # what speedj may use to follow the profile (> A_MAX)
RETURN_SPEED = 0.3      # rad/s, the drive back to the start afterwards
RETURN_ACCEL = 1.2

CHIRP_F0, CHIRP_F1, CHIRP_S = 0.05, 2.0, 60.0
# Half-ranges in degrees, base to wrist 3. Small at the base and shoulder,
# which swing the whole arm; larger at the wrist, which moves little mass.
CHIRP_AMP_DEG = (6.0, 6.0, 12.0, 15.0, 15.0, 20.0)

FOURIER_PERIOD_S = 10.0
FOURIER_PERIODS = 2
FOURIER_HARMONICS = (1, 2, 3, 5, 8, 13)          # 0.1 to 1.3 Hz
FOURIER_AMP_DEG = (6.0, 6.0, 12.0, 15.0, 15.0, 20.0)
FOURIER_SEEDS = (0, 1, 2)
FOURIER_REPEATS = 2

LOADS = ("bare", "added")
LOAD_WORDS = {"bare": "the carrier alone", "added": "the carrier with the added mass"}
MIN_ADDED_KG = 0.2      # the added load must weigh at least this much more
SCALES = (1.0, 0.7, 0.5)  # tried in turn when the full size does not fit
CHECK_EVERY = 16        # ticks between kinematic checks (32 ms)


# ---------------------------------------------------------------------------
# the plan
# ---------------------------------------------------------------------------

def plan(load: str) -> list[dict]:
    """The runs of one payload, in execution order."""
    if load not in LOADS:
        raise ValueError(f"load must be one of {LOADS}")
    runs = []
    for j in range(6):
        runs.append({"run_id": f"e1_{load}_mid_workspace_chirp_j{j + 1}_r00",
                     "arm_config": "mid_workspace", "traj_type": "chirp",
                     "repeat_idx": 0, "load": load,
                     "spec": {"kind": "chirp", "joint": j}})
    for cfg in cr.ARM_CONFIGS:
        for seed in FOURIER_SEEDS:
            for rep in range(FOURIER_REPEATS):
                runs.append({
                    "run_id": f"e1_{load}_{cfg}_fourier_s{seed}_r{rep:02d}",
                    "arm_config": cfg, "traj_type": "fourier",
                    "repeat_idx": rep, "load": load,
                    "spec": {"kind": "fourier", "seed": seed}})
    return runs


def _fourier_coeffs(seed: int, vmax: float, scale: float):
    """Per joint, per harmonic: (velocity amplitude, phase)."""
    rng = random.Random(1000 + seed)
    wf = 2 * math.pi / FOURIER_PERIOD_S
    hs = FOURIER_HARMONICS
    out = []
    for j in range(6):
        phases = [rng.uniform(0, 2 * math.pi) for _ in hs]
        A = math.radians(FOURIER_AMP_DEG[j]) * scale
        # A flat velocity spectrum, sized so the worst case of all three
        # bounds holds: position half-range, speed, acceleration.
        c = min(A / sum(1.0 / (h * wf) for h in hs),
                vmax / len(hs),
                A_MAX / sum(h * wf for h in hs))
        out.append([(c, p) for p in phases])
    return out


def duration(spec: dict) -> float:
    if spec["kind"] == "chirp":
        return CHIRP_S
    return FOURIER_PERIOD_S * FOURIER_PERIODS + 2 * TAPER_S


def _taper(t: float, T: float) -> float:
    if t < TAPER_S:
        return 0.5 - 0.5 * math.cos(math.pi * t / TAPER_S)
    if t > T - TAPER_S:
        return 0.5 - 0.5 * math.cos(math.pi * max(0.0, T - t) / TAPER_S)
    return 1.0


def velocity(spec: dict, t: float, vmax: float, co=None) -> list[float]:
    """The six joint velocities the program commands at time t."""
    sign = float(spec.get("sign", 1))
    scale = float(spec.get("scale", 1.0))
    T = duration(spec)
    e = _taper(t, T)
    if spec["kind"] == "chirp":
        j = int(spec["joint"])
        k = CHIRP_F1 / CHIRP_F0
        g = k ** (t / T)
        w = 2 * math.pi * CHIRP_F0 * g
        phi = 2 * math.pi * CHIRP_F0 * T / math.log(k) * (g - 1.0)
        A = math.radians(CHIRP_AMP_DEG[j]) * scale
        v = min(A * w, vmax, A_MAX / w)
        out = [0.0] * 6
        out[j] = sign * e * v * math.sin(phi)
        return out
    wf = 2 * math.pi / FOURIER_PERIOD_S
    co = co or _fourier_coeffs(int(spec["seed"]), vmax, scale)
    return [sign * e * sum(c * math.sin(h * wf * t + p)
                           for h, (c, p) in zip(FOURIER_HARMONICS, co[j]))
            for j in range(6)]


def profile(spec: dict, vmax: float = V_MAX):
    """
    (times, joint offsets from the start) exactly as the controller steps
    through them: velocity held for one tick, then the next.
    """
    vmax = min(vmax, V_MAX)
    T = duration(spec)
    n = int(math.ceil(T / DT))
    dq = [0.0] * 6
    ts, out = [0.0], [list(dq)]
    co = (_fourier_coeffs(int(spec["seed"]), vmax, float(spec.get("scale", 1.0)))
          if spec["kind"] == "fourier" else None)
    for k in range(n):
        v = velocity(spec, k * DT, vmax, co)
        dq = [a + b * DT for a, b in zip(dq, v)]
        ts.append((k + 1) * DT)
        out.append(list(dq))
    return ts, out


# ---------------------------------------------------------------------------
# the safety check
# ---------------------------------------------------------------------------

def check(spec: dict, q0, envelope_ok=None, tool_off=None,
          vmax: float = V_MAX) -> str:
    """Why this excitation is not safe from q0, in words, or "" if it is."""
    if ur_kin is None:
        return "the kinematics module is not available"
    _, path = profile(spec, vmax)
    sign0 = 1.0 if q0[cr.ELBOW] >= 0 else -1.0
    for k in list(range(0, len(path), CHECK_EVERY)) + [len(path) - 1]:
        q = [a + b for a, b in zip(q0, path[k])]
        t = k * DT
        bend = math.degrees(q[cr.ELBOW]) * sign0
        if bend < cr.MIN_BEND_DEG:
            return (f"after {t:.1f} s the elbow would be within "
                    f"{max(bend, 0):.0f} deg of straight")
        if bend > cr.MAX_BEND_DEG:
            return f"after {t:.1f} s the elbow would fold to {bend:.0f} deg"
        p = cr._tool_at(q, tool_off)
        if envelope_ok is not None:
            ok, why = envelope_ok(p + [0.0, 0.0, 0.0])
            if not ok:
                return f"after {t:.1f} s the tool would leave the safe envelope ({why})"
        w = ur_kin.frames(q)[4][:3, 3]
        if math.hypot(w[0], w[1]) < cr.MIN_RADIUS_M:
            return (f"after {t:.1f} s the wrist would come within "
                    f"{cr.MIN_RADIUS_M * 100:.0f} cm of the base's axis")
    return ""


def fit(spec: dict, q0, envelope_ok=None, tool_off=None,
        vmax: float = V_MAX) -> dict:
    """
    The excitation as it will run from q0: full size if it fits, the other
    way round if that fits, then smaller. {"ok", "spec", "why"}.
    """
    last = ""
    for scale in SCALES:
        for sign in (1, -1):
            s = {**spec, "sign": sign, "scale": scale}
            why = check(s, q0, envelope_ok, tool_off, vmax)
            if not why:
                return {"ok": True, "spec": s, "why": ""}
            last = why
    return {"ok": False, "spec": spec, "why": last}


# ---------------------------------------------------------------------------
# the program the controller runs
# ---------------------------------------------------------------------------

def _f(v) -> str:
    return f"{float(v):.9f}"


def urscript(spec: dict, q0, vmax: float = V_MAX) -> str:
    """
    One program: the excitation as a speedj loop, then stop and drive back
    to q0. Its arithmetic is velocity() line for line, so profile() -- which
    the safety check walks -- is the motion the controller makes.
    """
    vmax = min(vmax, V_MAX)
    sign = float(spec.get("sign", 1))
    scale = float(spec.get("scale", 1.0))
    T = duration(spec)
    n = int(math.ceil(T / DT))
    L = ["def sonair_excite():",
         "  k = 0",
         f"  while k < {n}:",
         f"    t = k * {_f(DT)}",
         "    e = 1.0",
         f"    if t < {_f(TAPER_S)}:",
         f"      e = 0.5 - 0.5 * cos({_f(math.pi / TAPER_S)} * t)",
         f"    elif t > {_f(T - TAPER_S)}:",
         f"      e = 0.5 - 0.5 * cos({_f(math.pi / TAPER_S)} * ({_f(T)} - t))",
         "    end"]
    if spec["kind"] == "chirp":
        j = int(spec["joint"])
        k = CHIRP_F1 / CHIRP_F0
        A = math.radians(CHIRP_AMP_DEG[j]) * scale
        L += [f"    g = pow({_f(k)}, t / {_f(T)})",
              f"    w = {_f(2 * math.pi * CHIRP_F0)} * g",
              f"    phi = {_f(2 * math.pi * CHIRP_F0 * T / math.log(k))} * (g - 1.0)",
              f"    v = {_f(A)} * w",
              f"    if v > {_f(vmax)}:",
              f"      v = {_f(vmax)}",
              "    end",
              f"    if v > {_f(A_MAX)} / w:",
              f"      v = {_f(A_MAX)} / w",
              "    end",
              f"    v = {_f(sign)} * e * v * sin(phi)"]
        vec = ["v" if i == j else "0.0" for i in range(6)]
    else:
        wf = 2 * math.pi / FOURIER_PERIOD_S
        co = _fourier_coeffs(int(spec["seed"]), vmax, scale)
        vec = []
        for jj in range(6):
            terms = " + ".join(f"{_f(c)} * sin({_f(h * wf)} * t + {_f(p)})"
                               for h, (c, p) in zip(FOURIER_HARMONICS, co[jj]))
            L.append(f"    v{jj} = {_f(sign)} * e * ({terms})")
            vec.append(f"v{jj}")
    L += [f"    speedj([{', '.join(vec)}], {_f(SPEEDJ_ACCEL)}, {_f(DT)})",
          "    k = k + 1",
          "  end",
          f"  stopj({_f(SPEEDJ_ACCEL)})",
          "  movej([" + ", ".join(_f(x) for x in q0[:6]) +
          f"], a={_f(RETURN_ACCEL)}, v={_f(RETURN_SPEED)})",
          "end"]
    return "\n".join(L)


def program_seconds(spec: dict, vmax: float = V_MAX) -> float:
    """How long the program runs, the drive back included."""
    _, path = profile(spec, vmax)
    back = max(abs(x) for x in path[-1])
    ramp = RETURN_SPEED / RETURN_ACCEL
    ret = (2 * math.sqrt(back / RETURN_ACCEL) if back < RETURN_SPEED * ramp
           else 2 * ramp + (back - RETURN_SPEED * ramp) / RETURN_SPEED)
    return duration(spec) + ret


def words(spec: dict) -> str:
    sc = float(spec.get("scale", 1.0))
    size = "" if sc >= 0.999 else f", reduced to {sc * 100:.0f}% to fit"
    if spec["kind"] == "chirp":
        return (f"joint {int(spec['joint']) + 1} swept from {CHIRP_F0:g} to "
                f"{CHIRP_F1:g} Hz over {CHIRP_S:.0f} s{size}")
    return (f"all six joints together, harmonic set {spec['seed']}, "
            f"{duration(spec):.0f} s{size}")


# ---------------------------------------------------------------------------
# state, preview and the job
# ---------------------------------------------------------------------------

def e1_state(state: dict) -> dict:
    e1 = state.setdefault("e1", {})
    e1.setdefault("done", {})
    e1.setdefault("rejected", {})
    e1.setdefault("loads", {})
    return e1


def load_problem(load: str, state: dict, carrier: dict) -> str:
    """Is the payload on the flange the one this load needs?"""
    e1 = e1_state(state)
    m = float((carrier or {}).get("carrier_mass_kg") or 0.0)
    if not (carrier or {}).get("measured", m > 0):
        return ("the carrier has not been weighed and saved yet. Describe it "
                "on the Automate page first")
    seen = e1["loads"].get(load)
    if seen and abs(float(seen.get("carrier_mass_kg", m)) - m) > 0.02:
        return (f"the {load} runs so far were recorded with "
                f"{seen['carrier_mass_kg']:.3f} kg on the flange, and the "
                f"carrier is now described as {m:.3f} kg. Put back what was "
                f"there, or save the description that matches it")
    if load == "added":
        bare = e1["loads"].get("bare")
        if not bare:
            return ("record the bare-carrier set first: the added-mass set is "
                    "only meaningful against it")
        if m < float(bare["carrier_mass_kg"]) + MIN_ADDED_KG:
            return (f"the carrier is described as {m:.3f} kg, which is not "
                    f"{MIN_ADDED_KG:g} kg more than the {bare['carrier_mass_kg']:.3f} "
                    f"kg of the bare set. Bolt on the added mass, weigh carrier "
                    f"and mass together, save that as the carrier description "
                    f"(with its centre of mass), set the payload on the pendant "
                    f"to match, then run this")
    return ""


def preview(load: str, state: dict, envelope_ok=None, tcp_now=None,
            q_now=None, vmax: float = V_MAX, carrier=None) -> dict:
    """Every run of this load, fitted and checked before anything moves."""
    configs = state.get("configs", {})
    done = set(e1_state(state)["done"])
    off = cr._tool_offset(q_now, tcp_now)
    problems, rows, secs = [], [], 0.0
    for c in sorted({r["arm_config"] for r in plan(load)}):
        if c not in configs:
            problems.append(f"{cr.CONFIG_WORDS.get(c, c)} has not been taught. "
                            f"Teach the three configurations above first")
        else:
            why = cr.bend_problem(c, configs[c].get("q"))
            if why:
                problems.append(why)
    if carrier is not None:
        why = load_problem(load, state, carrier)
        if why:
            problems.append(why)
    fitted = {}
    for r in plan(load):
        if r["run_id"] in done:
            continue
        cfg = configs.get(r["arm_config"])
        row = {"run_id": r["run_id"], "config": r["arm_config"],
               "kind": r["traj_type"]}
        if cfg:
            res = fit(r["spec"], cfg["q"], envelope_ok, off, vmax)
            row["ok"] = res["ok"]
            row["what"] = words(res["spec"])
            if res["ok"]:
                fitted[r["run_id"]] = res["spec"]
                s = program_seconds(res["spec"], vmax)
                row["seconds"] = round(s, 1)
                secs += s + 8.0
            else:
                row["why"] = res["why"]
                problems.append(f"{r['run_id']}: does not fit even at half "
                                f"size -- {res['why']}")
        rows.append(row)
    return {"ok": not problems, "load": load, "runs": len(rows),
            "done": len(done & {r["run_id"] for r in plan(load)}),
            "planned": len(plan(load)), "minutes": round(secs / 60.0, 1),
            "rows": rows, "fitted": fitted, "problems": problems}


def build_job(load: str, state: dict, fitted: dict, state_path=cr.STATE_PATH):
    """One payload's identification set as an automation Job."""
    from automation import Job
    configs = state["configs"]
    steps = [{"kind": "preflight"},
             {"kind": "e1_load", "load": load, "state_path": str(state_path)},
             {"kind": "ur_log_start"}, {"kind": "imu_log_start"}]
    n = 0
    for r in plan(load):
        spec = fitted.get(r["run_id"])
        if spec is None:
            continue
        n += 1
        steps += [
            {"kind": "goto_joints", "q": list(configs[r["arm_config"]]["q"]),
             "speed": 0.4, "label": r["arm_config"], "run_start": True},
            {"kind": "dwell", "seconds": 1.0},
            {"kind": "zero_ft"},
            {"kind": "record_start", "run_id_exact": r["run_id"],
             "experiment": "E1", "joint_vel": 0.0,
             "arm_config": r["arm_config"], "traj_type": r["traj_type"],
             "repeat_idx": r["repeat_idx"],
             "notes": f"E1 identification, {LOAD_WORDS[load]}; excitation "
                      + json.dumps(spec, sort_keys=True)},
            {"kind": "dwell", "seconds": 1.0},
            {"kind": "joint_excite", "spec": spec,
             "q0": list(configs[r["arm_config"]]["q"])},
            {"kind": "dwell", "seconds": 1.0},
            {"kind": "record_stop"},
            {"kind": "campaign_mark", "run_id": r["run_id"], "bucket": "e1",
             "state_path": str(state_path)},
        ]
    steps += [{"kind": "imu_log_stop"}, {"kind": "ur_log_stop"}]
    return Job(name=f"e1_identification_{load}", steps=steps, repeats=1,
               requires=["robot", "imu"],
               notes=f"E1 identification set, {LOAD_WORDS[load]}: {n} runs.")


def stamp_load(load: str, carrier: dict, state_path=cr.STATE_PATH) -> None:
    """Note which payload this load's runs were recorded with."""
    st = cr.load_state(state_path)
    e1 = e1_state(st)
    e1["loads"].setdefault(load, {
        "carrier_mass_kg": float((carrier or {}).get("carrier_mass_kg") or 0.0),
        "carrier_com_m": list((carrier or {}).get("carrier_com_m") or [0, 0, 0]),
        "carrier_id": (carrier or {}).get("carrier_id"),
        "carrier_saved_utc": (carrier or {}).get("saved_utc"),
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())})
    cr.save_state(st, state_path)


def progress(state: dict) -> dict:
    e1 = e1_state(dict(state))
    out = {}
    for load in LOADS:
        ids = {r["run_id"] for r in plan(load)}
        out[load] = {"planned": len(ids), "done": len(ids & set(e1["done"])),
                     "rejected": len(ids & set(e1["rejected"])),
                     "carrier_mass_kg": (e1["loads"].get(load) or {}).get(
                         "carrier_mass_kg")}
    return out
