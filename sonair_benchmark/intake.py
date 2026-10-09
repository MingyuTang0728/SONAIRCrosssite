"""
intake.py -- bring your own robot's data, and find out how far the reference
simulation is from it.

For people who own a robot (benchmark users of type B). They have a log of
what their arm was told to do and what it did; they want a number: how big
is the sim-to-real gap on THEIR robot, for the motions THEY run.

    python -m sonair_benchmark intake my_log.csv --robot ur5e --out my_report

does, in order:

  1. READ the log. Recognised without help: a UR RTDE recording (the UR
     client library's CSV: timestamp, target_q_0.., actual_q_0..), and the
     SONAIR console's own robot log. Anything else is read through a short
     mapping file that names the columns and units (`--mapping`).
  2. CHECK it, and say plainly what is wrong. The one thing that cannot be
     done without is the COMMANDED joint trajectory: the simulator is fed
     what the robot was told, and a log of only what it did measures nothing.
  3. CUT it into motions: each stretch where the commanded joints move,
     with time to settle on either side.
  4. REPLAY every motion through the reference simulation, S0 -- the
     MuJoCo Menagerie model of that arm, unchanged, exactly as the
     benchmark's own S0 is made -- with the tool offset worked out from the
     log itself.
  5. SCORE each motion by the benchmark's own rule (sonair_benchmark.scoring
     .score_run): tool-position error, median and 95th percentile, plus each
     joint's error, and write a report: report.json and report.html.

The runs are labelled experiment "U": a user's data, measured, never put on
a leaderboard and never released.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path

from .schema import (RunManifest, RunWriter, Sample, read_run, ROBOTS,
                     USER_UNSPECIFIED)

FORMATS = ("auto", "ur-rtde", "sonair-ur-log", "csv")
JOINT_WORDS = ("base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3")

MOVING_RAD_S = 0.02        # commanded speed above which a joint "moves"
MERGE_GAP_S = 0.4          # pauses shorter than this stay inside one motion
PAD_BEFORE_S = 0.5
PAD_AFTER_S = 1.0          # the settle after a stop is where the gap shows
MIN_MOTION_S = 0.3
SPEED_BINS = ((0.0, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, 99.0))

MAPPING_EXAMPLE = {
    "time": "time_s", "time_unit": "s",
    "angle_unit": "rad",
    "target_q": ["cmd_j1", "cmd_j2", "cmd_j3", "cmd_j4", "cmd_j5", "cmd_j6"],
    "q": ["pos_j1", "pos_j2", "pos_j3", "pos_j4", "pos_j5", "pos_j6"],
    "tcp_pos": ["tcp_x", "tcp_y", "tcp_z"], "tcp_unit": "m",
    "tcp_rot": ["tcp_rx", "tcp_ry", "tcp_rz"],
}


class IntakeError(Exception):
    """A problem with the log that stops the analysis, said plainly."""


@dataclass
class Table:
    t: list = field(default_factory=list)
    target_q: list = field(default_factory=list)
    q: list = field(default_factory=list)
    tcp_pos: list | None = None
    tcp_rot: list | None = None
    source: str = ""
    fmt: str = ""
    notes: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# 1. reading
# ---------------------------------------------------------------------------

def _rows(path: Path):
    with open(path, newline="", encoding="utf-8-sig") as fh:
        head = fh.readline()
        delim = "," if head.count(",") >= head.count(" ") else " "
        if "\t" in head and head.count("\t") > head.count(delim):
            delim = "\t"
        fh.seek(0)
        rd = csv.reader(fh, delimiter=delim, skipinitialspace=True)
        cols = [c.strip() for c in next(rd)]
        cols = [c for c in cols if c != ""] if delim == " " else cols
        for row in rd:
            if delim == " ":
                row = [x for x in row if x != ""]
            if row:
                yield cols, row


def detect_format(path: Path) -> str:
    with open(path, encoding="utf-8-sig") as fh:
        head = fh.readline()
    if "target_q_0" in head and "actual_q_0" in head:
        return "ur-rtde"
    if "target_q_base" in head and "actual_q_base" in head:
        return "sonair-ur-log"
    return "csv"


def _num(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _pick(cols, row, names):
    idx = {c: i for i, c in enumerate(cols)}
    out = []
    for n in names:
        i = idx.get(n)
        out.append(_num(row[i]) if i is not None and i < len(row) else None)
    return out


def read_log(path: str | Path, fmt: str = "auto", mapping: dict | None = None) -> Table:
    path = Path(path)
    if not path.exists():
        raise IntakeError(f"no such file: {path}")
    fmt = detect_format(path) if fmt == "auto" else fmt
    if fmt not in FORMATS:
        raise IntakeError(f"unknown format {fmt!r}; one of {FORMATS}")
    if fmt == "ur-rtde":
        m = {"time": "timestamp", "time_unit": "s", "angle_unit": "rad",
             "target_q": [f"target_q_{i}" for i in range(6)],
             "q": [f"actual_q_{i}" for i in range(6)],
             "tcp_pos": [f"actual_TCP_pose_{i}" for i in range(3)], "tcp_unit": "m",
             "tcp_rot": [f"actual_TCP_pose_{i}" for i in range(3, 6)]}
    elif fmt == "sonair-ur-log":
        j = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
        m = {"time": "t_s", "time_unit": "s", "angle_unit": "rad",
             "target_q": [f"target_q_{a}" for a in j],
             "q": [f"actual_q_{a}" for a in j],
             "tcp_pos": [f"actual_TCP_pose_{a}" for a in ("x", "y", "z")], "tcp_unit": "m",
             "tcp_rot": [f"actual_TCP_pose_{a}" for a in ("rx", "ry", "rz")]}
    else:
        if not mapping:
            raise IntakeError(
                "this log is not a format the reader recognises, so it needs a "
                "mapping file naming its columns and units. Write one like this "
                "and pass it as --mapping:\n" + json.dumps(MAPPING_EXAMPLE, indent=2))
        m = dict(mapping)
        for need in ("time", "target_q", "q"):
            if need not in m:
                raise IntakeError(f"the mapping has no {need!r} entry")
    tk = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}.get(m.get("time_unit", "s"))
    ak = {"rad": 1.0, "deg": math.pi / 180.0}.get(m.get("angle_unit", "rad"))
    pk = {"m": 1.0, "mm": 1e-3}.get(m.get("tcp_unit", "m"))
    if tk is None or ak is None or pk is None:
        raise IntakeError("units: time_unit s|ms|us|ns, angle_unit rad|deg, tcp_unit m|mm")
    tb = Table(source=str(path), fmt=fmt)
    have_tcp = bool(m.get("tcp_pos"))
    tcp, rot = [], []
    first_cols = None
    for cols, row in _rows(path):
        if first_cols is None:
            first_cols = cols
            missing = [c for c in [m["time"]] + list(m["target_q"]) + list(m["q"])
                       if c not in cols]
            if missing:
                raise IntakeError(
                    "the log has no column " + ", ".join(repr(c) for c in missing[:4])
                    + (" (and more)" if len(missing) > 4 else "")
                    + (". Without the COMMANDED joints (target_q) a simulator "
                       "cannot be fed what the robot was told, so there is "
                       "nothing to compare. On a UR, record target_q over RTDE."
                       if any(c in m["target_q"] for c in missing) else ""))
            have_tcp = have_tcp and all(c in cols for c in m["tcp_pos"])
        t = _pick(cols, row, [m["time"]])[0]
        tq = _pick(cols, row, m["target_q"])
        q = _pick(cols, row, m["q"])
        if t is None or None in tq or None in q:
            continue
        tb.t.append(t * tk)
        tb.target_q.append([v * ak for v in tq])
        tb.q.append([v * ak for v in q])
        if have_tcp:
            p = _pick(cols, row, m["tcp_pos"])
            r = _pick(cols, row, m["tcp_rot"]) if m.get("tcp_rot") else [None] * 3
            tcp.append(None if None in p else [v * pk for v in p])
            rot.append(None if None in r else r)
    if have_tcp and tcp and all(p is not None for p in tcp):
        tb.tcp_pos = tcp
        if all(r is not None for r in rot):
            tb.tcp_rot = rot
    return tb


# ---------------------------------------------------------------------------
# 2. checking
# ---------------------------------------------------------------------------

def check(tb: Table, robot: str) -> dict:
    """What is wrong with the log, worst first. Fatal problems raise."""
    n = len(tb.t)
    if n < 50:
        raise IntakeError(f"only {n} usable rows; a motion needs hundreds")
    keep = [0]
    for i in range(1, n):
        if tb.t[i] > tb.t[keep[-1]]:
            keep.append(i)
    dropped = n - len(keep)
    if dropped > 0.01 * n:
        raise IntakeError(f"{dropped} of {n} rows go back in time or repeat a "
                          "timestamp; the time column is probably not the "
                          "sample time")
    if dropped:
        for k in ("t", "target_q", "q", "tcp_pos", "tcp_rot"):
            v = getattr(tb, k)
            if v is not None:
                setattr(tb, k, [v[i] for i in keep])
        tb.notes.append(f"{dropped} rows with a repeated or earlier timestamp were dropped")
    dts = [b - a for a, b in zip(tb.t, tb.t[1:])]
    dt = statistics.median(dts)
    rate = 1.0 / dt if dt > 0 else 0.0
    if rate < 20:
        raise IntakeError(f"the log is sampled at {rate:.1f} Hz; the gap lives in "
                          "the first tenths of a second of every move, and at "
                          "under 20 Hz it is not in the data")
    if rate < 100:
        tb.notes.append(f"sampled at {rate:.0f} Hz; 125 Hz or more is better -- the "
                        "settling after a stop is fast")
    gaps = [i for i, d in enumerate(dts) if d > 5 * dt]
    if gaps:
        tb.notes.append(f"{len(gaps)} gaps in the recording (longest "
                        f"{max(dts[i] for i in gaps):.2f} s); motions are cut there")
    big = max(abs(v) for row in tb.q[:2000] for v in row)
    if big > 2 * math.pi + 0.5:
        raise IntakeError(f"a joint angle of {big:.1f} is more than a full turn in "
                          "radians: the log is probably in degrees. Say so in the "
                          "mapping (\"angle_unit\": \"deg\")")
    lag = statistics.median(max(abs(a - b) for a, b in zip(tq, q))
                            for tq, q in zip(tb.target_q[::10], tb.q[::10]))
    if lag > 0.2:
        tb.notes.append(f"commanded and measured joints differ by {lag:.2f} rad "
                        "typically; check the columns are not swapped or offset")
    if tb.tcp_pos is None:
        tb.notes.append("no tool position in the log: the flange is used, worked "
                        f"out from the joints with the nominal {robot_name(robot)} "
                        "kinematics")
    return {"rows": len(tb.t), "rate_hz": round(rate, 1),
            "duration_s": round(tb.t[-1] - tb.t[0], 1), "gaps": len(gaps),
            "dt": dt, "gap_idx": gaps}


# ---------------------------------------------------------------------------
# 3. cutting into motions
# ---------------------------------------------------------------------------

def motions(tb: Table, info: dict) -> list[tuple[int, int, float]]:
    """(first row, last row, peak commanded joint speed) for each motion."""
    t, tq = tb.t, tb.target_q
    n = len(t)
    speed = [0.0] * n
    for i in range(1, n):
        d = t[i] - t[i - 1]
        speed[i] = max(abs(a - b) for a, b in zip(tq[i], tq[i - 1])) / d if d > 0 else 0.0
    # a short running mean: one quantised step is not a motion
    w = max(1, int(round(0.04 / info["dt"])))
    sm = [statistics.fmean(speed[max(0, i - w):i + 1]) for i in range(n)]
    gap_after = set(info["gap_idx"])
    segs, start, last_move = [], None, None
    for i in range(n):
        if i - 1 in gap_after and start is not None:
            segs.append((start, last_move)); start = None
        if sm[i] > MOVING_RAD_S:
            if start is None:
                start = i
            elif t[i] - t[last_move] > MERGE_GAP_S:
                segs.append((start, last_move)); start = i
            last_move = i
    if start is not None:
        segs.append((start, last_move))
    out = []
    for a, b in segs:
        if t[b] - t[a] < MIN_MOTION_S:
            continue
        lo, hi = a, b
        while lo > 0 and t[a] - t[lo - 1] <= PAD_BEFORE_S and (lo - 1) not in gap_after:
            lo -= 1
        while hi < n - 1 and t[hi + 1] - t[b] <= PAD_AFTER_S and hi not in gap_after:
            hi += 1
        out.append((lo, hi, max(speed[a:b + 1])))
    # padding can make neighbours overlap: share the boundary instead
    for k in range(1, len(out)):
        if out[k][0] <= out[k - 1][1]:
            mid = (out[k - 1][1] + out[k][0]) // 2
            out[k - 1] = (out[k - 1][0], mid, out[k - 1][2])
            out[k] = (mid + 1, out[k][1], out[k][2])
    return out


# ---------------------------------------------------------------------------
# 4. writing runs, and replaying them
# ---------------------------------------------------------------------------

def tool_offset(robot: str, q0, tcp0, rot0=None):
    """The tool point (and its rotation) in the flange frame, from one row."""
    import numpy as np
    import ur_kin
    T = ur_kin.fk(q0, ur_kin.DH[robot])
    off = list(T[:3, :3].T @ (np.asarray(tcp0[:3], dtype=float) - T[:3, 3]))
    if rot0 is not None:
        R = T[:3, :3].T @ ur_kin.rotmat(rot0)
        off += list(ur_kin.rotvec(R))
    return [float(v) for v in off]


def write_runs(tb: Table, segs, out: Path, robot: str, payload_kg: float,
               stem: str) -> list[Path]:
    import ur_kin
    dh = ur_kin.DH[robot]
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    rate = 1.0 / statistics.median(b - a for a, b in zip(tb.t, tb.t[1:]))
    for k, (a, b, peak) in enumerate(segs):
        rid = f"{stem}_m{k:03d}"
        m = RunManifest(
            run_id=rid, side="real", calib_version="user",
            joint_vel=round(peak, 3), arm_config=USER_UNSPECIFIED,
            traj_type=USER_UNSPECIFIED, repeat_idx=k, experiment="U",
            robot=robot, carrier_mass_kg=float(payload_kg),
            sample_rate_hz=round(rate, 2),
            started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            operator="intake",
            notes=(f"user data from {Path(tb.source).name} ({tb.fmt}), rows "
                   f"{a}-{b}; joint_vel is the peak COMMANDED speed of the "
                   f"fastest joint, rad/s"))
        probs = m.validate()
        if probs:
            raise IntakeError("; ".join(probs))
        t0 = tb.t[a]
        with RunWriter(out / f"{rid}.jsonl", m) as w:
            for i in range(a, b + 1):
                if tb.tcp_pos is not None:
                    p = tb.tcp_pos[i]
                    r = tb.tcp_rot[i] if tb.tcp_rot is not None else None
                else:
                    fp = ur_kin.fk_pose(tb.q[i], dh)
                    p, r = fp[:3], fp[3:6]
                w.write(Sample(t=round(tb.t[i] - t0, 6), q=tb.q[i],
                               target_q=tb.target_q[i], tcp_pos=p,
                               tcp_rot=r))
        paths.append(out / f"{rid}.jsonl")
    return paths


def s0_model(robot: str, menagerie: Path | None):
    """(menagerie dir, model xml or None, why-not) for this arm's S0."""
    dirs = [menagerie] if menagerie else []
    try:
        import twin
        dirs += twin.menagerie_dirs()
    except Exception:                               # noqa: BLE001
        pass
    for d in dirs:
        if not d:
            continue
        d = Path(d)
        sub = d / f"universal_robots_{robot}"
        if (sub / f"{robot}.xml").exists():
            return d, (None if robot == "ur5e" else sub / f"{robot}.xml"), ""
    return None, None, (f"the MuJoCo Menagerie has no {robot_name(robot)} model here "
                        "(it ships the UR5e and UR10e). Install it with "
                        "python install_sim.py, or pass --menagerie")


# ---------------------------------------------------------------------------
# 5. scoring and the report
# ---------------------------------------------------------------------------

def _joint_errors(real, sim):
    from .clock import resample_to
    rt = [s["t"] for s in real.samples if s.get("q")]
    rq = [s["q"] for s in real.samples if s.get("q")]
    st = [s["t"] for s in sim.samples if s.get("q")]
    sq = [s["q"] for s in sim.samples if s.get("q")]
    if not rt or not st:
        return None
    o = resample_to(rt, st, sq)
    rms = [math.degrees(math.sqrt(statistics.fmean((a[j] - b[j]) ** 2
                                                   for a, b in zip(rq, o))))
           for j in range(6)]
    peak = [math.degrees(max(abs(a[j] - b[j]) for a, b in zip(rq, o))) for j in range(6)]
    return {"rms_deg": [round(v, 3) for v in rms], "peak_deg": [round(v, 3) for v in peak]}


def build_report(real_dir: Path, sim_dir: Path, info: dict, notes: list,
                 robot: str, payload: float, source: str, problems: list) -> dict:
    from .scoring import score_run
    from .schema import read_dataset
    sims = {r.manifest.run_id: r for r in read_dataset(sim_dir, side="sim")} \
        if sim_dir.exists() else {}
    rows = []
    for r in read_dataset(real_dir, side="real"):
        rid = r.manifest.run_id
        s = sims.get(rid)
        row = {"run_id": rid, "peak_speed": r.manifest.joint_vel,
               "duration_s": round(r.samples[-1]["t"] - r.samples[0]["t"], 2),
               "n": len(r.samples)}
        if s is not None:
            sc = score_run(r, s, None)
            row.update(tool_median_mm=round(sc.baseline_pos_median_mm, 3),
                       tool_p95_mm=round(sc.baseline_pos_p95_mm, 3),
                       ori_median_deg=round(sc.baseline_ori_median_deg, 3))
            je = _joint_errors(r, s)
            if je:
                row.update(je)
        rows.append(row)
    scored = [r for r in rows if "tool_p95_mm" in r]
    bins = []
    for lo, hi in SPEED_BINS:
        g = [r for r in scored if lo <= r["peak_speed"] < hi]
        if g:
            bins.append({"from": lo, "to": hi if hi < 99 else None, "motions": len(g),
                         "tool_median_mm": round(statistics.fmean(r["tool_median_mm"] for r in g), 2),
                         "tool_p95_mm": round(statistics.fmean(r["tool_p95_mm"] for r in g), 2)})
    overall = None
    if scored:
        overall = {
            "motions": len(scored),
            "tool_median_mm": round(statistics.fmean(r["tool_median_mm"] for r in scored), 2),
            "tool_p95_mm": round(statistics.fmean(r["tool_p95_mm"] for r in scored), 2),
            "worst": max(scored, key=lambda r: r["tool_p95_mm"])["run_id"],
            "joint_rms_deg": [round(statistics.fmean(r["rms_deg"][j] for r in scored
                                                     if "rms_deg" in r), 3)
                              for j in range(6)] if any("rms_deg" in r for r in scored) else None,
        }
    return {
        "kind": "sonair-intake-report", "version": 1,
        "made_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source": source, "robot": robot, "payload_kg": payload,
        "reference": "S0: MuJoCo Menagerie model of this arm, unchanged, driven "
                     "by the log's own commanded joints",
        "log": {k: v for k, v in info.items() if k not in ("dt", "gap_idx")},
        "notes": notes, "problems": problems,
        "overall": overall, "by_speed": bins, "motions": rows,
        "correction": ("SONAIR's corrected models \u2014 S2, identified on the E1 "
                       "set, and B3, a learned residual \u2014 will be applied to "
                       "these motions once trained on the SONAIR dataset. This "
                       "report is the reference gap they start from."),
    }


def run_intake(src: str | Path, out: str | Path, robot: str = "ur5e",
               fmt: str = "auto", mapping: dict | None = None,
               payload_kg: float = 0.0, menagerie: Path | None = None,
               log=print) -> dict:
    robot = robot.lower()
    if robot not in ROBOTS:
        raise IntakeError(f"robot must be one of {ROBOTS}")
    out = Path(out)
    if out.exists() and any(out.iterdir()):
        raise IntakeError(f"{out} already has files in it; choose an empty folder")
    tb = read_log(src, fmt, mapping)
    info = check(tb, robot)
    log(f"read {info['rows']} rows, {info['duration_s']} s at {info['rate_hz']} Hz ({tb.fmt})")
    segs = motions(tb, info)
    if not segs:
        raise IntakeError("no motion found: the commanded joints never move faster "
                          f"than {MOVING_RAD_S} rad/s")
    log(f"{len(segs)} motions")
    stem = "".join(c if c.isalnum() else "_" for c in Path(src).stem)[:40] or "log"
    real_dir, sim_dir = out / "runs" / "real", out / "runs" / "sim_s0"
    write_runs(tb, segs, real_dir, robot, payload_kg, stem)
    problems = []
    men, model, why = s0_model(robot, Path(menagerie) if menagerie else None)
    if men is None:
        problems.append("S0 not run: " + why)
    else:
        import sim_mujoco as sm
        ok, why = sm.available()
        if not ok:
            problems.append("S0 not run: " + why)
        else:
            for p in sorted(real_dir.glob("*.jsonl")):
                run = read_run(p)
                s0 = run.samples[0]
                off = tool_offset(robot, s0["q"], s0["tcp_pos"], s0.get("tcp_rot"))
                res = sm.replay(run, sim_dir, men, carrier_mass_kg=payload_kg,
                                tcp_offset=off, imu_cal=None, model_xml=model,
                                model_name=f"S0 {robot_name(robot)}" if model else "")
                if not res.get("ok"):
                    problems.append(f"{run.manifest.run_id}: {res.get('error')}")
    rep = build_report(real_dir, sim_dir, info, tb.notes, robot, payload_kg,
                       str(src), problems)
    (out / "report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    (out / "report.html").write_text(report_html(rep), encoding="utf-8")
    log(f"report: {out / 'report.html'}")
    return rep


# ---------------------------------------------------------------------------
# the page
# ---------------------------------------------------------------------------

def robot_name(r: str) -> str:
    """ur5e -> UR5e, as Universal Robots writes it."""
    r = str(r)
    return (r[:-1].upper() + r[-1]) if r.endswith("e") else r.upper()


def _e(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def _f(v, nd=1):
    return "&mdash;" if v is None else f"{v:.{nd}f}"


def report_html(rep: dict) -> str:
    ov = rep.get("overall") or {}
    bins = rep.get("by_speed") or []
    mx = max([b["tool_p95_mm"] for b in bins] + [1e-9])
    def bin_row(b):
        span = (f'{b["from"]:.1f}+' if b["to"] is None
                else f'{b["from"]:.1f}&ndash;{b["to"]:.1f}')
        return (f'<tr><td>{span} rad/s</td><td class="n">{b["motions"]}</td>'
                f'<td class="n">{_f(b["tool_median_mm"])}</td>'
                f'<td class="n">{_f(b["tool_p95_mm"])}</td>'
                f'<td><span class="bar"><i style="width:'
                f'{100 * b["tool_p95_mm"] / mx:.0f}%"></i></span></td></tr>')
    bar_rows = "".join(bin_row(b) for b in bins)
    jr = ov.get("joint_rms_deg") or []
    joints = "".join(f'<div class="stat sm"><div class="k">{_e(w)}</div><div class="v">'
                     f'{_f(v, 2)}<span class="u">&deg;</span></div></div>'
                     for w, v in zip(JOINT_WORDS, jr))
    mot = "".join(
        f'<tr><td>{_e(m["run_id"])}</td><td class="n">{_f(m["peak_speed"], 2)}</td>'
        f'<td class="n">{_f(m["duration_s"], 1)}</td><td class="n">{_f(m.get("tool_median_mm"))}</td>'
        f'<td class="n">{_f(m.get("tool_p95_mm"))}</td></tr>' for m in rep.get("motions", []))
    notes = "".join(f"<li>{_e(n)}</li>" for n in rep.get("notes", []))
    probs = "".join(f"<li>{_e(n)}</li>" for n in rep.get("problems", []))
    lg = rep.get("log", {})
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sim-to-real report</title>
<style>
:root{{--bg:#f4f6f8;--panel:#fff;--ink:#14191f;--muted:#5d6874;--line:#dde2e8;
  --accent:#1f5f9e;--warn:#9a5b00;--mono:"Cascadia Mono",Consolas,ui-monospace,monospace;
  --sans:"Segoe UI",system-ui,-apple-system,Arial,sans-serif;color-scheme:light}}
@media (prefers-color-scheme:dark){{:root{{--bg:#111417;--panel:#191d21;--ink:#e3e7eb;
  --muted:#9aa4ad;--line:#2b3238;--accent:#5fa0dc;--warn:#e0a640;color-scheme:dark}}}}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 var(--sans)}}
.wrap{{max-width:1040px;margin:0 auto;padding:28px 18px 60px}}
.eyebrow{{font:600 11px var(--sans);letter-spacing:.14em;text-transform:uppercase;color:var(--muted)}}
h1{{font-size:28px;margin:4px 0 6px;text-wrap:balance}} h2{{font-size:18px;margin:30px 0 10px}}
p{{max-width:68ch}} .note{{color:var(--muted);font-size:13px}}
.grid{{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(200px,1fr))}}
.grid.j{{grid-template-columns:repeat(auto-fit,minmax(130px,1fr))}}
.stat{{background:var(--panel);border:1px solid var(--line);border-radius:4px;padding:12px 14px}}
.stat .k{{font:600 10.5px var(--sans);letter-spacing:.1em;text-transform:uppercase;color:var(--muted)}}
.stat .v{{font:650 26px var(--mono);font-variant-numeric:tabular-nums;margin-top:2px}}
.stat.sm .v{{font-size:18px}} .stat .u{{font:500 13px var(--sans);color:var(--muted);margin-left:3px}}
.stat .d{{font-size:12.5px;color:var(--muted)}}
.tw{{overflow-x:auto;background:var(--panel);border:1px solid var(--line);border-radius:4px}}
table{{border-collapse:collapse;width:100%;font-size:14px}}
th{{font:600 10.5px var(--sans);letter-spacing:.1em;text-transform:uppercase;color:var(--muted);
  text-align:left;padding:8px 10px;border-bottom:1px solid var(--line)}}
td{{padding:7px 10px;border-bottom:1px solid var(--line)}} td.n,th.n{{text-align:right;
  font-family:var(--mono);font-variant-numeric:tabular-nums}}
.bar{{display:inline-block;width:140px;height:8px;background:var(--line);border-radius:2px;vertical-align:middle}}
.bar i{{display:block;height:100%;background:var(--accent);border-radius:2px}}
ul.warn li{{color:var(--warn)}}
</style></head><body><div class="wrap">
<div class="eyebrow">SONAIR &middot; sim-to-real report &middot; your data</div>
<h1>How far the reference simulation is from your {_e(robot_name(rep["robot"]))}</h1>
<p>Every motion in <code>{_e(Path(rep["source"]).name)}</code> was replayed through the
reference simulation, S0 &mdash; the MuJoCo Menagerie model of this arm, unchanged,
driven by the joints your controller commanded &mdash; and compared with what your arm
actually did, by the SONAIR benchmark's own rule: the distance between the real and
simulated tool point, at the median and the 95th percentile of each motion.</p>
<div class="grid">
 <div class="stat"><div class="k">Tool error, p95</div><div class="v">{_f(ov.get("tool_p95_mm"))}<span class="u">mm</span></div><div class="d">mean over motions</div></div>
 <div class="stat"><div class="k">Tool error, median</div><div class="v">{_f(ov.get("tool_median_mm"))}<span class="u">mm</span></div><div class="d">mean over motions</div></div>
 <div class="stat"><div class="k">Motions</div><div class="v">{ov.get("motions", 0)}</div><div class="d">{lg.get("duration_s", "&mdash;")} s of log at {lg.get("rate_hz", "&mdash;")} Hz</div></div>
 <div class="stat"><div class="k">Payload</div><div class="v">{_f(rep.get("payload_kg"), 2)}<span class="u">kg</span></div><div class="d">as given; it changes the result</div></div>
</div>
<h2>By speed</h2>
<p class="note">Peak commanded speed of the fastest joint in each motion. The gap grows
with speed: this is where a simulator's servo model shows.</p>
<div class="tw"><table><thead><tr><th>Peak joint speed</th><th class="n">Motions</th>
<th class="n">Median mm</th><th class="n">p95 mm</th><th></th></tr></thead><tbody>{bar_rows or '<tr><td colspan="5">no scored motion</td></tr>'}</tbody></table></div>
<h2>Each joint, simulated against real</h2>
<div class="grid j">{joints or '<p class="note">not available</p>'}</div>
<p class="note">RMS of the joint angle difference over each motion, averaged.</p>
<h2>Every motion</h2>
<div class="tw"><table><thead><tr><th>Motion</th><th class="n">Peak rad/s</th><th class="n">Seconds</th>
<th class="n">Median mm</th><th class="n">p95 mm</th></tr></thead><tbody>{mot}</tbody></table></div>
{f'<h2>About the log</h2><ul>{notes}</ul>' if notes else ''}
{f'<h2>Not done</h2><ul class="warn">{probs}</ul>' if probs else ''}
<h2>Closing the gap</h2><p>{_e(rep["correction"])}</p>
<p class="note">Made {_e(rep["made_utc"])}. The run files and the simulated runs are in
<code>runs/</code> beside this page, in the SONAIR run format, so the numbers can be
checked and re-scored. Your data is labelled as yours (experiment U): it is never
placed on a leaderboard or released.</p>
</div></body></html>"""
