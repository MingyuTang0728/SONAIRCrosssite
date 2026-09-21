"""
Phase 6 — the scoring harness. This is the benchmark proper.

Sam set the shape directly: a metric matrix held privately, external teams
submitting models, those models scored on how well they move simulation data
to real data, with published examples and a stated methodology.

THE INTERFACE (fix this before the dataset is split — two teams reading it
must build the same thing):

  INPUT to a submission, per run:
      manifest       cell coordinates: joint_vel, arm_config, traj_type
      commanded      the commanded trajectory, exactly as the controller got it
      sim_sequence   the Isaac-generated sequence: t, tcp_pos, tcp_rot,
                     imu.{quat,gyro,accel} at the declared rate

  OUTPUT from a submission, per run:
      pred_sequence  predicted REAL-side sequence: the same fields, on the
                     same timestamps as sim_sequence

  A submission may equivalently predict the CORRECTION (pred = sim + delta);
  the harness accepts either and states which in the result.

THE SCORE — Gap Closure Ratio:

      GCR = 1 - err(prediction, real) / err(simulation, real)

  1.0  the submission perfectly reproduces the real side
  0.0  no better than handing back the simulation unchanged
  <0   actively worse than doing nothing

Reported at the median AND at the 95th percentile. The p95 figure is the
long-tail score, and it is the headline number: a model that matches the
centre of the distribution and not its tails scores well on GCR-median and
badly on GCR-p95, and that is precisely the distinction the benchmark exists
to make.

The trivial baselines below set the floor the leaderboard starts from. Run
them before publication — a leaderboard whose bottom entry is unknown is not
interpretable.
"""
from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Callable, Sequence

from . import SUBMISSION_VERSION
from .clock import resample_to
from .metrics import percentile, quat_geodesic_deg, rotvec_to_quat, wasserstein_1d


# ----------------------------------------------------------------------------
# submission interface
# ----------------------------------------------------------------------------

@dataclass
class SubmissionRun:
    """One predicted run coming back from a submitted model."""

    run_id: str
    t: list[float]
    tcp_pos: list[list[float]] = field(default_factory=list)
    tcp_rot: list[list[float]] = field(default_factory=list)
    mode: str = "absolute"   # "absolute" | "correction"

    def validate(self, expect_n: int | None = None) -> list[str]:
        problems = []
        if not self.t:
            problems.append(f"{self.run_id}: empty timestamp array")
        if self.tcp_pos and len(self.tcp_pos) != len(self.t):
            problems.append(f"{self.run_id}: tcp_pos length {len(self.tcp_pos)} != t length {len(self.t)}")
        if self.tcp_rot and len(self.tcp_rot) != len(self.t):
            problems.append(f"{self.run_id}: tcp_rot length {len(self.tcp_rot)} != t length {len(self.t)}")
        if self.mode not in ("absolute", "correction"):
            problems.append(f"{self.run_id}: mode must be 'absolute' or 'correction'")
        if expect_n is not None and self.t and len(self.t) != expect_n:
            problems.append(f"{self.run_id}: expected {expect_n} samples, got {len(self.t)}")
        return problems


def load_submission(path: str | Path) -> dict[str, SubmissionRun]:
    """
    A submission file is JSON Lines, one predicted run per line:
        {"run_id": "...", "mode": "absolute",
         "t": [...], "tcp_pos": [[x,y,z], ...], "tcp_rot": [[rx,ry,rz], ...]}
    """
    out: dict[str, SubmissionRun] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        obj = json.loads(line)
        run = SubmissionRun(
            run_id=obj["run_id"],
            t=[float(v) for v in obj.get("t", [])],
            tcp_pos=[[float(c) for c in p] for p in obj.get("tcp_pos", [])],
            tcp_rot=[[float(c) for c in p] for p in obj.get("tcp_rot", [])],
            mode=obj.get("mode", "absolute"),
        )
        out[run.run_id] = run
    return out


def write_submission(path: str | Path, runs: Sequence[SubmissionRun]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as fh:
        for r in runs:
            fh.write(json.dumps({
                "run_id": r.run_id, "mode": r.mode,
                "t": [round(v, 6) for v in r.t],
                "tcp_pos": [[round(c, 6) for c in p_] for p_ in r.tcp_pos],
                "tcp_rot": [[round(c, 6) for c in p_] for p_ in r.tcp_rot],
            }) + "\n")


# ----------------------------------------------------------------------------
# scoring
# ----------------------------------------------------------------------------

def _pos_errors(ref_t, ref_pos, other_t, other_pos) -> list[float]:
    if not ref_t or not other_t:
        return []
    o = resample_to(ref_t, other_t, other_pos)
    return [math.dist(a, b) * 1000.0 for a, b in zip(ref_pos, o)]


def _ori_errors(ref_t, ref_quat, other_t, other_quat) -> list[float]:
    if not ref_t or not other_t:
        return []
    o = resample_to(ref_t, other_t, other_quat)
    return [quat_geodesic_deg(a, b) for a, b in zip(ref_quat, o)]


@dataclass
class RunScore:
    run_id: str
    cell_key: str
    n: int = 0
    baseline_pos_median_mm: float = 0.0
    baseline_pos_p95_mm: float = 0.0
    pred_pos_median_mm: float = 0.0
    pred_pos_p95_mm: float = 0.0
    gcr_median: float = 0.0
    gcr_p95: float = 0.0
    baseline_ori_median_deg: float = 0.0
    pred_ori_median_deg: float = 0.0
    gcr_orientation: float = 0.0
    wasserstein_closure: float = 0.0
    below_floor: bool = False


def score_run(real_run, sim_run, submission: SubmissionRun | None,
              floor_position_mm: float = 0.0) -> RunScore:
    """
    Score one predicted run against its real counterpart.

    `submission=None` scores the identity baseline, i.e. the simulation used
    unchanged. That is the denominator of every GCR, so it is computed the
    same way here as everywhere else, from the same code path — a benchmark
    whose baseline is computed by a separate path will eventually disagree
    with itself.
    """
    sc = RunScore(run_id=real_run.manifest.run_id,
                  cell_key=real_run.manifest.cell_key())

    rt = [float(s["t"]) for s in real_run.samples if "tcp_pos" in s]
    rp = [[float(v) for v in s["tcp_pos"]] for s in real_run.samples if "tcp_pos" in s]
    st = [float(s["t"]) for s in sim_run.samples if "tcp_pos" in s]
    sp = [[float(v) for v in s["tcp_pos"]] for s in sim_run.samples if "tcp_pos" in s]
    if not rt or not st:
        return sc

    base_err = _pos_errors(rt, rp, st, sp)
    if not base_err:
        return sc
    sc.n = len(base_err)
    sc.baseline_pos_median_mm = statistics.median(base_err)
    sc.baseline_pos_p95_mm = percentile(base_err, 95)
    sc.below_floor = (floor_position_mm > 0
                      and sc.baseline_pos_median_mm < floor_position_mm)

    # orientation baseline
    rot_t = [float(s["t"]) for s in real_run.samples if "tcp_rot" in s]
    rot_q = [rotvec_to_quat(s["tcp_rot"]) for s in real_run.samples if "tcp_rot" in s]
    srot_t = [float(s["t"]) for s in sim_run.samples if "tcp_rot" in s]
    srot_q = [rotvec_to_quat(s["tcp_rot"]) for s in sim_run.samples if "tcp_rot" in s]
    base_ori = _ori_errors(rot_t, rot_q, srot_t, srot_q)
    if base_ori:
        sc.baseline_ori_median_deg = statistics.median(base_ori)

    if submission is None:
        # Identity baseline: prediction IS the simulation.
        sc.pred_pos_median_mm = sc.baseline_pos_median_mm
        sc.pred_pos_p95_mm = sc.baseline_pos_p95_mm
        sc.pred_ori_median_deg = sc.baseline_ori_median_deg
        sc.gcr_median = sc.gcr_p95 = sc.gcr_orientation = 0.0
        return sc

    pt = submission.t
    pp = submission.tcp_pos
    if submission.mode == "correction" and pp:
        sim_on_pred = resample_to(pt, st, sp)
        pp = [[a[i] + b[i] for i in range(3)] for a, b in zip(sim_on_pred, pp)]

    if pp:
        pred_err = _pos_errors(rt, rp, pt, pp)
        if pred_err:
            sc.pred_pos_median_mm = statistics.median(pred_err)
            sc.pred_pos_p95_mm = percentile(pred_err, 95)
            if sc.baseline_pos_median_mm > 1e-9:
                sc.gcr_median = 1.0 - sc.pred_pos_median_mm / sc.baseline_pos_median_mm
            if sc.baseline_pos_p95_mm > 1e-9:
                sc.gcr_p95 = 1.0 - sc.pred_pos_p95_mm / sc.baseline_pos_p95_mm
            # Distributional closure: did the predicted error DISTRIBUTION move
            # towards the real one, or only its centre?
            w_base = wasserstein_1d(base_err, [0.0] * len(base_err))
            w_pred = wasserstein_1d(pred_err, [0.0] * len(pred_err))
            if w_base > 1e-9:
                sc.wasserstein_closure = 1.0 - w_pred / w_base

    pr = submission.tcp_rot
    if pr and rot_t:
        pq = [rotvec_to_quat(v) for v in pr]
        if submission.mode == "correction":
            # A correction on orientation is applied as a rotation vector sum,
            # which is only valid for small corrections — stated in the spec.
            sim_on_pred = resample_to(pt, srot_t, [list(s) for s in
                                                   ([r["tcp_rot"] for r in sim_run.samples
                                                     if "tcp_rot" in r])])
            pq = [rotvec_to_quat([a[i] + b[i] for i in range(3)])
                  for a, b in zip(sim_on_pred, pr)]
        pred_ori = _ori_errors(rot_t, rot_q, pt, pq)
        if pred_ori:
            sc.pred_ori_median_deg = statistics.median(pred_ori)
            if sc.baseline_ori_median_deg > 1e-9:
                sc.gcr_orientation = 1.0 - sc.pred_ori_median_deg / sc.baseline_ori_median_deg

    return sc


@dataclass
class LeaderboardEntry:
    name: str
    submitted: str = ""
    n_runs: int = 0
    n_cells: int = 0
    gcr_median: float = 0.0
    gcr_p95: float = 0.0            # the headline: the long-tail score
    gcr_orientation: float = 0.0
    wasserstein_closure: float = 0.0
    worst_cell_gcr: float = 0.0
    worst_cell: str = ""
    per_cell: dict = field(default_factory=dict)
    notes: str = ""
    version: str = SUBMISSION_VERSION


def score_submission(pairs, submission: dict[str, SubmissionRun] | None,
                     name: str, budget=None,
                     held_out_cells: set[str] | None = None) -> LeaderboardEntry:
    """
    Score a whole submission over a set of (real, sim) pairs.

    If `held_out_cells` is given, only those cells are scored. Holding back
    WHOLE CELLS rather than random samples is deliberate: random holdout tests
    interpolation within a condition, which is easy; whole-cell holdout tests
    generalisation ACROSS conditions, which is the thing that is actually
    unsolved.
    """
    floor = budget.floor_position_mm() if budget else 0.0
    scores: list[RunScore] = []
    for real, sim in pairs:
        key = real.manifest.cell_key()
        if held_out_cells is not None and key not in held_out_cells:
            continue
        sub = submission.get(real.manifest.run_id) if submission else None
        scores.append(score_run(real, sim, sub, floor_position_mm=floor))

    entry = LeaderboardEntry(name=name, n_runs=len(scores))
    if not scores:
        entry.notes = "no scorable runs — check run_id matching between submission and held-out set"
        return entry

    entry.gcr_median = statistics.fmean([s.gcr_median for s in scores])
    entry.gcr_p95 = statistics.fmean([s.gcr_p95 for s in scores])
    entry.gcr_orientation = statistics.fmean([s.gcr_orientation for s in scores])
    entry.wasserstein_closure = statistics.fmean([s.wasserstein_closure for s in scores])

    by_cell: dict[str, list[RunScore]] = {}
    for s in scores:
        by_cell.setdefault(s.cell_key, []).append(s)
    entry.n_cells = len(by_cell)
    for k, group in by_cell.items():
        entry.per_cell[k] = {
            "n": len(group),
            "gcr_median": statistics.fmean([g.gcr_median for g in group]),
            "gcr_p95": statistics.fmean([g.gcr_p95 for g in group]),
            "baseline_pos_median_mm": statistics.fmean([g.baseline_pos_median_mm for g in group]),
            "pred_pos_median_mm": statistics.fmean([g.pred_pos_median_mm for g in group]),
            "below_floor": any(g.below_floor for g in group),
        }
    worst_k = min(entry.per_cell, key=lambda k: entry.per_cell[k]["gcr_p95"])
    entry.worst_cell = worst_k
    entry.worst_cell_gcr = entry.per_cell[worst_k]["gcr_p95"]

    n_below = sum(1 for s in scores if s.below_floor)
    if n_below:
        entry.notes = (f"{n_below}/{len(scores)} runs have a baseline gap below the "
                       f"rig's measurement floor ({floor:.2f} mm); their scores are "
                       f"not interpretable and are reported but flagged")
    return entry


# ----------------------------------------------------------------------------
# trivial baselines — run these BEFORE publication
# ----------------------------------------------------------------------------

def baseline_identity(pairs) -> dict[str, SubmissionRun]:
    """Hand back the simulation unchanged. GCR is 0 by construction."""
    out = {}
    for real, sim in pairs:
        t = [float(s["t"]) for s in sim.samples if "tcp_pos" in s]
        pos = [[float(v) for v in s["tcp_pos"]] for s in sim.samples if "tcp_pos" in s]
        rot = [[float(v) for v in s["tcp_rot"]] for s in sim.samples if "tcp_rot" in s]
        out[real.manifest.run_id] = SubmissionRun(
            run_id=real.manifest.run_id, t=t, tcp_pos=pos,
            tcp_rot=rot if len(rot) == len(t) else [], mode="absolute")
    return out


def baseline_constant_offset(pairs, fit_pairs=None) -> dict[str, SubmissionRun]:
    """
    Add one global constant offset, fitted on `fit_pairs` (the published
    example set) and applied everywhere.

    This is the baseline that matters. It is the dumbest thing that could
    possibly work, so any submitted model that does not beat it has not
    demonstrated anything, and the leaderboard should say so plainly.
    """
    src = fit_pairs if fit_pairs is not None else pairs
    dx = dy = dz = 0.0
    n = 0
    for real, sim in src:
        rt = [float(s["t"]) for s in real.samples if "tcp_pos" in s]
        rp = [[float(v) for v in s["tcp_pos"]] for s in real.samples if "tcp_pos" in s]
        st = [float(s["t"]) for s in sim.samples if "tcp_pos" in s]
        sp = [[float(v) for v in s["tcp_pos"]] for s in sim.samples if "tcp_pos" in s]
        if not rt or not st:
            continue
        sr = resample_to(rt, st, sp)
        for a, b in zip(rp, sr):
            dx += a[0] - b[0]
            dy += a[1] - b[1]
            dz += a[2] - b[2]
            n += 1
    if n:
        dx, dy, dz = dx / n, dy / n, dz / n

    out = {}
    for real, sim in pairs:
        t = [float(s["t"]) for s in sim.samples if "tcp_pos" in s]
        pos = [[float(s["tcp_pos"][0]) + dx, float(s["tcp_pos"][1]) + dy,
                float(s["tcp_pos"][2]) + dz]
               for s in sim.samples if "tcp_pos" in s]
        rot = [[float(v) for v in s["tcp_rot"]] for s in sim.samples if "tcp_rot" in s]
        out[real.manifest.run_id] = SubmissionRun(
            run_id=real.manifest.run_id, t=t, tcp_pos=pos,
            tcp_rot=rot if len(rot) == len(t) else [], mode="absolute")
    return out


def write_leaderboard(path: str | Path, entries: Sequence[LeaderboardEntry],
                      budget=None, gate_c_result=None) -> dict:
    """
    Emit leaderboard.json — the file the benchmark page reads.

    The methodology block travels with the scores on purpose. A benchmark that
    does not publish its own measurement floor will be challenged on exactly
    that point, and the answer should already be in the file.
    """
    ranked = sorted(entries, key=lambda e: e.gcr_p95, reverse=True)
    doc = {
        "benchmark": "SONAIR",
        "title": "Sim2real Operational beNchmark for AI Robotics",
        "headline_metric": "gcr_p95",
        "headline_metric_name": "Gap Closure Ratio at the 95th percentile",
        "methodology": {
            "task": "given an Isaac-generated simulated sequence and its commanded "
                    "trajectory, predict the real UR5e sequence",
            "score": "GCR = 1 - err(prediction, real) / err(simulation, real)",
            "why_p95": "a model matching the centre of the error distribution but "
                       "not its tails scores well at the median and badly at p95; "
                       "the tails are the long-tail cases the benchmark exists for",
            "holdout": "whole condition cells, not random samples, so what is "
                       "tested is generalisation across conditions",
        },
        "measurement_floor": budget.to_dict() if budget else None,
        "gate_c": gate_c_result,
        "entries": [asdict(e) for e in ranked],
    }
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(doc, indent=2), encoding="utf-8")
    return doc
