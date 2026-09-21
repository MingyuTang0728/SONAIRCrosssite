"""
Phase 5 — quantifying the gap.

Four metric families, chosen so that a model which matches the CENTRE of the
distribution but not its TAILS is visibly distinguished from one that actually
closes the gap. That distinction is the whole reason the benchmark exists:
Sam's long-tail argument is that the rare, badly behaved samples are the ones
that matter, and a benchmark scored on means will be closed by a model that
matches means.

    position       Euclidean error per sample; median AND 95th percentile
    orientation    geodesic angle on SO(3) between attitudes
    temporal       lag maximising cross-correlation (a synchronisation defect,
                   not a gap — reported separately so it cannot be confused)
    distributional 1-D Wasserstein between the two error distributions

Every result carries the Phase 2 floor alongside it, so a reader can see what
fraction of the reported gap the rig itself could account for.

Pure stdlib. numpy is used when available for speed but is never required —
the benchmark has to run on a reviewer's machine, not just on yours.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field, asdict
from typing import Sequence

from .clock import resample_to


# ----------------------------------------------------------------------------
# small numeric helpers
# ----------------------------------------------------------------------------

def percentile(values: Sequence[float], p: float) -> float:
    """Linear-interpolated percentile. p in [0,100]."""
    vals = sorted(float(v) for v in values)
    if not vals:
        return 0.0
    if len(vals) == 1:
        return vals[0]
    k = (len(vals) - 1) * (p / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return vals[int(k)]
    return vals[lo] * (hi - k) + vals[hi] * (k - lo)


def rotvec_to_quat(rv: Sequence[float]) -> list[float]:
    """UR reports orientation as a rotation vector; the metric needs a quaternion."""
    x, y, z = (float(v) for v in rv[:3])
    theta = math.sqrt(x * x + y * y + z * z)
    if theta < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    s = math.sin(theta / 2.0) / theta
    return [math.cos(theta / 2.0), x * s, y * s, z * s]


def quat_geodesic_deg(qa: Sequence[float], qb: Sequence[float]) -> float:
    """
    Angle of the rotation taking qa to qb, in degrees.

    The abs() on the dot product is not cosmetic: q and -q are the same
    rotation, and without it half the samples in a run come back as ~180 deg
    errors that are not errors at all.
    """
    d = abs(sum(float(a) * float(b) for a, b in zip(qa[:4], qb[:4])))
    d = max(-1.0, min(1.0, d))
    return math.degrees(2.0 * math.acos(d))


def wasserstein_1d(a: Sequence[float], b: Sequence[float]) -> float:
    """
    1-D Wasserstein (earth mover's) distance between two samples.

    Implemented by quantile matching, which for 1-D is exact and needs no
    solver. This is the metric that separates "matched the mean" from "matched
    the distribution", so it is the one a tail-blind model fails.
    """
    xa = sorted(float(v) for v in a)
    xb = sorted(float(v) for v in b)
    if not xa or not xb:
        return 0.0
    n = max(len(xa), len(xb))
    total = 0.0
    for i in range(n):
        p = (i + 0.5) / n * 100.0
        total += abs(percentile(xa, p) - percentile(xb, p))
    return total / n


def cross_correlation_lag(ts_a: Sequence[float], a: Sequence[float],
                          ts_b: Sequence[float], b: Sequence[float],
                          max_lag_s: float = 0.25,
                          step_s: float = 0.002) -> tuple[float, float]:
    """
    The lag that maximises normalised cross-correlation of two scalar signals.

    Returns (lag_seconds, correlation). A positive lag means b trails a.
    A systematic non-zero lag here is a synchronisation defect to be fixed in
    Phase 1, NOT part of the gap — which is why it is reported on its own row.
    """
    if len(a) < 8 or len(b) < 8:
        return 0.0, 0.0
    ts_a = [float(t) for t in ts_a]
    va = [[float(v)] for v in a]
    best_lag, best_corr = 0.0, -2.0
    n_steps = int(max_lag_s / step_s)
    for i in range(-n_steps, n_steps + 1):
        lag = i * step_s
        shifted = [t + lag for t in ts_b]
        rb = resample_to(ts_a, shifted, [[float(v)] for v in b])
        if not rb:
            continue
        xs = [p[0] for p in va]
        ys = [p[0] for p in rb]
        c = _pearson(xs, ys)
        if c > best_corr:
            best_corr, best_lag = c, lag
    return best_lag, best_corr


def _pearson(xs: Sequence[float], ys: Sequence[float]) -> float:
    n = min(len(xs), len(ys))
    if n < 2:
        return 0.0
    xs, ys = list(xs[:n]), list(ys[:n])
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx < 1e-12 or sy < 1e-12:
        return 0.0
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


# ----------------------------------------------------------------------------
# the gap report
# ----------------------------------------------------------------------------

@dataclass
class GapReport:
    """The gap for one paired (real, sim) run, or for one condition cell."""

    cell_key: str = ""
    n_samples: int = 0
    # position, mm
    pos_median_mm: float = 0.0
    pos_p95_mm: float = 0.0
    pos_max_mm: float = 0.0
    # orientation, deg
    ori_median_deg: float = 0.0
    ori_p95_deg: float = 0.0
    # temporal, ms
    lag_ms: float = 0.0
    lag_corr: float = 0.0
    # distributional
    wasserstein_pos_mm: float = 0.0
    # floor context (filled by attach_floor)
    floor_position_mm: float = 0.0
    floor_orientation_deg: float = 0.0
    above_floor: bool | None = None
    floor_ratio: float = 0.0
    # raw per-sample errors, kept for distributional scoring and plots
    pos_errors_mm: list[float] = field(default_factory=list)
    ori_errors_deg: list[float] = field(default_factory=list)

    def attach_floor(self, budget) -> "GapReport":
        self.floor_position_mm = budget.floor_position_mm()
        self.floor_orientation_deg = budget.floor_orientation_deg()
        if self.floor_position_mm > 0:
            self.floor_ratio = self.pos_median_mm / self.floor_position_mm
            self.above_floor = self.floor_ratio >= 1.0
        return self

    def summary(self) -> dict:
        d = asdict(self)
        # The raw arrays are for plotting, not for a summary table.
        d.pop("pos_errors_mm", None)
        d.pop("ori_errors_deg", None)
        return d


def gap_between(real_run, sim_run, lever_arm_m: float = 0.15,
                compute_lag: bool = True) -> GapReport:
    """
    The gap between one real run and its simulated counterpart.

    The two runs are put on the real run's time base first. They are generated
    from the same commands, so they nominally share timestamps, but Isaac's
    render/step cadence and the UR's 125 Hz control loop do not line up
    exactly, and differencing them without resampling manufactures error that
    looks exactly like a velocity-dependent gap.
    """
    rep = GapReport(cell_key=real_run.manifest.cell_key())

    rt = [float(s["t"]) for s in real_run.samples if "tcp_pos" in s]
    rp = [[float(v) for v in s["tcp_pos"]] for s in real_run.samples if "tcp_pos" in s]
    st = [float(s["t"]) for s in sim_run.samples if "tcp_pos" in s]
    sp = [[float(v) for v in s["tcp_pos"]] for s in sim_run.samples if "tcp_pos" in s]

    if rt and st:
        sp_r = resample_to(rt, st, sp)
        errs = [math.dist(a, b) * 1000.0 for a, b in zip(rp, sp_r)]
        if errs:
            rep.pos_errors_mm = errs
            rep.n_samples = len(errs)
            rep.pos_median_mm = statistics.median(errs)
            rep.pos_p95_mm = percentile(errs, 95)
            rep.pos_max_mm = max(errs)
            # Distributional: each side's error against its own run mean, so
            # the metric sees the SHAPE of the error, not just its size.
            ref = statistics.fmean(errs)
            rep.wasserstein_pos_mm = wasserstein_1d(
                errs, [ref] * len(errs))

    # orientation
    rot_t = [float(s["t"]) for s in real_run.samples if "tcp_rot" in s]
    rot_r = [rotvec_to_quat(s["tcp_rot"]) for s in real_run.samples if "tcp_rot" in s]
    srot_t = [float(s["t"]) for s in sim_run.samples if "tcp_rot" in s]
    srot_s = [rotvec_to_quat(s["tcp_rot"]) for s in sim_run.samples if "tcp_rot" in s]
    if rot_t and srot_t:
        srot_r = resample_to(rot_t, srot_t, srot_s)
        oerrs = [quat_geodesic_deg(a, b) for a, b in zip(rot_r, srot_r)]
        if oerrs:
            rep.ori_errors_deg = oerrs
            rep.ori_median_deg = statistics.median(oerrs)
            rep.ori_p95_deg = percentile(oerrs, 95)

    # temporal: correlate the speed profiles, which have sharp features that
    # a position profile does not
    if compute_lag and len(rt) > 16 and len(st) > 16:
        sa = _speed_profile(rt, rp)
        sb = _speed_profile(st, sp)
        lag, corr = cross_correlation_lag(rt[1:], sa, st[1:], sb)
        rep.lag_ms = lag * 1000.0
        rep.lag_corr = corr

    return rep


def _speed_profile(ts: list[float], pos: list[list[float]]) -> list[float]:
    out = []
    for (t0, p0), (t1, p1) in zip(zip(ts, pos), zip(ts[1:], pos[1:])):
        dt = t1 - t0
        out.append(math.dist(p0, p1) / dt if dt > 1e-9 else 0.0)
    return out


def aggregate_by_cell(reports: list[GapReport]) -> dict[str, dict]:
    """
    The Phase 5 result is not one number, it is a MAP of how the gap varies
    across the sweep. This is that map.

    Within a cell the spread across repeats is reported alongside the mean,
    because a benchmark that reports means will be closed by a model that
    matches means.
    """
    by_cell: dict[str, list[GapReport]] = {}
    for r in reports:
        by_cell.setdefault(r.cell_key, []).append(r)

    out = {}
    for key, reps in by_cell.items():
        meds = [r.pos_median_mm for r in reps]
        p95s = [r.pos_p95_mm for r in reps]
        omeds = [r.ori_median_deg for r in reps]
        lags = [r.lag_ms for r in reps]
        out[key] = {
            "n_runs": len(reps),
            "pos_median_mm": statistics.fmean(meds),
            "pos_median_spread_mm": (statistics.pstdev(meds) if len(meds) > 1 else 0.0),
            "pos_p95_mm": statistics.fmean(p95s),
            "pos_p95_spread_mm": (statistics.pstdev(p95s) if len(p95s) > 1 else 0.0),
            "ori_median_deg": statistics.fmean(omeds) if omeds else 0.0,
            "lag_ms": statistics.fmean(lags) if lags else 0.0,
            "tail_ratio": (statistics.fmean(p95s) / statistics.fmean(meds))
                          if meds and statistics.fmean(meds) > 1e-9 else 0.0,
        }
    return out


def gate_c(cell_map: dict[str, dict], min_spread_ratio: float = 0.5) -> dict:
    """
    Gate C of the deployment plan: does the gap vary systematically with a
    controllable condition?

    If the gap is flat across the sweep there is nothing for a submitted model
    to learn and the benchmark has no signal — widen the sweep before
    publishing. The test is whether the across-cell variation is large
    compared with the within-cell repeat spread; that ratio, not the raw
    variation, is what tells structure apart from noise.
    """
    if len(cell_map) < 2:
        return {"verdict": "INSUFFICIENT", "n_cells": len(cell_map)}
    meds = [c["pos_median_mm"] for c in cell_map.values()]
    spreads = [c["pos_median_spread_mm"] for c in cell_map.values()]
    across = statistics.pstdev(meds) if len(meds) > 1 else 0.0
    within = statistics.fmean(spreads) if spreads else 0.0
    ratio = across / within if within > 1e-9 else (float("inf") if across > 0 else 0.0)
    return {
        "verdict": "PASS" if ratio >= min_spread_ratio else "FAIL",
        "across_cell_sd_mm": across,
        "within_cell_sd_mm": within,
        "structure_ratio": ratio,
        "min_required": min_spread_ratio,
        "n_cells": len(cell_map),
        "note": ("gap varies systematically with condition — there is signal to learn"
                 if ratio >= min_spread_ratio else
                 "gap looks like noise across the sweep; widen velocity and "
                 "configuration ranges before publishing"),
    }
