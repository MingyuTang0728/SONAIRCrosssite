"""
Synthetic end-to-end demonstration.

Generates a small real/sim dataset with a KNOWN, velocity-dependent gap, then
runs the whole chain over it: budget -> gap -> gates -> baselines -> leaderboard.

This exists so that the toolchain is proven before a single real sample is
recorded. When the Phase 3 data lands, the only thing that changes is where
the files come from. It is also the fastest way to show a supervisor what the
benchmark will look like, which is the thing that was actually asked for.

The injected gap is deliberately shaped like the real one is expected to be:
small and nearly constant below ~0.6 rad/s at the elbow, growing sharply above
it, with a heavier tail at high velocity. That shape is what gives Gate C
something to find and what makes GCR-p95 differ from GCR-median.
"""
from __future__ import annotations

import json
import math
import random
from pathlib import Path

from .budget import from_phase0
from .campaign import plan_campaign, split_cells, save_plan, campaign_size
from .imu import NoiseFloor
from .metrics import aggregate_by_cell, gap_between, gate_c
from .schema import RunWriter, Sample, read_dataset, pair_runs
from .scoring import (baseline_constant_offset, baseline_identity,
                      score_submission, write_leaderboard, SubmissionRun)

CALIB = "calib-demo-1"


def _gap_scale(vel: float) -> float:
    """Small below the divergence, growing sharply above it. Metres."""
    knee = 0.6
    if vel <= knee:
        return 0.0008 + 0.0004 * vel
    return 0.0008 + 0.0004 * knee + 0.030 * (vel - knee) ** 1.6


def _write_run(path: Path, planned, side: str, rng: random.Random,
               duration: float = 6.0, rate: float = 125.0) -> None:
    manifest = planned.manifest(side, CALIB, sample_rate_hz=rate,
                                carrier_mass_kg=0.180,
                                carrier_com_m=(0.0, 0.0, 0.035),
                                notes="synthetic demo data")
    n = int(duration * rate)
    scale = _gap_scale(planned.joint_vel)
    # heavier tail at high velocity — the long-tail cases the benchmark is for
    tail_p = 0.02 + 0.10 * max(0.0, planned.joint_vel - 0.6)
    with RunWriter(path, manifest) as w:
        for i in range(n):
            t = i / rate
            # a contour the arm sweeps; identical command on both sides
            base = [0.35 + 0.10 * math.sin(2 * math.pi * 0.25 * t),
                    0.10 * math.cos(2 * math.pi * 0.25 * t),
                    0.30 + 0.02 * math.sin(2 * math.pi * 0.5 * t)]
            rot = [0.0, 3.14, 0.0]
            if side == "real":
                # systematic lag-like tracking error plus occasional tail event
                err = [scale * math.sin(2 * math.pi * 0.25 * t + 0.3),
                       scale * 0.6 * math.cos(2 * math.pi * 0.25 * t),
                       scale * 0.2]
                if rng.random() < tail_p:
                    err = [e * rng.uniform(3.0, 7.0) for e in err]
                base = [b + e + rng.gauss(0, 2e-5) for b, e in zip(base, err)]
                rot = [r + rng.gauss(0, 3e-4) for r in rot]
            else:
                base = [b + rng.gauss(0, 5e-6) for b in base]
            w.write(Sample(
                t=t, tcp_pos=base, tcp_rot=rot,
                imu={"ind0": {"gyro": [rng.gauss(0, 0.002) for _ in range(3)],
                              "accel": [rng.gauss(0, 0.01), rng.gauss(0, 0.01),
                                        9.807 + rng.gauss(0, 0.01)]}},
            ))


def build_demo(out: Path) -> int:
    out = Path(out)
    rng = random.Random(7)

    runs = plan_campaign(velocities=(0.2, 0.4, 0.6, 0.7, 0.9),
                         configs=("mid_workspace", "extended"),
                         traj_types=("contour", "stop_start"),
                         repeats=3, sessions=2)
    published, held = split_cells(runs, holdout_frac=0.3, seed=3)
    save_plan(out / "campaign" / "plan.json", runs, holdout_frac=0.3,
              published_cells=sorted(published), held_out_cells=sorted(held),
              **campaign_size(runs))

    for p in runs:
        _write_run(out / "data" / "real" / f"{p.run_id}.jsonl", p, "real", rng)
        _write_run(out / "data" / "sim" / f"{p.run_id}.jsonl", p, "sim", rng)

    budget = from_phase0(
        NoiseFloor("ind0", gyro_noise=[0.002, 0.0018, 0.0021],
                   accel_noise=[0.01, 0.01, 0.012]),
        tap_spread_s=0.0008, tracker_distortion_mm=0.9,
        frame_fit_residual_mm=0.6, calib_version=CALIB)
    budget.save(out / "calib" / "budget.json")

    real = read_dataset(out / "data" / "real", side="real")
    sim = read_dataset(out / "data" / "sim", side="sim")
    pairs = pair_runs(real, sim)

    reports = [gap_between(r, s, compute_lag=False).attach_floor(budget)
               for r, s in pairs]
    cell_map = aggregate_by_cell(reports)
    gc = gate_c(cell_map)
    overall = sum(r.pos_median_mm for r in reports) / len(reports)
    gb = budget.gate_b(overall)

    (out / "results").mkdir(parents=True, exist_ok=True)
    (out / "results" / "gap.json").write_text(json.dumps({
        "n_pairs": len(pairs), "overall_pos_median_mm": overall,
        "gate_b": gb, "gate_c": gc, "cell_map": cell_map,
        "runs": [r.summary() for r in reports],
        "measurement_floor": budget.to_dict(),
    }, indent=2), encoding="utf-8")

    fit_pairs = [p for p in pairs if p[0].manifest.cell_key() not in held]
    entries = [
        score_submission(pairs, None, "baseline: identity (simulation unchanged)",
                         budget=budget, held_out_cells=held),
        score_submission(pairs, baseline_constant_offset(pairs, fit_pairs),
                         "baseline: constant offset", budget=budget, held_out_cells=held),
        score_submission(pairs, _oracle_half(pairs), "example: 50% oracle correction",
                         budget=budget, held_out_cells=held),
    ]
    doc = write_leaderboard(out / "site" / "leaderboard.json", entries,
                            budget=budget, gate_c_result=gc)

    print(f"demo dataset: {len(pairs)} paired runs, {len(cell_map)} cells")
    print(f"overall median gap: {overall:.3f} mm   floor: {budget.floor_position_mm():.3f} mm")
    print(f"Gate B: {gb['verdict']} (ratio {gb['worst_ratio']:.2f})")
    print(f"Gate C: {gc['verdict']} (structure ratio {gc['structure_ratio']:.2f})")
    print()
    print(f"{'entry':<46} {'GCR p95':>9} {'GCR med':>9}")
    for e in doc["entries"]:
        print(f"{e['name'][:45]:<46} {e['gcr_p95']:>9.3f} {e['gcr_median']:>9.3f}")
    print(f"\nwritten under {out}/")
    return 0


def _oracle_half(pairs) -> dict[str, SubmissionRun]:
    """
    A deliberately mediocre 'model': move half way from sim towards real.

    It exists to show a mid-table leaderboard entry, and to demonstrate the
    property the benchmark is built around — closing half the MEDIAN error
    while leaving the tails largely intact, so GCR-p95 lags GCR-median.
    """
    from .clock import resample_to
    out = {}
    for real, sim in pairs:
        rt = [float(s["t"]) for s in real.samples if "tcp_pos" in s]
        rp = [[float(v) for v in s["tcp_pos"]] for s in real.samples if "tcp_pos" in s]
        st = [float(s["t"]) for s in sim.samples if "tcp_pos" in s]
        sp = [[float(v) for v in s["tcp_pos"]] for s in sim.samples if "tcp_pos" in s]
        r_on_s = resample_to(st, rt, rp)
        pred = []
        for s_, r_ in zip(sp, r_on_s):
            d = [r_[i] - s_[i] for i in range(3)]
            mag = math.sqrt(sum(v * v for v in d))
            # correct the bulk, but leave the outliers alone — the tail-blind model
            frac = 0.5 if mag < 0.004 else 0.1
            pred.append([s_[i] + d[i] * frac for i in range(3)])
        out[real.manifest.run_id] = SubmissionRun(
            run_id=real.manifest.run_id, t=st, tcp_pos=pred, mode="absolute")
    return out
