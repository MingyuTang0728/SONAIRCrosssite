"""
Command line front end.

    python -m sonair_benchmark plan      --out campaign/plan.json
    python -m sonair_benchmark phase0    --fusionhub log.csv --out phase0/ind0.json
    python -m sonair_benchmark budget    --phase0 phase0/ind0.json --out calib/budget.json
    python -m sonair_benchmark gap       --real data/real --sim data/sim \
                                         --budget calib/budget.json --out results/gap.json
    python -m sonair_benchmark score     --real data/real --sim data/sim \
                                         --budget calib/budget.json \
                                         --submission subs/team_a.jsonl --name "Team A" \
                                         --out site/leaderboard.json
    python -m sonair_benchmark demo      --out demo/   (synthetic end-to-end run)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .budget import ErrorBudget, from_phase0
from .campaign import plan_campaign, campaign_size, split_cells, save_plan
from .imu import read_fusionhub_file, stationary_stats, sample_rate_stability
from .metrics import aggregate_by_cell, gap_between, gate_c
from .schema import read_dataset, pair_runs, unpaired
from .scoring import (baseline_constant_offset, baseline_identity, load_submission,
                      score_submission, write_leaderboard)


def cmd_plan(args) -> int:
    runs = plan_campaign(repeats=args.repeats, sessions=args.sessions)
    size = campaign_size(runs, seconds_per_run=args.seconds_per_run)
    published, held = split_cells(runs, holdout_frac=args.holdout)
    save_plan(args.out, runs, holdout_frac=args.holdout,
              published_cells=sorted(published), held_out_cells=sorted(held), **size)
    print(json.dumps(size, indent=2))
    print(f"published cells: {len(published)}   held-out cells: {len(held)}")
    print(f"plan written to {args.out}")
    return 0


def cmd_phase0(args) -> int:
    samples = read_fusionhub_file(args.fusionhub)
    if not samples:
        print(f"no samples parsed from {args.fusionhub}", file=sys.stderr)
        return 1
    nf = stationary_stats(samples, unit=args.unit, expected_hz=args.expected_hz)
    rate = sample_rate_stability(samples)
    doc = {"noise_floor": nf.as_dict(), "rate": rate}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(doc, indent=2), encoding="utf-8")
    print(json.dumps(doc, indent=2))
    if rate["jitter_ms"] > 2.0:
        print("WARNING: sample jitter above 2 ms — a drifting rate reads downstream "
              "as a velocity-dependent gap that is not real", file=sys.stderr)
    return 0


def cmd_budget(args) -> int:
    from .imu import NoiseFloor
    nf = None
    if args.phase0:
        d = json.loads(Path(args.phase0).read_text(encoding="utf-8"))["noise_floor"]
        nf = NoiseFloor(unit=d.get("unit", "ind0"),
                        gyro_bias=d.get("gyro_bias", [0, 0, 0]),
                        gyro_noise=d.get("gyro_noise", [0, 0, 0]),
                        accel_bias=d.get("accel_bias", [0, 0, 0]),
                        accel_noise=d.get("accel_noise", [0, 0, 0]))
    b = from_phase0(nf, tap_spread_s=args.tap_spread_ms / 1000.0,
                    tracker_distortion_mm=args.tracker_mm,
                    frame_fit_residual_mm=args.frame_fit_mm,
                    arm_repeatability_mm=args.arm_repeat_mm,
                    calib_version=args.calib_version)
    b.save(args.out)
    print(json.dumps(b.to_dict(), indent=2))
    if not b.is_complete():
        print(f"INCOMPLETE rows: {b.unmeasured()}", file=sys.stderr)
    return 0


def _load_pairs(real_dir, sim_dir):
    real = read_dataset(real_dir, side="real")
    sim = read_dataset(sim_dir, side="sim")
    pairs = pair_runs(real, sim)
    lonely_real, lonely_sim = unpaired(real, sim)
    if lonely_real or lonely_sim:
        print(f"WARNING: {len(lonely_real)} real and {len(lonely_sim)} sim runs "
              f"failed to pair — an unpaired run is a setup error, not a variable",
              file=sys.stderr)
    return pairs


def cmd_gap(args) -> int:
    pairs = _load_pairs(args.real, args.sim)
    if not pairs:
        print("no paired runs found", file=sys.stderr)
        return 1
    budget = ErrorBudget.load(args.budget) if args.budget else None
    reports = []
    for real, sim in pairs:
        rep = gap_between(real, sim, compute_lag=not args.no_lag)
        if budget:
            rep.attach_floor(budget)
        reports.append(rep)
    cell_map = aggregate_by_cell(reports)
    gc = gate_c(cell_map)

    overall = sum(r.pos_median_mm for r in reports) / len(reports)
    gate_b = budget.gate_b(overall) if budget else None

    doc = {
        "n_pairs": len(pairs),
        "overall_pos_median_mm": overall,
        "gate_b": gate_b,
        "gate_c": gc,
        "cell_map": cell_map,
        "runs": [r.summary() for r in reports],
        "measurement_floor": budget.to_dict() if budget else None,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(doc, indent=2), encoding="utf-8")
    print(f"pairs={len(pairs)}  overall median gap={overall:.3f} mm")
    if gate_b:
        print(f"Gate B: {gate_b['verdict']} (gap/floor = {gate_b['worst_ratio']:.2f})")
    print(f"Gate C: {gc['verdict']} — {gc.get('note','')}")
    print(f"written to {args.out}")
    return 0


def cmd_score(args) -> int:
    pairs = _load_pairs(args.real, args.sim)
    if not pairs:
        print("no paired runs found", file=sys.stderr)
        return 1
    budget = ErrorBudget.load(args.budget) if args.budget else None

    held = None
    if args.plan:
        plan_doc = json.loads(Path(args.plan).read_text(encoding="utf-8"))
        held = set(plan_doc["meta"].get("held_out_cells") or [])
        fit_pairs = [p for p in pairs if p[0].manifest.cell_key() not in held]
    else:
        fit_pairs = pairs

    entries = [
        score_submission(pairs, None, "baseline: identity (simulation unchanged)",
                         budget=budget, held_out_cells=held),
        score_submission(pairs, baseline_constant_offset(pairs, fit_pairs),
                         "baseline: constant offset", budget=budget, held_out_cells=held),
    ]
    if args.submission:
        sub = load_submission(args.submission)
        entries.append(score_submission(pairs, sub, args.name or Path(args.submission).stem,
                                        budget=budget, held_out_cells=held))

    reports = [gap_between(r, s, compute_lag=False) for r, s in pairs]
    gc = gate_c(aggregate_by_cell(reports))
    doc = write_leaderboard(args.out, entries, budget=budget, gate_c_result=gc)

    print(f"{'entry':<46} {'GCR p95':>9} {'GCR med':>9} {'worst cell':>11}")
    for e in doc["entries"]:
        print(f"{e['name'][:45]:<46} {e['gcr_p95']:>9.3f} {e['gcr_median']:>9.3f} "
              f"{e['worst_cell_gcr']:>11.3f}")
    print(f"\nleaderboard written to {args.out}")
    return 0


def cmd_demo(args) -> int:
    """Synthetic end-to-end run: proves the whole chain before real data exists."""
    from .demo import build_demo
    return build_demo(Path(args.out))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="sonair_benchmark",
                                 description="SONAIR sim-to-real benchmark toolchain")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("plan", help="emit the Phase 3 condition sweep")
    p.add_argument("--out", default="campaign/plan.json")
    p.add_argument("--repeats", type=int, default=5)
    p.add_argument("--sessions", type=int, default=3)
    p.add_argument("--seconds-per-run", type=float, default=25.0)
    p.add_argument("--holdout", type=float, default=0.3)
    p.set_defaults(func=cmd_plan)

    p = sub.add_parser("phase0", help="IMU noise floor and rate stability")
    p.add_argument("--fusionhub", required=True)
    p.add_argument("--unit", default="ind0")
    p.add_argument("--expected-hz", type=float, default=None)
    p.add_argument("--out", default="phase0/ind0.json")
    p.set_defaults(func=cmd_phase0)

    p = sub.add_parser("budget", help="assemble the Phase 2 error budget")
    p.add_argument("--phase0")
    p.add_argument("--tap-spread-ms", type=float, default=0.0)
    p.add_argument("--tracker-mm", type=float, default=0.0)
    p.add_argument("--frame-fit-mm", type=float, default=0.0)
    p.add_argument("--arm-repeat-mm", type=float, default=0.03)
    p.add_argument("--calib-version", default="calib-0")
    p.add_argument("--out", default="calib/budget.json")
    p.set_defaults(func=cmd_budget)

    p = sub.add_parser("gap", help="Phase 5: measure the gap and run Gates B and C")
    p.add_argument("--real", required=True)
    p.add_argument("--sim", required=True)
    p.add_argument("--budget")
    p.add_argument("--no-lag", action="store_true")
    p.add_argument("--out", default="results/gap.json")
    p.set_defaults(func=cmd_gap)

    p = sub.add_parser("score", help="Phase 6: score submissions, emit leaderboard.json")
    p.add_argument("--real", required=True)
    p.add_argument("--sim", required=True)
    p.add_argument("--budget")
    p.add_argument("--plan", help="campaign plan, for the held-out cell list")
    p.add_argument("--submission")
    p.add_argument("--name")
    p.add_argument("--out", default="site/leaderboard.json")
    p.set_defaults(func=cmd_score)

    p = sub.add_parser("demo", help="synthetic end-to-end demonstration")
    p.add_argument("--out", default="demo")
    p.set_defaults(func=cmd_demo)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
