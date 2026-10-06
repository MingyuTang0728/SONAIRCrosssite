"""
Command line front end.

    python -m sonair_benchmark plan      --out campaign/plan.json
    python -m sonair_benchmark phase0    --fusionhub log.csv --out phase0/ind0.json
    python -m sonair_benchmark budget    --phase0 phase0/ind0.json --out calib/budget.json
    python -m sonair_benchmark gap       --real data/real --sim data/sim \
                                         --budget calib/budget.json --out results/gap.json
    python -m sonair_benchmark simulate  --model subs/team_a/ur5e.xml \
                                         --real data/real --out subs/team_a/sim
    python -m sonair_benchmark score     --real data/real --sim data/sim \
                                         --budget calib/budget.json --plan campaign/plan.json \
                                         --track-a "Team A=subs/team_a/sim" \
                                         --track-b "Team B=subs/team_b.jsonl" \
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
                      score_submission, submission_from_sim_runs, write_leaderboard)


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
                        accel_noise=d.get("accel_noise", [0, 0, 0]),
                        accel_scale_error=d.get("accel_scale_error", 0.0))
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


def _many(dirs):
    return dirs if isinstance(dirs, (list, tuple)) else [dirs]


def _load_pairs(real_dir, sim_dir):
    real = [r for d in _many(real_dir) for r in read_dataset(d, side="real")]
    sim = [r for d in _many(sim_dir) for r in read_dataset(d, side="sim")]
    # The identification set (E1) is training data: published so submissions
    # can be fitted on it, and for that reason never part of what they are
    # scored on.
    n_id = sum(1 for r in real if r.manifest.experiment == "E1")
    if n_id:
        print(f"{n_id} identification runs (E1) left out of scoring")
    real = [r for r in real if r.manifest.experiment != "E1"]
    sim = [r for r in sim if r.manifest.experiment != "E1"]
    # Runs from the simulated cell (sim_cell.py) are rehearsals, never data.
    rehearsal = lambda r: str(r.manifest.notes).startswith("SIMULATED CELL")  # noqa: E731
    n_reh = sum(1 for r in real + sim if rehearsal(r))
    if n_reh:
        print(f"{n_reh} simulated-cell rehearsal runs left out")
    real = [r for r in real if not rehearsal(r)]
    sim = [r for r in sim if not rehearsal(r)]
    pairs = pair_runs(real, sim)
    lonely_real, lonely_sim = unpaired(real, sim)
    if lonely_real or lonely_sim:
        print(f"WARNING: {len(lonely_real)} real and {len(lonely_sim)} sim runs "
              f"failed to pair — an unpaired run is a setup error, not a variable",
              file=sys.stderr)
    return pairs


def _imu_lag(args) -> float:
    """The IMU's measured latency (imu_align), or zero with a warning."""
    path = Path(getattr(args, "imu_cal", "") or "calib/imu_cal.json")
    try:
        cal = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        cal = {}
    if cal.get("ok") and cal.get("lag_s") is not None:
        print(f"IMU latency {cal['lag_s'] * 1000:+.1f} ms taken out of the "
              f"gyro gap (from {path})")
        return float(cal["lag_s"])
    print("WARNING: no IMU calibration found, so the gyro gap includes the "
          "IMU's own latency. Run the imu_mount_cal job first.",
          file=sys.stderr)
    return 0.0


def cmd_gap(args) -> int:
    pairs = _load_pairs(args.real, args.sim)
    if not pairs:
        print("no paired runs found", file=sys.stderr)
        return 1
    budget = ErrorBudget.load(args.budget) if args.budget else None
    imu_lag = _imu_lag(args)
    reports = []
    for real, sim in pairs:
        rep = gap_between(real, sim, compute_lag=not args.no_lag,
                          imu_lag_s=imu_lag)
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


def _named(spec: str) -> tuple[str, str]:
    if "=" not in spec:
        return Path(spec).stem, spec
    name, _, path = spec.partition("=")
    return name.strip(), path.strip()


def dataset_summary(real_dir, held=None) -> dict:
    """What the scores were computed over, for the page's Data section."""
    runs = [r for d in _many(real_dir) for r in read_dataset(d, side="real")
            if not str(r.manifest.notes).startswith("SIMULATED CELL")]
    by_exp: dict[str, int] = {}
    for r in runs:
        by_exp[r.manifest.experiment] = by_exp.get(r.manifest.experiment, 0) + 1
    e2 = [r for r in runs if r.manifest.experiment != "E1"]
    return {"runs": by_exp,
            "cells": len({r.manifest.cell_key() for r in e2}),
            "held_out_cells": len(held) if held else 0,
            "rate_hz": max((r.manifest.sample_rate_hz for r in runs), default=0)}


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
        if not fit_pairs:
            # B1 is FITTED, and fitting it on the cells it is scored on would
            # hand the baseline the answers. With no published cell given it
            # is fitted on nothing (a zero offset) and says so.
            print("WARNING: no published-cell runs given, so B1 has nothing to "
                  "be fitted on. Pass the public release's published folders "
                  "too: --real <public>/E2/published/real <private>/E2/heldout/real",
                  file=sys.stderr)
    else:
        fit_pairs = pairs
        print("WARNING: no --plan, so every cell is scored and B1 is fitted on "
              "all of them -- fine for a check, not for a leaderboard",
              file=sys.stderr)

    entries = [
        score_submission(pairs, None, "S0 · MuJoCo menagerie UR5e, unchanged",
                         budget=budget, held_out_cells=held, track="A",
                         kind="baseline",
                         description="the reference simulation; GCR 0 by definition"),
        score_submission(pairs, baseline_constant_offset(pairs, fit_pairs),
                         "B1 · one constant offset", budget=budget,
                         held_out_cells=held, track="B", kind="baseline",
                         description="one XYZ offset fitted on the published cells"),
    ]
    for spec in args.track_a or []:
        name, path = _named(spec)
        cand = [r for r in read_dataset(path, side="sim")
                if r.manifest.experiment != "E1"]
        if not cand:
            print(f"WARNING: no simulated runs under {path} for {name}", file=sys.stderr)
        entries.append(score_submission(
            pairs, submission_from_sim_runs(cand, pairs), name, budget=budget,
            held_out_cells=held, track="A"))
    b_specs = list(args.track_b or [])
    if args.submission:
        b_specs.append(f"{args.name or Path(args.submission).stem}={args.submission}")
    for spec in b_specs:
        name, path = _named(spec)
        entries.append(score_submission(pairs, load_submission(path), name,
                                        budget=budget, held_out_cells=held,
                                        track="B"))

    reports = [gap_between(r, s, compute_lag=False) for r, s in pairs]
    gc = gate_c(aggregate_by_cell(reports))
    doc = write_leaderboard(args.out, entries, budget=budget, gate_c_result=gc,
                            dataset=dataset_summary(args.real, held))

    print(f"{'entry':<46} {'track':>5} {'GCR p95':>9} {'GCR med':>9} {'worst cell':>11}")
    for e in doc["entries"]:
        print(f"{e['name'][:45]:<46} {e['track']:>5} {e['gcr_p95']:>9.3f} "
              f"{e['gcr_median']:>9.3f} {e['worst_cell_gcr']:>11.3f}")
    print(f"\nleaderboard written to {args.out}")
    return 0


def cmd_simulate(args) -> int:
    """Track A: replay every recorded run through a submitted MuJoCo model."""
    root = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root))
    import sim_mujoco
    argv = ["--real", args.real, "--out", args.out, "--model", args.model]
    if args.menagerie:
        argv += ["--menagerie", args.menagerie]
    if args.tcp_offset:
        argv += ["--tcp-offset", args.tcp_offset]
    return sim_mujoco.main(argv)


def cmd_release(args) -> int:
    from .release import build_release
    try:
        res = build_release(args.runs, args.out, sim_dir=args.sim, plan_path=args.plan,
                            state_path=args.state, private_out=args.private_out,
                            menagerie=args.menagerie, make_s0=not args.no_make_s0,
                            version=args.version, imu_cal_path=args.imu_cal,
                            tcp_offset=[float(v) for v in args.tcp_offset.split(",")]
                            if args.tcp_offset else None)
    except FileExistsError as e:
        print(e, file=sys.stderr)
        return 2
    c = res["counts"]
    print(f"public release : {res['out']}\n"
          f"  E1 {c['E1']} runs, E2 published {c['E2_published']}, "
          f"E2 held-out {c['E2_heldout']} (commands + S0 only)\n"
          f"private set    : {res['private']}  -- never publish\n"
          f"  E2 held-out real {c['E2_heldout']}, E3 {c['E3']}")
    for s in res["skipped"]:
        print("  left out: " + s)
    if res["problems"]:
        print("\nNOT SAFE TO PUBLISH:", file=sys.stderr)
        for p in res["problems"]:
            print("  " + p, file=sys.stderr)
        return 1
    print("\nverified: nothing from a held-out cell or E3 is in the public folder, "
          "and every file matches its checksum")
    return 0


def cmd_verify_release(args) -> int:
    from .release import verify_release
    problems = verify_release(args.folder)
    if problems:
        print("NOT SAFE TO PUBLISH:")
        for p in problems:
            print("  " + p)
        return 1
    print(f"{args.folder}: safe to publish -- no held-out or E3 real data, no logs, "
          "every file matches its checksum")
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
    p.add_argument("--imu-cal", default="calib/imu_cal.json",
                   help="IMU calibration from the imu_mount_cal job")
    p.add_argument("--out", default="results/gap.json")
    p.set_defaults(func=cmd_gap)

    p = sub.add_parser("score", help="Phase 6: score submissions, emit leaderboard.json")
    p.add_argument("--real", required=True, nargs="+",
                   help="real run folder(s): e.g. the public published cells and "
                        "the private held-out cells together")
    p.add_argument("--sim", required=True, nargs="+", help="S0 run folder(s)")
    p.add_argument("--budget")
    p.add_argument("--plan", help="campaign plan, for the held-out cell list")
    p.add_argument("--submission", help="a Track B prediction file (legacy form)")
    p.add_argument("--name")
    p.add_argument("--track-a", action="append", metavar="NAME=DIR",
                   help="a Track A entry: a folder of its simulated runs "
                        "(repeatable)")
    p.add_argument("--track-b", action="append", metavar="NAME=FILE",
                   help="a Track B entry: a prediction file (repeatable)")
    p.add_argument("--out", default="site/leaderboard.json")
    p.set_defaults(func=cmd_score)

    p = sub.add_parser("simulate", help="Track A: replay recorded runs through "
                                        "a submitted MuJoCo model")
    p.add_argument("--model", required=True, help="the submitted MJCF file")
    p.add_argument("--real", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--menagerie", default="")
    p.add_argument("--tcp-offset", default="")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("release", help="build the public dataset release and its "
                                       "private scoring set")
    p.add_argument("--runs", default="bench_runs", help="the recorded run files")
    p.add_argument("--out", required=True, help="a new folder for the public release")
    p.add_argument("--private-out", default="",
                   help="where the private scoring set goes (default: <out>_PRIVATE)")
    p.add_argument("--sim", default="", help="existing S0 simulations, if any")
    p.add_argument("--plan", default="", help="campaign plan (held-out cells)")
    p.add_argument("--state", default="campaign/state.json",
                   help="campaign state: only runs it accepted are released")
    p.add_argument("--menagerie", default="")
    p.add_argument("--tcp-offset", default="",
                   help="the pendant's TCP, x,y,z[,rx,ry,rz] -- as for sim_mujoco")
    p.add_argument("--imu-cal", default="calib/imu_cal.json")
    p.add_argument("--no-make-s0", action="store_true",
                   help="do not simulate missing S0 runs; leave those runs out")
    p.add_argument("--version", default="")
    p.set_defaults(func=cmd_release)

    p = sub.add_parser("verify-release", help="check a release folder is safe to publish")
    p.add_argument("folder")
    p.set_defaults(func=cmd_verify_release)

    p = sub.add_parser("demo", help="synthetic end-to-end demonstration")
    p.add_argument("--out", default="demo")
    p.set_defaults(func=cmd_demo)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
