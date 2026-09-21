"""
Phase 3 — the condition sweep.

The gap is not a single number, it is a function of the operating condition,
so the campaign is a sweep rather than a recording session. Four factors:

  joint_vel    commanded elbow angular velocity, bracketing the region where
               the simulator is ALREADY known to change behaviour (~0.6 rad/s).
               Sampling either side of a known divergence turns a bug into the
               structure the benchmark measures.
  arm_config   near-singular / mid-workspace / extended. Dynamic error is
               configuration dependent; one configuration would give a
               benchmark that generalises to nothing.
  traj_type    point-to-point / contour / stop-start. Transients and steady
               motion fail differently, and stop-start is closest to real
               inspection scanning.
  repeat_idx   the same cell repeated across sessions, days apart, with at
               least one carrier refit. Without repeats a reviewer cannot tell
               drift and refit error from the gap.

`plan_campaign` emits the full run list with ids, so the same plan drives the
real acquisition and the Isaac generation and the two cannot silently diverge.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path

from .schema import ARM_CONFIGS, TRAJ_TYPES, RunManifest

# Either side of the known ~0.6 rad/s divergence.
DEFAULT_VELOCITIES = (0.2, 0.4, 0.5, 0.6, 0.7, 0.9)
DEFAULT_REPEATS = 5


@dataclass
class PlannedRun:
    run_id: str
    joint_vel: float
    arm_config: str
    traj_type: str
    repeat_idx: int
    session: int = 0          # which physical session; repeats span sessions
    refit_before: bool = False  # deliberate carrier refit, so refit error is measured

    def cell_key(self) -> str:
        return f"{self.joint_vel:.3f}|{self.arm_config}|{self.traj_type}"

    def manifest(self, side: str, calib_version: str, **kw) -> RunManifest:
        return RunManifest(
            run_id=self.run_id, side=side, calib_version=calib_version,
            joint_vel=self.joint_vel, arm_config=self.arm_config,
            traj_type=self.traj_type, repeat_idx=self.repeat_idx, **kw)


def plan_campaign(velocities=DEFAULT_VELOCITIES,
                  configs=ARM_CONFIGS,
                  traj_types=TRAJ_TYPES,
                  repeats: int = DEFAULT_REPEATS,
                  sessions: int = 3) -> list[PlannedRun]:
    """
    The full run list.

    Repeats are spread ACROSS sessions rather than run back to back, because
    back-to-back repeats measure short-term noise and the thing that needs
    measuring is whether the gap survives a day, a thermal cycle and a refit.
    """
    runs: list[PlannedRun] = []
    for v in velocities:
        for cfg in configs:
            for tt in traj_types:
                for r in range(repeats):
                    session = r % max(1, sessions)
                    runs.append(PlannedRun(
                        run_id=f"v{v:.2f}_{cfg}_{tt}_r{r:02d}".replace(".", "p"),
                        joint_vel=v, arm_config=cfg, traj_type=tt,
                        repeat_idx=r, session=session,
                        # One deliberate refit partway through the repeats.
                        refit_before=(r == max(1, repeats // 2)),
                    ))
    return runs


def campaign_size(runs: list[PlannedRun], seconds_per_run: float = 25.0,
                  sample_rate_hz: float = 125.0) -> dict:
    """Sanity figures before committing four weeks of arm time."""
    n = len(runs)
    cells = len({r.cell_key() for r in runs})
    samples = n * seconds_per_run * sample_rate_hz
    # ~200 bytes per JSON sample row with all channels fitted
    return {
        "n_runs": n,
        "n_cells": cells,
        "runs_per_cell": n / cells if cells else 0,
        "total_samples": int(samples),
        "est_raw_gb": samples * 200 / 1e9,
        "est_arm_hours": n * seconds_per_run / 3600.0,
        "n_sessions": len({r.session for r in runs}),
    }


def split_cells(runs: list[PlannedRun], holdout_frac: float = 0.3,
                seed: int = 0) -> tuple[set[str], set[str]]:
    """
    Split by WHOLE CELL into (published_example_cells, held_out_cells).

    Cells are held out by stratifying on velocity so that the held-out set
    spans the divergence region rather than accidentally landing entirely on
    one side of it — which would make the benchmark trivially easy or
    trivially impossible, and in neither case informative.
    """
    import random

    rng = random.Random(seed)
    by_vel: dict[float, list[str]] = {}
    for r in runs:
        by_vel.setdefault(r.joint_vel, []).append(r.cell_key())

    held: set[str] = set()
    published: set[str] = set()
    for vel, keys in sorted(by_vel.items()):
        uniq = sorted(set(keys))
        rng.shuffle(uniq)
        k = max(1, int(round(len(uniq) * holdout_frac)))
        held.update(uniq[:k])
        published.update(uniq[k:])
    return published, held


def save_plan(path: str | Path, runs: list[PlannedRun], **meta) -> None:
    doc = {
        "meta": {"n_runs": len(runs), **meta},
        "runs": [asdict(r) for r in runs],
    }
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(doc, indent=2), encoding="utf-8")


def load_plan(path: str | Path) -> list[PlannedRun]:
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    return [PlannedRun(**r) for r in doc["runs"]]
