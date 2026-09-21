# sonair_benchmark

Toolchain for SONAIR — Sim2real Operational beNchmark for AI Robotics.

Pure standard library, no dependencies. A reviewer must be able to re-score the
leaderboard on their own machine without an RTX card or a conda environment.

## Quick start

```bash
# see the whole pipeline on synthetic data with a known injected gap
python -m sonair_benchmark demo --out demo/

# then open benchmark.html next to demo/site/ and demo/results/
```

## The pipeline

```
Phase 0   phase0    IMU noise floor, bias, rate stability          -> phase0/ind0.json
Phase 2   budget    the five-row error budget (the floor)          -> calib/budget.json
Phase 3   plan      the condition sweep and held-out cell split    -> campaign/plan.json
          (acquire real runs via bench_agent / Remote_control.html)
Phase 4   isaac     SimContract -> Isaac replay -> canonical runs
Phase 5   gap       the gap map, Gate B and Gate C                 -> results/gap.json
Phase 6   score     submissions, baselines, leaderboard            -> site/leaderboard.json
```

## Run format

One run is one JSONL file. Line 0 is the manifest, every later line is a sample.
A run killed halfway is still valid up to where it died.

```json
{"_manifest": {"run_id": "...", "side": "real", "calib_version": "calib-1",
               "joint_vel": 0.7, "arm_config": "extended",
               "traj_type": "stop_start", "repeat_idx": 2}}
{"t": 0.000, "q": [...], "tcp_pos": [x,y,z], "tcp_rot": [rx,ry,rz],
 "imu": {"ind0": {"gyro": [...], "accel": [...]}}}
```

`calib_version` is mandatory and validated on load. Runs recorded either side of a
recalibration cannot safely be pooled, and without the field there is no way to find
out afterwards which side a run fell on.

## Submission format

One predicted run per line:

```json
{"run_id": "...", "mode": "absolute",
 "t": [...], "tcp_pos": [[x,y,z], ...], "tcp_rot": [[rx,ry,rz], ...]}
```

`mode` is `"absolute"` (predict the real sequence) or `"correction"` (predict the
delta; the harness adds it to the simulation).

## The score

```
GCR = 1 - err(prediction, real) / err(simulation, real)
```

Reported at the median and the 95th percentile. **p95 is the headline.** A model that
matches the centre of the error distribution but not its tails scores well at the
median and badly at p95, and that split is what the benchmark exists to expose.

## Design notes worth knowing before you change anything

- **Error budget rows combine in root-sum-square, not arithmetic sum.** They are
  independent measurements; summing them overstates the floor and lets a real gap be
  dismissed as noise.
- **Quaternion geodesic distance takes `abs()` of the dot product.** `q` and `-q` are
  the same rotation; without it half the samples read as ~180° errors that are not.
- **Runs are resampled onto a common time base before differencing.** Isaac's step
  cadence and the UR's 125 Hz loop do not line up, and differencing them raw
  manufactures error that looks exactly like a velocity-dependent gap.
- **Simulated sensor bias is drawn once per run and held; noise is per sample.** A
  per-sample bias averages out and produces unrealistically well-behaved sim data,
  which flatters any model later scored against it.
- **Holdout is by whole cell, not random sample.** Random holdout tests interpolation
  within a condition, which is easy. Whole-cell holdout tests generalisation across
  conditions, which is the unsolved part.
- **Baselines go through the same code path as submissions.** A benchmark whose
  baseline is computed separately will eventually disagree with itself.

See `docs/SONAIR_Next_Steps.md` for the full plan and the rationale behind the
modality choice.
