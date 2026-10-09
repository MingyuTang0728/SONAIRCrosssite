# Taking part: Track A and Track B

The public page is `benchmark.html`. It reads `site/leaderboard.json`, written
by `python -m sonair_benchmark score`, and `results/gap.json`, written by
`… gap`. This file is the same contract in more detail.

## What is common to both tracks

The robot's controller commanded a joint trajectory, `target_q`, at 125 Hz.
The real arm executed it. The **reference simulation, S0**, executed it too:
the MuJoCo Menagerie UR5e, unchanged, replayed by `sim_mujoco.py`. Both
tracks are scored the same way, on the tool position, against the real run,
relative to S0:

    GCR = 1 − err(entry, real) / err(S0, real)

The score is reported at the median and at the 95th percentile, per condition
cell and overall. The headline is GCR-p95 on the held-out cells. Both tracks
go through the same code, so their numbers are directly comparable.

| Data | Use |
|---|---|
| E1 (identification) | training: every joint excited, two payloads. Never scored |
| E2, published cells | training and validation: real runs released |
| E2, held-out cells | scoring: commands and S0 released, real runs kept |
| E3 (out of distribution) | scoring only, hidden |

## Track A: a better simulator

**Option 1: send a MuJoCo model.** We replay every run through it with
exactly the harness that produces S0. The contract is checked when the
model loads:

* the six UR joints with their UR names, in UR order: `shoulder_pan_joint`,
  `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`,
  `wrist_3_joint`;
* six actuators whose `ctrl` is the commanded joint position;
* a site named `attachment_site` at the tool flange.

Everything else is yours: masses, inertias, friction, armature, damping,
gains and solver options. Our sensor block (IMU and tool pose at
`attachment_site`) is wrapped around your file, so every model is measured
the same way at the same point. Send the folder with its meshes. Paths inside
the XML are resolved relative to it.

```
python -m sonair_benchmark simulate --model my_ur5e/ur5e.xml \
    --real data/real --out results/my_model
python -m sonair_benchmark score --real data/real --sim data/sim_s0 \
    --budget calib/budget.json --plan campaign/plan.json \
    --track-a "My model=results/my_model" --out site/leaderboard.json
```

Each simulated run's notes name the model that produced it.

**Option 2: any other engine.** Generate one simulated run per real run from
its `target_q`, in the run-file format (`"side": "sim"`, same `run_id`, cell
and repeat), and pass the folder to `--track-a`.

**Live, on the real cell.** A Track A model can be scored while the cell
runs. In the console's Live arm column, type the model's path under *Live
benchmark* and press **Run beside S0** (or start the agent with
`SONAIR_TWIN_MODEL=path\to\ur5e.xml`). S0 and the candidate are then both
driven by the robot's own commanded joints, packet by packet, and:

* the table shows each model's tool-position error over the last 5 s, and the
  candidate's GCR against S0;
* every recorded run is scored the moment it ends, by the offline rules
  (median and p95 per run, GCR per run, mean per cell and overall), and the
  session is written to `results/live_score.json`;
* `benchmark.html`, served from the cell PC, shows the same thing under
  **Live**.

On two test runs the live GCR-p95 agreed with the offline harness to 0.01
(`tests/test_live_bench.py`). It is still a preview: the score of record is
the offline replay of each run file. Simulated-cell rehearsals are scored
too, and kept apart. **New session** starts the tally again.

## Track B: a correction

Send one prediction per run, as JSON Lines:

```
{"run_id": "...", "mode": "absolute",
 "t": [...], "tcp_pos": [[x,y,z], ...], "tcp_rot": [[rx,ry,rz], ...]}
```

* `"mode": "absolute"` predicts the real run.
* `"mode": "correction"` predicts the difference, which is added to S0.

Timestamps are those of the S0 simulation you were given.

```
python -m sonair_benchmark score ... --track-b "My model=predictions.jsonl"
```

## The baselines

| | Track | Status |
|---|---|---|
| S0: default MuJoCo, the reference (GCR 0) | A | live |
| S1: velocity feed-forward | A | next |
| S2: identified on E1 (CMA-ES) | A | planned |
| S3: Isaac Sim | A | planned |
| B1: one constant offset | B | live |
| B2: per-cell offset | B | planned |
| B3: residual network | B | planned |

## Trying it

```
python -m sonair_benchmark demo --out demo/
```

The demo builds the whole chain on synthetic data with a known, injected gap.
To see the page on it, copy `benchmark.html` into `demo/` and serve that
folder (`python -m http.server 8000` inside it).
