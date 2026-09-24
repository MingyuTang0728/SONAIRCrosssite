# From the first IMU recording to a sim-to-real number

Everything below already exists in `sonair_benchmark/` and runs on stdlib
alone — a reviewer can re-score the whole leaderboard without a GPU. What is
missing is only your data.

The whole chain, verified end to end on synthetic data:

```
python -m sonair_benchmark demo --out .
  60 paired runs, 20 cells
  overall median gap: 1.739 mm   floor: 1.127 mm
  Gate B: MARGINAL (ratio 1.54)
  Gate C: PASS (structure ratio 293.96)
```

That is the shape of the answer you are working toward. Your job is to replace
the synthetic runs with real ones and Isaac ones.

---

## Step 1 — characterise the sensor before recording anything with it

One hour of the IMU lying perfectly still, then:

```
python -m sonair_benchmark phase0 --fusionhub <run>.jsonl --expected-hz 300 \
       --out calib/ind0.json
```

This gives the gyroscope bias, the noise on each axis, and how stable the
sample rate is. **Every later error claim rests on these numbers**, and they
are the reason a 2 mm gap can be called a measurement rather than a guess: a
gap smaller than the measurement floor is not a gap, it is your own noise.

The console's **Sensors** page computes the same bias live and shows it as
"drift correction" — that is the sanity check, not the record.

## Step 2 — the error budget, and the floor it implies

```
python -m sonair_benchmark budget --phase0 calib/ind0.json \
       --handeye calibration/handeye.json --out calib/budget.json
```

The hand-eye residual the console reports goes in here. So does the timing
spread from "Check sensor timing" on the Record page. They combine in
root-sum-square into one number: the smallest gap this rig can honestly
resolve. **Gate B is the test that your measurement floor is well below the
gap you are trying to measure** — if it is not, stop and improve the rig,
because nothing downstream can be trusted.

## Step 3 — plan the campaign, then record it

```
python -m sonair_benchmark plan --out campaign/
```

This emits the condition sweep: joint velocities, arm configurations,
trajectory types, repeats. It is stratified and it reserves **whole cells** as
a holdout — not a random sample of rows. A model that has seen every condition
and is tested on held-out rows is being asked to interpolate; held-out cells
ask it to generalise, which is the actual claim.

Then record each cell from the console's **Record** page. Put the calibration
version from the Calibrate page in the box — the recorder refuses without one,
because runs either side of a recalibration cannot be compared and this is the
only record of which side a run came from.

## Step 4 — give Isaac the same commands, not the same outcome

```python
from sonair_benchmark.isaac import SimContract, export_commands, write_isaac_stub

contract = SimContract.from_measurements("handeye-20260924T…", noise_floor, budget)
contract.save("calib/contract.json")
export_commands(planned_run, waypoints, "sim/run_0001.commands.json", contract)
write_isaac_stub("sim/isaac_replay.py")
```

**This is where sim-to-real is won or lost.** Isaac must be given the
*commanded* trajectory — the same waypoints, the same units, the same rate —
and must not be given the real robot's measured result. If the simulation is
fed what actually happened it will reproduce what actually happened, and the
gap you measure will be zero for reasons that mean nothing.

The contract fixes the four things that otherwise drift apart:

1. the commanded trajectory, its units and its rate;
2. the carrier's **measured** mass and centre of mass, not the CAD value;
3. the sensor's rate and mounting offset — the hand-eye transform you just
   solved;
4. noise and bias applied **outside** Isaac, by `import_isaac_run(degrade=True)`.

Point 4 matters more than it looks. Isaac renders an ideal sensor. If you let
it also invent the noise, the noise becomes part of the thing being scored and
a submission can win by modelling your random number generator. Applying the
measured noise floor afterwards, from the Phase 0 file, keeps it a property of
the hardware.

Run the stub inside Isaac (`./python.sh isaac_replay.py run_0001.commands.json`).
It logs one JSON object per line in the same schema the real recorder writes,
so nothing downstream has to know which side a run came from.

## Step 5 — measure the gap

```
python -m sonair_benchmark gap --real data/real --sim data/sim --out results/
```

Runs are paired by run id and differenced on the modalities that are both
cheap to ground-truth and faithful to simulate: orientation, angular rate,
acceleration, position. It reports at the **median and the p95**, and the p95
is the headline — the long tail is where a benchmark earns its keep, and a
median hides exactly the rare conditions the inspection case cares about.

Gate C then checks the gap has *structure*: that it varies across conditions
by much more than it varies within one. A gap that is the same everywhere is
usually a constant offset — a calibration error, not a sim-to-real gap — and
there is nothing for a model to learn from it.

## Step 6 — the leaderboard

```
python -m sonair_benchmark score --submissions subs/ --out site/
```

Scored on the **Gap Closure Ratio**: `1 − err(prediction, real) / err(simulation, real)`.
Zero means the submission did not improve on raw simulation. One means it
predicted the real robot exactly. Two baselines ship with it and both are
meant to be beaten: *identity* (change nothing) and *constant offset*. In the
demo the constant-offset baseline scores **−0.317 at p95** — worse than doing
nothing — which is the point: a trick that helps on average hurts in the tail,
and the tail is what is being scored.

---

## What to do this week

1. One hour of the IMU stationary → `phase0`. You cannot claim any accuracy
   before this exists.
2. Hand-eye calibration on the console, saved, and its version noted.
3. Both into `budget` → read the floor. If the floor is above about 2 mm, fix
   the rig before recording a campaign.
4. Record **two** runs of the same condition. Difference them against each
   other, not against simulation. That is your repeatability, and it is the
   real floor — if two real runs differ by more than your claimed gap, the
   gap is not measurable yet.

Only then is Isaac worth wiring up. Step 4 is the one people skip, and it is
the one that decides whether the number at the end means anything.
