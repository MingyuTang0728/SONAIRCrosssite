# SONAIR — next steps after the bracket is mounted

Written against the review recording with Sam, the Experimental Deployment Plan,
and the OMAIB proposal. It covers what to build next, in what order, and why each
piece exists. Where the recording and an earlier plan disagree, the recording wins.

---

## 1. The one thing to get right

Sam said it three times, in three different ways:

> "But that's not a benchmark." … "You need to do a benchmark." … "Don't extend."

The deliverable is **not** a platform that shows sensor outputs, and it is not an
inspection demo. It is a **scored sim-to-real benchmark**: a metric matrix held
privately, external teams submitting models, those models scored on how well they
move simulation data to real data, with published examples and a stated methodology —
the shape of BenchCAD or GPQA Diamond.

Everything else is an *application case* that sits beside the benchmark. Sam said
that explicitly too: *"Maybe it's an application case instead of the actual benchmark.
Maybe that's the way you present it."*

Your contributions, in his words and in his order:

1. **The benchmark.** This is the main one.
2. **A basic AI model** that solves it. *"It doesn't have to be a good one."*
3. **One application domain** to apply it to — inspection.

---

## 2. Answers to the questions you asked

### 2.1 Will the D435i work over USB on your platform right now?

**Yes, for RGB / depth / IR — that path already exists and needs no new code.**
`multimodal_bridge.py` already opens the RealSense pipeline, and
`Remote_control.html` already renders all four streams. Three practical conditions:

- Use a **USB 3.0 port and the cable that came with the camera**. On USB 2.0 the
  D435i enumerates but silently drops to a reduced stream set, and the failure looks
  like a bandwidth bug rather than a cable problem.
- The bridge runs on the **workstation wired to the UR5e**, not on your laptop. The
  browser gets frames over the WebSocket. That is the existing topology and it is
  the right one — you do not want image data crossing the relay.
- `pyrealsense2`, `opencv-python` and `numpy` must be installed on that machine, or
  the agent boots with vision disabled and prints a warning.

**What was missing, and is now added: the D435i's own IMU.** The camera contains a
BMI055 — accelerometer and gyroscope, on `rs.stream.accel` and `rs.stream.gyro`. You
already own an inertial unit and it is wired up the moment the USB cable is in.
`bench_agent.D435iImuSource` opens it on **its own pipeline**, deliberately separate
from the image pipeline: the motion streams run at 200 Hz and 63 Hz against the
images' 30 Hz, and forcing them into one `wait_for_frames()` throttles the IMU to the
frame rate, which destroys the only property that made it worth logging.

It is a consumer-grade part and it is **not** a substitute for the industrial unit.
Use it as the third tier and as a sanity channel — but it means you can begin Phase 0
today, with no FusionHub and no new hardware.

### 2.2 FusionHub — direct read, or record alongside?

**Both, and the schema is identical either way**, which is the point.

- **Live path (preferred):** configure FusionHub to stream JSON over UDP to the
  workstation. `bench_agent.FusionHubBridge` listens on port 5005 and pushes samples
  into the hub. Nothing else changes.
- **Replay path (start here if the live stream fights you):** record in FusionHub,
  export CSV or JSONL, and read it with `sonair_benchmark.imu.read_fusionhub_file`.
  It accepts the field spellings FusionHub actually emits rather than demanding one.

Start on the replay path. It takes live-integration risk off the critical path, and
because both paths produce the same canonical record, **nothing downstream has to
change when you switch over.**

One thing that is *not* optional either way: FusionHub timestamps in **its own
clock**. `parse_fusionhub_row` returns that timestamp unconverted, on purpose — it
refuses to pretend the two clocks agree. Measure the offset with the tap test, then
apply it. A temporal misalignment that is assumed away reappears downstream as a
position error and gets attributed to the sim-to-real gap it is not.

> Sam, on the laptop failure: *"This is trash mag… you have to use a PC."*
> He was right, and it is worth reproducing the failure once on the desktop before
> concluding anything about the sensors themselves.

### 2.3 What is the IMU data actually for, and does it match what Sam wants?

**It matches exactly.** Asked what makes a good ground truth, Sam answered:

> "And the IMU? Yeah, perfect, perfect. So orientation. And everything the IMU gives
> you. So your orientation plus your change of orientation plus your acceleration of
> orientation. So all of those temporal fields of orientation. Easy ground truth."

And on what to avoid — after you described the probe lift-off idea:

> "Can you simulate the defect and the response of that probe to the defect?"
> "No, it's very hard." — "So therefore how would you make a sim to real gap with that?"

That is the selection rule for the whole project: **a modality earns its place only
if it is both cheap to ground-truth on real hardware and faithful to generate in
simulation.** Orientation, angular rate, acceleration and position pass. Eddy current,
ultrasonic, thermography and Raman fail — not because they are uninteresting, but
because nobody can simulate them well enough for the difference to mean anything.
They belong to your PhD, which Sam was explicit is a separate track.

So the IMU is used for three things:

1. **The orientation modality of the benchmark** — geodesic attitude error between
   simulated and real, per sample, reported at median and p95.
2. **The noise model that Isaac needs** (Phase 0 → `SimContract`). An *ideal*
   simulated IMU makes the gap look larger than it is, for a reason that has nothing
   to do with dynamics. You would be measuring "Isaac has no sensor noise", which
   nobody needs a benchmark to discover.
3. **The temporal alignment check** — the tap test that produces the timing row of
   the error budget.

Note the multimodal angle Sam raised at the end, worth keeping in the proposal:
*"Can the IMU sense a magnetic field to then self-calibrate a magnetic tracker?"*
That is a genuine cross-modal research question and it fits the OMAIB framing.

### 2.4 Isaac Sim — what exactly is the sim-to-real link?

The link is a **contract**, not a vibe. `sonair_benchmark.isaac.SimContract` holds it,
and it is written next to the sim dataset and checked on import, so a run generated
under a different contract is refused rather than silently pooled.

Four things must match. Three are easy and the fourth is the one everybody skips:

| # | Must match | Where it comes from | Failure if you skip it |
|---|---|---|---|
| 1 | Same commanded trajectory, units, command rate | `Export scan path for Isaac` button, feeding the planner's own waypoint list | You measure a different trajectory, not a gap |
| 2 | Carrier mass and centre of mass at the wrist | **Measured** in Phase 1 — weigh the printed bracket | Wrist dynamics differ; gap is inflated by payload error |
| 3 | Sensor rate and mounting offsets | **Measured** in Phase 2, not read off the CAD | A lever-arm error reads as an orientation gap |
| 4 | Simulated sensors degraded with Phase 0 noise and bias | `SensorDegrader` | You measure "Isaac has no sensor noise" |

On point 2 and 3: the plan says *"Print the carrier, weigh it, and measure the
mounting offsets rather than taking them from the CAD… they will not match the
drawing."* Your bracket is printed and mounted — **weigh it this week** and measure
the offsets. Those two numbers are inputs to both Phase 2 and Phase 4.

Practical note on bias, in `SensorDegrader`: bias is drawn **once per run and held**,
noise is drawn **per sample**. A per-sample bias would average out over a run, and the
resulting sim data would be unrealistically well behaved — which flatters any model
later scored against it.

`sonair_benchmark.isaac.write_isaac_stub()` emits a replay script skeleton that names
the four contract points and the exact log format, and leaves your USD scene wiring
alone.

---

## 3. Is your camera → scan → defect → fusion plan feasible?

**Technically yes — most of it already runs.** The existing console already does
vision capture, two-point pixel→physical calibration, ROI selection, zig-zag path
generation, and chunked URScript execution.

**But it is not the benchmark, and presenting it as one is the trap Sam warned about
twice.** Here is the honest split:

| Your step | Verdict | Where it belongs |
|---|---|---|
| Camera locates the part | Works today | Application case |
| Auto scan-path planning | Works today | **Both** — it generates the commanded trajectory the benchmark replays |
| Find suspected defects | Works, but is not simulatable | Application case / PhD |
| Sensor fusion when sensors arrive | Later | PhD, mostly |

The path planner is the genuinely dual-use piece, and that is why the Isaac export
button hangs off it. Use the scan path as the **trajectory generator** for the Phase 3
sweep: the stop-start profile is the one closest to real inspection scanning, which is
what makes the benchmark's conditions relevant rather than arbitrary.

One more reason not to lead with defect detection: Sam's own framing of the value.

> "So in that case, we transfer the simulation challenge to the universal robot
> itself. And it will be much easier… That could be the paper."

And the industrial anchor from the Karl conversation: rotor magnet slices are 3–4 mm
thick, so **a position error above 1 mm is a real failure**. That is the number that
makes a position-accuracy benchmark matter to someone. Put it in the paper's
introduction.

---

## 4. Where the results live

Two pages, two audiences. Do not merge them.

**`Remote_control.html` — acquisition.** The robot, the sensors, the 3D scene and now
the benchmark recorder all live here. This is where you *operate*. The new panel
(bottom of the sensor column) gives you live inertial channels, the tap-alignment
check, the run manifest form, record/stop, and the Isaac command export.

**`benchmark.html` — the deliverable.** This is the page Sam was pointing at when he
opened BenchCAD and GPQA Diamond on screen: leaderboard, task and I/O specification,
the gap map, the measurement floor, and how to submit. It reads `leaderboard.json`
and `gap.json` straight from the harness, so it is never hand-maintained.

Both are linked from `index.html`.

> *"It's kind of like a data visualization." "Yeah. All the models and their progress
> towards solving your PhD set of questions."*

---

## 5. The benchmark design

### The task

Given an Isaac-generated simulated sequence and the commanded trajectory, **predict
the real UR5e sequence**. Submissions may predict absolute values or the correction;
the harness accepts both.

### The score

```
GCR = 1 - err(prediction, real) / err(simulation, real)
```

Reported at the **median** and the **95th percentile**. The p95 figure is the headline,
and that choice is the whole design:

> Sam: *"The long tail problem is when you have very small examples of training data,
> but very safety critical consequences… it's those rare events that are the most
> critical."*

A benchmark scored on means will be closed by a model that matches means. Scoring at
p95 means a model that matches the centre of the error distribution but not its tails
**scores well on GCR-median and badly on GCR-p95** — and that visible split is the
thing the benchmark exists to measure. A Wasserstein distance between the predicted
and real error *distributions* backs it up.

### The held-out set

**Whole condition cells, not random samples.** Interpolating inside a condition you
have seen is easy. The unsolved problem is generalising *across* conditions — to an
elbow velocity or arm configuration you were never shown.

### The sweep

Four factors (`sonair_benchmark.campaign`):

- **Elbow velocity** 0.2 / 0.4 / 0.5 / 0.6 / 0.7 / 0.9 rad/s — bracketing the ~0.6 rad/s
  region where the simulator is *already known* to change behaviour. Sampling either
  side of a known divergence turns a bug into the structure the benchmark measures.
- **Arm configuration** near-singular / mid-workspace / extended.
- **Trajectory type** point-to-point / contour / stop-start.
- **Repeats**, spread across sessions days apart, with one deliberate carrier refit —
  so refit error is *measured* rather than assumed away.

Default plan: 270 runs, 54 cells, ~1.9 hours of arm time, ~0.2 GB raw.

### The gates

| Gate | When | Asks | If it fails |
|---|---|---|---|
| **A** | End of Phase 0 | Do the IMUs and trackers produce usable data? | It is a driver problem, not a sensing one. No purchase justified yet. |
| **B** | End of Phase 2 | Is the gap large compared with the calibration residual? | The benchmark is measuring itself. Publish as an upper bound, or stop. |
| **C** | End of Phase 5 | Does the gap vary systematically with condition? | No signal to learn. Widen the sweep before publishing. |

Gate B and Gate C are implemented (`budget.gate_b`, `metrics.gate_c`) and run
automatically — a result cannot be published without its own floor attached.

### The baselines

Run both before publication. They set the floor the leaderboard starts from:

1. **Identity** — hand back the simulation unchanged. Scores 0 by construction.
2. **Constant offset** — one global XYZ offset fitted on the example set. It is the
   dumbest thing that could possibly work, and **a model that does not beat it has not
   demonstrated anything.**

Both go through the same code path as every submission. A benchmark whose baseline is
computed separately will eventually disagree with itself.

---

## 6. What to do, in order

### This week — Phase 0, costs nothing, starts now

1. **Weigh the printed bracket** and measure the IMU mounting offsets. Two numbers,
   ten minutes, and they feed both Phase 2 and Phase 4.
2. **Plug the D435i into the workstation** and confirm its built-in IMU streams:
   start the bridge and watch the inertial panel populate.
3. **Move the industrial IMU to the desktop PC** and reproduce the laptop failure
   once before blaming the sensor.
4. **One hour stationary log**, unit clamped, surface isolated from foot traffic:
   ```bash
   python -m sonair_benchmark phase0 --fusionhub stationary.csv --expected-hz 200 \
       --out phase0/ind0.json
   ```
   This gives the noise floor every later error claim is measured against. Without it
   there is no way to tell a real sim-to-real difference from sensor noise.
5. **Six-position tumble test** using gravity as reference (`imu.tumble_check`).
6. **Tap test** — tap the carrier once, firmly, press *Verify tap alignment*. Record
   the spread.

> Sam: *"Your IMUs are your first bet."*

### Week 2

7. **Assemble the error budget:**
   ```bash
   python -m sonair_benchmark budget --phase0 phase0/ind0.json \
       --tap-spread-ms 0.8 --tracker-mm 1.2 --frame-fit-mm 0.8 \
       --calib-version calib-1 --out calib/budget.json
   ```
8. **Put the open decisions to Sam** — the third modality (force is the proposal),
   and whether the acoustic group counts for SONAIR. They block the carrier design.
9. **Draw the PowerPoint block diagram.** Sam asked for this directly and twice:
   *"Not AI, like PowerPoint with shapes… where it's SIM, GAP"* — showing the
   UoN/UCL split and both UR arms. It is independent of everything else.

### Weeks 3–6 — Phases 3 and 4, in parallel

10. `python -m sonair_benchmark plan --out campaign/plan.json`
11. Run the sweep. Each run: fill the manifest form, press *Record run*.
12. For each run, *Export scan path for Isaac* and generate the simulated counterpart.
13. **Ask Tianyi for a Master's student** to do the data collection. Sam pushed this
    hard and was right: *"That's what I used to do with my PhD."* Thirty hours of
    someone else's time, and they get a project out of it.

### Weeks 7–9 — Phases 5 and 6

14. `python -m sonair_benchmark gap …` → Gates B and C.
15. `python -m sonair_benchmark score …` → leaderboard, read by `benchmark.html`.
16. Write the paper: benchmark as contribution 1, a basic model as contribution 2,
    inspection as the application domain.

### Before the October review

Bring the **Phase 0 numbers**, the **error budget**, the **block diagram**, and
`benchmark.html` running on demo data. The deployment plan puts it well: those numbers
are what make the review substantive rather than a status update.

---

## 7. Try the whole thing before you have any real data

```bash
python -m sonair_benchmark demo --out demo/
```

Generates a synthetic real/sim dataset with a **known** velocity-dependent gap — flat
below 0.6 rad/s, diverging sharply above, with heavier tails at high velocity — then
runs budget → gap → gates → baselines → leaderboard over it.

Open `benchmark.html` next to `demo/site/` and `demo/results/` and the whole page
populates. This proves the toolchain before a single real sample is recorded, and it
is the fastest way to show Sam what the benchmark will look like.

On the demo data the "50% oracle" entry scores **GCR-p95 below GCR-median** — it
closes the bulk of the error and leaves the outliers intact. That split is the
benchmark working as designed.

---

## 8. Open decisions that block work

| Decision | Blocks | Note |
|---|---|---|
| The third modality | Carrier design, Phase 4 sensor models | Force is the proposal, at no extra cost. Must settle before the carrier is finalised |
| Submission I/O specification | Phase 6, and the Phase 3 schema | Draft is in `scoring.py` and on the benchmark page. Fix it early — the fields a submission needs are fields the campaign has to record |
| Whether UCL data joins the same benchmark | Phase 6 packaging | Decides whether the spec must accommodate a second platform from the outset |
| Electromagnetic trackers | Position modality | Sam: they are useless in a magnetic field, and *"don't touch any EM models. Not this year."* Verify the arm's own distortion in Phase 0 before relying on them |

---

## 9. Files

```
sonair_benchmark/
  schema.py     canonical run format (JSONL, manifest on line 0)
  clock.py      time master, offset fitting, tap detection, resampling
  imu.py        FusionHub live + replay, noise floor, tumble, rate stability
  budget.py     the five-row error budget and Gate B
  campaign.py   the Phase 3 sweep and whole-cell holdout split
  isaac.py      SimContract, sensor degradation, Isaac replay stub
  metrics.py    position / orientation / temporal / distributional + Gate C
  scoring.py    submission interface, GCR, baselines, leaderboard
  demo.py       synthetic end-to-end proof
  cli.py        python -m sonair_benchmark …

bench_agent.py       host-side: IMU ingestion, time master, run recorder
multimodal_bridge.py existing agent, now wired to bench_agent
benchmark.html       the public benchmark page
Remote_control.html  existing console, now with the acquisition panel
```

The benchmark package is pure standard library. That is deliberate: a reviewer has to
be able to re-score your leaderboard on their own machine without an RTX card or a
conda environment.
