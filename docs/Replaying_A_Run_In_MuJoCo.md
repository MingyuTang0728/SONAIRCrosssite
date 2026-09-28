# Watching a run again in MuJoCo

There is nothing to export. **The run file is the input.**

Every recorded run already carries the thing a simulator needs: `target_q`, the
UR controller's own joint setpoint at every sample. That is what the real arm
was *told* to do, and handing it to a simulator asks the simulator the only
fair question — *given the same command, where does your arm end up?*

The measured trajectory is deliberately **not** what is fed in. Feed a simulator
the joint angles the real arm actually reached and it will reach them too, and
the gap comes out as zero by construction. That is why the recorder refuses to
start at all when `target_q` is not arriving.

## One command

```bat
python sim_mujoco.py ^
  --real bench_runs\single_run_20260928_171029_i0_r0.jsonl ^
  --out  sim_runs ^
  --menagerie mujoco_menagerie ^
  --view
```

`--real` takes a single `.jsonl` or a folder of them. `--view` opens the MuJoCo
window and plays the simulated arm at the speed the real one moved; `--speed 4`
runs it four times faster, `--speed 0.25` a quarter speed for a close look.

What is on screen is the **same simulation that gets written and scored**, not a
playback of the recording. The obvious way to add a picture would be a second
loop animating the recorded joint angles — and it would show a beautiful arm
tracing exactly the real trajectory, because it would be replaying the
measurement rather than simulating anything. Closing the window stops the
drawing and nothing else; the run is still written.

First time only, fetch the robot model:

```bat
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git
```

## Then the gap

```bat
python -m sonair_benchmark gap --real bench_runs --sim sim_runs
```

Run against the real `single_run` from 28 September this gives a median gap of
**7.3 mm** over one pair. One pair is a plumbing check, not a result — Gate C
needs whole condition cells, which is what a campaign is for.

## Two flags that change the number

**`--phase0 phase0\ind0.json`.** Without it the simulated sensors are perfect,
so the gap silently includes *"the simulator has no sensor noise"*, which is not
a property of the simulator. Record two minutes of the unit standing still, run
`python -m sonair_benchmark phase0 --fusionhub <csv>`, and pass the result here.

**`--tcp-offset x,y,z`** in metres, read off the pendant under
Installation → TCP. The robot reports its tool centre point; the model reports
the bare flange. If a tool offset is configured and not passed, the gap is that
offset — constant, large, and varying with nothing. The replay checks the two
frames at the first sample and refuses beyond 50 mm rather than producing a
number. The 28 September runs pass without it, so that cell's pendant offset is
zero.

The flange payload is **not** a flag you have to remember: it comes from the
run's own manifest, which is where the carrier you entered on the Automate page
is recorded. `--carrier-mass-kg` remains, as a deliberate override for asking
what-if, and it says so in the simulated run's notes when used.

## What comes out

A `.jsonl` per run under `--out`, the same shape as the real one — same
timestamps, same sample count, `side: "sim"` — so the two can be differenced
row for row with no resampling. Each holds the simulated joint angles and
velocities, the simulated tool pose, and a simulated IMU at the flange
(`quat`, `gyro`, `accel`, the last being *proper* acceleration, so it reads
9.81 at rest exactly as the real part does).

A note on rate: the pairing assumes both sides are on the same grid. A real run
that came up short of its declared rate says so in its dataset manifest under
`measured`; generate or resample the simulated side to the **achieved** rate,
not the declared one.
