# The arc scan of 29 September, and what it exposed

The path was right. The recording was not. A two-minute arc scan came back with
robot state that was fresh in **16% of its samples**, and nothing anywhere said
so while it was happening.

## What the files show

Measured from the uploaded capture, not from the logs:

| | |
|---|---|
| samples in the run file | 11 127 over 121.9 s (91.3 Hz of a declared 125) |
| rows repeating the previous row's robot state | **9 767 — 87.8%** |
| distinct robot readings in the whole file | **1 360** |
| worst joint jump when the link caught up | **52.0°** |
| `source` on every one of 9 313 robot-log rows | **`primary-30003`** |
| gaps in the robot stream | 13 of them, **13.0 – 14.5 s each** |

The cause was a single fact that no part of the console reported: **RTDE never
started, and the robot was being read over the fallback interface on port
30003.**

## Why it failed the way it did

Three faults compounded, and each is fixed:

**1. Asking for more fields could kill the link.** The RTDE recipe was extended
from 30 fields to 44. `setup_outputs` built its unpack format straight from the
types the controller granted, so a single type the parser did not recognise
raised `KeyError`, RTDE was abandoned, and the console silently fell back. An
unfamiliar type now costs **that field**, not the stream — and if the controller
grants nothing at all, that is an error rather than a silent downgrade.

**2. A working stream was punished for a hiccup.** The reconnect backoff
doubled to ten seconds on every failure regardless of whether the stream had
been delivering. Combined with a full RTDE connect-and-refuse on each cycle,
that is the 13–14 s hole: RTDE attempt, primary runs ~5 s, drop, 10 s backoff,
repeat — a 19-second period, which is exactly what the data shows. A stream that
was delivering now reconnects in 0.2 s, and RTDE is retried on its own slower
schedule instead of costing seconds on every reconnect.

**3. Nothing said which interface was in use.** The dataset manifest recorded
the robot as the string `"UR5e"` and a host address. The console showed a
healthy link. Pre-flight passed.

## What now makes this impossible to repeat

**Pre-flight blocks it.** "Connected" was never the right question — *which
interface* is. A job that needs the robot is now refused while it is on the
fallback, naming the reason RTDE gave:

> The robot is being read over the FALLBACK interface on port 30003, not RTDE on
> 30004, because RTDE would not start: *«the controller's own words»*. RTDE is
> the only interface that carries the full field set at a steady rate… Check
> that nothing else holds port 30004.

**Every sample carries its own age.** `robot_age_s` is how old the robot reading
was when that row was written — near zero on a healthy link. A file of repeats
is now self-evident in the file.

**The export audit measures it.** Running the new audit over the bad capture
reproduces the analysis from the file alone:

```
"repeated_robot_state_frac": 0.8778,
"distinct_robot_states": 1360,
"worst_joint_jump_deg": 52.04,
"notes": ["88% of this run's rows repeat the previous row's robot state, and
          when it does change the joints jump by up to 52 degrees. The robot
          link was stalling: this file holds 1360 real robot readings wearing
          the shape of 11127 samples. Do not measure a gap against it …"]
```

**The manifest records the link** — interface, rate, field count, controller
version, reconnects, and the reason RTDE was unavailable.

## The accelerometer needs six faces

Separately, and confirmed from the same capture: on samples where the carrier
was demonstrably still, **|a| ranged 8.31 – 10.09 m/s², an 18% spread**, and it
tracked attitude — about 10.00 m/s² with the tool pitched near −15°, about 9.43
near +60°. That is per-axis bias and scale.

It matters more than a raw accuracy figure would suggest: gravity is removed
using the orientation, so a scale error leaks gravity into `linear_accel` **as a
function of pose**, which is indistinguishable from a pose-dependent sim-to-real
gap and survives every average.

**It cannot be fitted from run data.** That shortcut was tried on this scan and
returns nothing usable: across the whole path the sensor's Y axis saw gravity
only between −0.54 and +0.99 m/s², because the tool never rolls far that way. An
axis never presented to gravity carries no information about its own scale, and
the solution for it divides by approximately zero.

So `accel_cal` solves it from **six static faces** — the minimum set that
constrains all six unknowns, each axis seeing +1 g and −1 g. Rest the carrier on
each of its six sides for a few seconds. The model is per-axis bias and scale
only; six poses constrain six parameters and no more, and solving for nine would
invent three.

Meanwhile the console *measures the symptom continuously* from whatever the unit
is already doing, and pre-flight warns when one g varies by more than 3% — so
this is something the operator is told, not something found in the analysis
afterwards.

## Still open

- **The inertial log arrives in bursts**: dt median 0.16 ms with 19 gaps over
  0.5 s and a worst of 2.79 s. No samples are lost — the count and the average
  rate are right — but per-sample spacing carries the transport's jitter, which
  the dataset manifest already states per channel.
- **Nominal kinematics vs the controller** differ by 1.2–1.5 mm and ~0.2°. That
  is a floor on position error for any simulator using nominal DH parameters,
  and belongs in the error budget rather than in the gap.
