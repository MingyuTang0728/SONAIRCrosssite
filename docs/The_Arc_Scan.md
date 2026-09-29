# `arc_scan` — a minute of inspection worth recording

## Why the old scan carried nothing

`scan_shaped` traces a box in a plane with the tool at a fixed orientation. On
the real cell it was measured: **0.9° of rotation in 46 seconds**, and linear
acceleration flat on the sensor's own noise floor — 27 samples out of 8749
above 0.5 m/s².

Orientation and angular rate are two of the three channels the benchmark is
scored on. That run was not badly executed; there was nothing in it.

## Why this one does

Real parts are not flat, and real inspection does not hold a fixed orientation.
A probe, a camera at fixed standoff, an ultrasonic wheel — all of them are
carried **normal to the surface**. Sweep across a curved part and the tool
rotates by the part's own curvature, continuously, *for free*: it is what the
job requires, not a wiggle added to give the sensors something to look at.

So the path is generated from a **part**, not from a box. `scan_paths.arc_zigzag`
takes a radius, an arc and a length and returns the poses that keep the tool at
a fixed standoff, normal to the surface, zig-zagging over it.

| | |
|---|---|
| waypoints | 192 (12 passes × 16) |
| tool travel | 2.67 m at 45 mm/s |
| **duration** | **59.4 s** |
| **total rotation** | **840°** |
| **mean angular rate** | **14.15 °/s** |
| cell's measured gyro noise floor | 0.11 °/s |

14 °/s is over a hundred times the noise floor. Between consecutive waypoints
the tool turns 4.67°, smoothly, throughout every pass.

## It moves as one path

`trajectory` sends one `movel` per waypoint and waits for each to land within
two millimetres, so the arm comes to a **complete stop at every one**. For 192
waypoints that is wrong twice over: several times longer, and what the sensors
see is 192 start-stop transients instead of a sweep.

The `path` step sends the whole thing to the controller as a single blended
URScript program, so the arm never stops until the end. Every waypoint is
checked against the safe envelope **before any of it is sent** — a batch program
cannot be halted between points, so a path refused half-way through would leave
the arm somewhere nobody chose.

## What it writes

Three files, started and stopped together:

| File | What | Rate |
|---|---|---|
| `bench_runs/arc_scan_*.jsonl` | the benchmark run, on the fixed sample grid | 125 Hz |
| `imu_logs/imu_*.csv` | every inertial sample | ~190 Hz |
| `ur_logs/ur_*.csv` | **every packet the robot sent, 132 columns** | 125 Hz |

The robot CSV is the complete record, and it is the new one. Run files hold the
subset the gap is scored on; this holds everything the controller will part
with:

- joint angles, velocities, accelerations and their commanded counterparts
- joint **currents**, torques, control output and voltages
- joint **temperatures**, all six, plus the tool's
- tool pose, speed and force, and the target pose
- the wrist accelerometer
- main voltage, robot current, momentum, execution time
- robot mode, safety mode, safety status, runtime state — **as integers and as
  decoded text**
- digital and analogue I/O, tool I/O

The column set is derived from the RTDE recipe, so it is the **same on every
controller**: a field this one does not provide is a present, empty column,
which states "this robot does not report joint voltages". A missing column only
raises the question.

It subscribes to the telemetry stream rather than polling it. A poller running
beside a 125 Hz stream sees some packets twice and misses others, and the misses
are invisible afterwards.

## Running it

Automate → pick **arc_scan** → Run the job. Pre-flight still applies: the
carrier must be described and the sensor scales settled first.

The part is assumed to sit a little in front of and below the tool's current
pose, with its axis along the robot's X. **Check the first pose before running
it unattended** — the poses are listed in the step panel, and the geometry can
be changed there.

Then, to compare it against a simulator:

```bat
python sim_mujoco.py --real bench_runs\arc_scan_...jsonl ^
  --out sim_runs --menagerie mujoco_menagerie --view
```
