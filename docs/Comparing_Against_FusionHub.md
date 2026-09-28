# When FusionHub and this console disagree

On 28 September 2026 the smoke test ran clean and the operator then noticed that
the numbers on FusionHub's own Overview page and the numbers on this console's
Sensors page were not the same. Two recordings were taken of the same sensor at
the same moment — FusionHub's own `.mcap` and this console's `.csv` export — and
compared sample by sample.

**The data was identical. The interpretation was not.** All 1425 rows of the
console's export matched a FusionHub sample to six decimal places: same
quaternion, same accelerometer, same gyroscope, same magnetometer, nothing
dropped, nothing duplicated, nothing corrupted. Three things were then being
read wrongly, and all three were this console's fault. This page records what
they were and how each is now decided from the data rather than assumed,
because every one of them was invisible on a chart.

## The sensor

An **LPMS-B2** over Bluetooth, firmware 2.3.1, published by FusionHub's
External Output as Protocol Buffers on `tcp://<host>:8901`.

FusionHub's `.mcap` recordings embed the full `FileDescriptorSet`, so the schema
is not a guess. The message is `Fusion.proto.ImuData` from `stream_data.proto`:

| Field | # | Type | Note |
|---|---|---|---|
| `timecode` | 1 | int64 | **Frozen on this sensor — see below** |
| `recorded_time` | 2 | int64 | 0 |
| `gyroscope` | 3 | Vector | **degrees per second** |
| `accelerometer` | 4 | Vector | **g**, not m/s² |
| `period` | 5 | double | 0 |
| `frame_count` | 6 | int32 | 0 |
| `sensor_time` | 7 | int32 | 0 |
| `latency` | 8 | double | 0 |
| `start_tick` | 9 | int64 | 0 |
| `fake_timecode` | 10 | bool | false |
| `sensor_name` | 11 | string | `referenceImu` |
| `quaternion` | 12 | Quaternion | **world-to-sensor** |
| `euler` | 13 | Vector | FusionHub's own roll/pitch/yaw, degrees |
| `timestamp` | 14 | int64 | same frozen value as `timecode` |
| `sender_id` | 15 | string | `referenceImu` |
| `linear_velocity` | 16 | Vector | empty on this sensor |
| `magnetometer` | 17 | Vector | microtesla |

Half of those fields are zero on this hardware path. That is the real reason
the console identifies channels by physics rather than by field number: a
schema tells you what a field is called, not whether the sensor fills it in.

## 1. The gyroscope is in degrees per second

FusionHub publishes degrees. The console wrote the same numbers into columns
named `gyro_x_rad_s` and then multiplied by 57.3 to display "°/s", so a
**22.9 °/s elbow sweep was reported as 1425 °/s** — a UR5e elbow turning four
times a second.

The units decision is made per link and measured, not assumed, because degrees
and radians are the same numbers a factor of 57.3 apart and no reading tells
you which. The decisive test compares the gyroscope, integrated over a third of
a second, against the unit's own quaternion over the same window; the ratio is
either about 1 or about 57 and nothing in between. On this recording it came
out at 56.3.

Two things had stopped that test working:

* it needed a clock, and it was given the sensor's own — which is frozen (§3).
  It is now given arrival time at the host, which is jittery but perfectly
  adequate for telling 1 from 57;
* it compared **neighbouring samples**. This link arrives in Bluetooth bursts,
  several packets inside one millisecond and then a gap, so a sample-to-sample
  interval is mostly transport noise. Dividing a quaternion rounding error by
  30 µs produces hundreds of degrees per second and the ratio came out saying
  "radians". Integrating over a window fixes it.

With both starved, the decision fell through to a weak magnitude fallback,
which looked at a *stationary* sensor reading 0.12 and concluded radians. That
fallback is now used **only** for units that publish no orientation at all.
On a unit that does publish one, waiting for the arm to move is strictly
better than guessing: the wait costs a still sensor's gyro rows, a wrong guess
costs the whole campaign a factor of 57.

Backstops:

* a reading impossible under the standing verdict overturns it, loudly and
  counted (7 rad/s would be 400 °/s on this machine);
* until the verdict lands, the reading is moved out of `gyro` into `gyro_raw`,
  so a number whose unit is unknown can never be written into a column that
  asserts one. An empty cell can be read later; a plausible wrong number
  cannot.

## 2. The quaternion is published world-to-sensor

The console consumed it as sensor-to-world. That is not a cosmetic difference:

* **pitch changes sign.** The console showed +5.65° where FusionHub showed
  −5.30°, and roll differed by 1.9°.
* **gravity points the wrong way.** `gravity_from_quat` was 11.0° off, so
  subtracting gravity from the accelerometer left 9.81 × sin(11°) = **1.9 m/s²
  of pure fiction in `linear_accel` while the arm stood perfectly still** — on
  a channel the benchmark is scored against.
* it produced the "sensor and our own estimate agree on which way is down to
  11.0°" warning on the Sensors page, which read as a failing sensor and was
  nothing of the kind.

A quaternion handed over the wrong way round is still unit length, still
smooth, and still tracks the motion, so nothing downstream complains.

It is now measured. While the unit is near-stationary the accelerometer *is*
the gravity direction in the sensor frame, and so is `gravity_from_quat` under
the correct sense. The angle between them is computed both ways round over a
window of samples whose acceleration magnitude is close to 1 g; one answer is a
fraction of a degree and the other is not. On this recording: **11.19° as
published, 0.12° inverted.** Conjugating reproduces FusionHub's own Euler
angles to better than 0.05° on every sample.

## 3. FusionHub's timestamp field never advances

`timecode` and `timestamp` are **bit-identical on all 4280 messages** of
FusionHub's own recording. `recorded_time`, `period`, `frame_count`,
`sensor_time`, `latency` and `start_tick` are all zero, and `fake_timecode` is
false. FusionHub itself does not use them: its `.mcap` log times come from its
own receive clock, 290 s away from the `timecode` value.

Taken at face value, that field produced an export whose 1425 rows all carried
the same instant, an integration step of exactly zero for every orientation
filter, and a channel age computed against a clock that never moved.

`TimeMaster.to_master` used to be `t_src + offset` with the offset defaulting
to zero — a number from someone else's clock, returned as master time. A
channel now earns the right to place its own samples only once its timestamps
have been shown to advance **and** an offset onto the master has actually been
fitted from a shared event. Otherwise its rows are stamped as they arrive at
the host, and `clock.inertial_files_on` in the dataset manifest says which of
the two happened, per channel, and why.

Arrival time costs per-sample spacing and keeps everything else: it is
monotonic, it is shared with every other channel, and it is never a lie.

## 4. And the clock the host itself was using

The run file from the same smoke test declared 125 Hz in its own manifest and
held **43 Hz**, with a 1.469 s hole in it and 108 of its 303 samples sharing a
timestamp with a neighbour. Every interval in the file was a multiple of
15.6 ms.

15.625 ms is the Windows scheduler tick, and it is the resolution of
`time.monotonic()` there — which was the clock every recorded sample was
stamped with, and the clock the recorder paced itself by. The inertial unit
arrives every 5.3 ms, so that clock could not even order its samples, let
alone measure a sim-to-real gap with them.

Data timestamps and the recorder's pacing now use `time.perf_counter()`
(QueryPerformanceCounter on Windows, sub-microsecond), the recorder sleeps to
within a millisecond of each tick and spins the last of it, and the achieved
rate is measured and reported rather than assumed. Ordinary timeouts and UI
throttles still use `monotonic`, where 15 ms costs nothing.

## What the console now tells the operator

* the gyroscope's units, **and how they were decided**;
* whether the orientation was published back-to-front and turned round here,
  and how closely it then agrees with the sensor's own gravity reading;
* which clock each channel's readings are timed by, and why — the frozen
  timestamp now reads as a fact about the sensor rather than as "the clock
  jumped 8192 times";
* the arrival rate averaged over a second, not over fifty milliseconds, so it
  reads 188 Hz where FusionHub reads 190 rather than swinging 89–2040 Hz.

Pre-flight **blocks a recording job** until both the units and the orientation
sense are established for every live unit, and the `settle_sensors` job exists
to establish them: it sweeps the elbow 20° each way, records nothing, and takes
a few seconds. Both measurements need the arm to move, the fix is free before
a campaign, and it is unrecoverable after one.

## Checking it again yourself

`tests/test_fusionhub_agreement.py` replays a real FusionHub recording of this
cell — taken during a 25° elbow sweep at 0.4 rad/s, with FusionHub's own Euler
angles for every sample — through the actual ingestion path, and asserts that
this console reports what FusionHub reports. Against the code as it was, 8 of
its 13 checks fail, with exactly the numbers above.

```
python -m pytest tests/test_fusionhub_agreement.py -q
```

To take a fresh comparison recording: FusionHub → Recording → start, run a job,
stop; the `.mcap` lands in FusionHub's recordings folder. Export the console's
own data from Automate → Export. The two should agree channel for channel, and
the console's Euler angles should match FusionHub's `euler` field to a
fraction of a degree.

---

# What three real recordings showed about the campaign design

On the afternoon of 28 September the operator ran `single_run`, `elbow_sweep`
and `scan_shaped` on the cell and recorded each one from FusionHub's side.
Those three files say something the console could not have told them, and it is
about the experiment rather than the software.

## The sensor is fit for the job

From 26 s of the quiet stretch, with the arm holding still:

| | measured |
|---|---|
| gyro bias | below 0.001 °/s on all three axes |
| gyro noise (1σ) | 0.05 / 0.11 / 0.05 °/s |
| accelerometer noise (1σ) | 0.008 / 0.010 / 0.008 m/s² |
| orientation drift | 0.12° over 26 s |
| arrival rate | 190.4 Hz, steady |

That gives an **orientation measurement floor of roughly 0.2° over a ten second
run**, and an acceleration floor of about 0.01 m/s² per sample. Any sim-to-real
gap comfortably above those is a real gap. Gate A is not in doubt.

One correctable finding: the accelerometer reads **|a| = 9.977 m/s² at rest
against a true 9.807**, a **+1.73% scale error**. It is stable and it is in the
sensor, not the pipeline. Left alone it puts a systematic 1.7% on every
acceleration channel the benchmark scores.

And a positive result worth keeping: where the motion was long enough to reach
steady state, the **gyroscope tracked the commanded elbow rate to better than
2%** — 0.2035 rad/s measured against 0.2 commanded, 0.3995 against 0.4. The
sensor's scale factor is sound; there is no hidden 9% anywhere.

## Two of the four campaign cells could not have been what they were called

`elbow_sweep` was to sweep 0.2, 0.4, 0.6 and 0.9 rad/s at a fixed 25° of elbow
travel. What the recording contains is three repeats each at 0.2, 0.4 and 0.6 —
and nothing at 0.9.

Measuring the rate profile within each move, rather than its peak:

| commanded | profile over the move | sustained | verdict |
|---|---|---|---|
| 0.2 rad/s | flat for ~1.8 s | 0.2035 rad/s | a real cell |
| 0.4 rad/s | flat for ~0.8 s | 0.3995 rad/s | a real cell |
| 0.6 rad/s | 0.49 0.51 **0.65** 0.56 **0.65** 0.47 0.45 0.37 | never settles | **not a 0.6 cell** |
| 0.9 rad/s | did not run | — | **missing** |

At the controller's 1.2 rad/s² a 25° move spends v²/a of its travel on the
ramps alone. That leaves a 2.0 s cruise at 0.2, a 0.8 s cruise at 0.4, a 0.23 s
corner at 0.6, and at 0.9 **nothing at all** — 25° under that acceleration
peaks at 0.72 rad/s and must start braking before it ever reaches 0.9.

This is the defect that matters most, because it is invisible. The runs look
perfectly good. They open cleanly, they plot correctly, their cell keys read
`0.600|mid_workspace|point_to_point`. But the factor the campaign exists to
sweep would not have spanned what its own labels claimed; the 0.6 and 0.9 cells
would have held nearly the same motion as each other; and Gate C — *does the
gap vary with condition?* — would have been asked about a condition that barely
varied.

**A fixed excursion is the wrong thing to hold constant across a speed sweep.**
What should be held constant is how long the joint spends at the speed the cell
is named after. The excursion is now derived from the speed:

| speed | sized to | ramp each end | cruise |
|---|---|---|---|
| 0.2 rad/s | 13° | 0.17 s | 1.00 s |
| 0.4 rad/s | 31° | 0.33 s | 1.00 s |
| 0.6 rad/s | 52° | 0.50 s | 1.00 s |
| 0.9 rad/s | 90° | 0.75 s | 1.00 s |

A `joint_move` given an explicit `amplitude_deg` too small for its speed is now
refused with the arithmetic, and the dataset export reads `target_qd` back out
of each finished run and flags any run whose joint never reached, or never
held, the speed its cell is named after.

## `scan_shaped` carries nothing to score

Over 46 seconds the carrier turned **0.9°** in total, and linear acceleration
sat flat on the sensor's own noise floor at 0.17 m/s², with 27 samples out of
8749 above 0.5 m/s². A tool-space box traced at 0.08 m/s with the tool
orientation held constant is, to an inertial unit, indistinguishable from
standing still.

It is a fine demonstration of the inspection application. It is not a benchmark
run: scored on orientation, angular rate and acceleration it would return a gap
of about zero with an error bar larger than the gap, and averaging it in with
real runs would dilute every number it touched. Its job note now says so.
