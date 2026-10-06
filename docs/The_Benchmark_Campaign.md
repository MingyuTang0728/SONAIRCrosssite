# The benchmark campaign, and the IMU calibration it needs

The campaign is **6 elbow speeds × 3 arm configurations × 3 trajectory types =
54 conditions, 5 repeats each, over 3 sessions**: 270 runs. About 30% of the
conditions are held out, chosen once with a fixed seed and spread across the
speeds, so a model fitted to the published runs is tested on conditions it
has never seen. The third session is recorded after one deliberate carrier
refit, so that refit error is measured rather than assumed away.

It runs from the **Automate** page, in the section *The benchmark campaign*.

## Once: teach the three configurations

Jog the arm into each configuration (near singular, mid workspace, extended)
with the carrier clear of everything, or use **Suggest from here**. Choose
which way the elbow has room to move, then press **Teach here**. Every run
moves the elbow up to 45° from the taught position in that direction.

**Near singular** must be taught with the elbow nearly straight: about 28°
of bend, and no more than 35°. A near-singular configuration bent more than
that is refused. In the pilot it was taught at 53°, which is further from
straight than the extended configuration should be. Its elbow must move away
from straight, and the check refuses a direction that would bring it within
10° of straight.

## Protocol 2 (from October 2026)

The pilot sessions (protocol 1) sized point-to-point and stop-start moves
from the speed, so the elbow's travel grew with the speed: 6° at 0.2 rad/s
and 64° at 0.9 rad/s. The load the elbow feels changes with its angle, so
any trend in the gap could have come from speed or from travel, and the two
could not be separated. Protocol 2 fixes the travel:

| Trajectory | Elbow travel | Acceleration | Pauses |
|---|---|---|---|
| point to point | 45° out and back | 3 rad/s² | 0.5 s at the turn |
| stop-start | 2 × 22.5° out, 2 × 22.5° back | 3 rad/s² | 0.4 s at each stop |
| contour | 0–40° sinusoid, 2 cycles | — | — |

Both of these motions are sent to the controller as one program. The pauses
are then timed by the controller's own `sleep()`, not by how quickly the
agent notices that the arm has arrived. The wrist force sensor is zeroed at
the start pose before every run.

Runs recorded under protocol 1 stay in the state file as history, but they
do not count as done: running a session records them again. A run that is
recorded again never overwrites the earlier file. The old file is kept next
to the new one as `<run>.jsonl.<time>.superseded`.

## Once per carrier: run `imu_mount_cal`

Pick **imu mount cal** in the job list and run it. It takes about a minute.
The wrist turns about four joints, up to 40° each way, stopping after every
move, and ends where it started. It measures two things the simulator
comparison cannot do without:

* **The IMU's timing.** The robot's packets and the IMU's reach the PC by
  different routes: RTDE on one side, Bluetooth and FusionHub on the other.
  So the same motion is stamped at two different times. On this cell's
  arc_scan the IMU was about **100 ms** late. At 0.9 rad/s, 100 ms is 5° of
  wrist rotation. That would be charged to the simulator as a gap.
* **The IMU's mounting.** The IMU reports in its own axes, and the simulated
  IMU reports in the flange's axes. The arc_scan data shows this cell's IMU
  turned **−90° about the flange's z axis**. Without that rotation, gyro x is
  compared with gyro y.

The job log shows the offset, the mounting angle, and two checks: how well
the rotated gyro matches the robot's own turning rate, and how well gravity
agrees at the stops. The job refuses to save a calibration it cannot stand
behind:

* one-axis motion with no stops;
* a robot log with holes in it;
* a gyro reporting deg/s;
* a robot base that is not upright.

The result goes to `calib/imu_cal.json`. Run the job again after the refit.
The campaign preview reminds you if the calibration is missing, or if it is
older than the carrier description.

You can also run it by hand on any pair of logs recorded together:

```bat
python imu_align.py --ur ur_logs\ur_X.csv --imu imu_logs\imu_X.csv --save calib\imu_cal.json
```

## Each session: check, then run

Choose the session and press **Check this session**. It lists every condition
the session still needs, with the elbow travel and duration of each. Every
excursion is pushed through the arm's kinematics and checked against the safe
envelope before anything moves. **Run this session** then records every run
under its plan name. Each run is read back when it closes: it must have
reached its commanded speed and have no stale robot data. A run that passes
is marked done.

A session that stops, for whatever reason, can simply be run again. Runs
already done are skipped. A run whose file failed its check is recorded
again, and nothing else is.

| Session | Runs | Time |
|---|---|---|
| 1 | 108 | about 21 min |
| 2 | 108 | about 21 min |
| 3 (after the refit) | 54 | about 11 min |

Session 3 is refused until the carrier has been described again after the
refit.

## The identification set (E1)

The section *Identification set (E1)*, below the campaign, records the
benchmark's **training data**. The campaign moves only the elbow, and its
runs are the ones submissions are scored on. E1 moves every joint, so a
simulator can be tuned without seeing the runs it is scored on. E1 runs
carry `experiment: "E1"` in their manifest, and `gap` and `score` leave them
out.

| Motion | Where | Each |
|---|---|---|
| chirp, one joint at a time, 0.05 → 2 Hz (log sweep) | mid workspace | 60 s × 6 joints |
| all six joints together, Fourier (0.1–1.3 Hz), 3 harmonic sets × 2 repeats | each configuration | 24 s × 18 |

Every excitation is one URScript program: a `speedj` loop computed from the
time, then a stop and a slow `movej` back to the start. Before anything is
sent, the whole path is integrated exactly as the controller will step
through it, and checked every 32 ms:

* the tool must stay inside the safe envelope;
* the elbow must stay 10–155° bent;
* the wrist must stay 20 cm from the base axis.

An excitation that fails the check is tried in the opposite direction, then
at 70% and 50% of its size. If it still fails, it is left out and the reason
is given. Joint speed is capped at 0.8 rad/s and acceleration at 2.5 rad/s².

The set is recorded twice, about 20 minutes each:

1. **Set 1**, with the carrier alone.
2. **Set 2**, with a known added mass, for example 0.5 kg. Before running it:
   * bolt the mass on;
   * weigh the carrier and mass together;
   * save that weight, with its centre of mass, under *What is on the flange*;
   * set the same payload on the pendant.

   Set 2 is refused until the carrier is described at least 0.2 kg heavier
   than it was for Set 1. Either set is also refused if the carrier changes
   partway through recording it.

## What the calibration changes downstream

* `sim_mujoco.py` reads `calib/imu_cal.json` and writes the simulated IMU in
  the real IMU's axes. The simulated run's notes say which axes were used.
* `python -m sonair_benchmark gap` takes the IMU's latency out before
  comparing gyros. It reports a gyro gap per run (median, p95, and relative
  RMS) and per condition.

## A finding to decide on: the simulator's servo lag

In the menagerie UR5e model, each joint is a position servo with kp 2000 and
kv 400. A servo like that trails its command by kv/kp = **0.2 s**. The real
controller tracks its command within milliseconds. At the campaign's speeds
this puts the simulated arm 1.5–2.6° behind the real one, and the gap grows
with speed. That is the simulator's controller model, not the arm's physics.

`tests/test_imu_sim.py` prints it rather than hiding it. Whether to keep it as
part of the measured gap, or to give the simulated servo the real
controller's velocity feed-forward, is a modelling decision for the study.
