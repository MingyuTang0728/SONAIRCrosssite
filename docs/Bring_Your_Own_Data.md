# Measuring the sim-to-real gap on your own robot

For anyone who owns a robot and wants one number: **how far is the standard
simulation from my arm, for the motions I run?** You bring a log; you get a
report, scored by the SONAIR benchmark's own rule.

```
python -m sonair_benchmark intake my_log.csv --robot ur5e --payload 1.2 --out my_report
```

Open `my_report/report.html`. The numbers behind it are in `report.json`, and
every motion, real and simulated, is in `runs/` in the SONAIR run format, so
anything in the report can be checked and scored again.

## What the log must contain

| | Needed | Why |
|---|---|---|
| Time | yes | each row's sample time, increasing |
| **Commanded joints** (`target_q`) | **yes** | the simulator is fed what the robot was *told*. A log of only what it did measures nothing |
| Measured joints (`actual_q`) | yes | what the arm did |
| Tool position and rotation (`actual_TCP_pose`) | recommended | without it, the flange is used, from the nominal kinematics |
| Rate | 20 Hz minimum, 125 Hz or more recommended | the gap is in the first tenths of a second of each move and in the settle after it |

## Formats

* **UR RTDE recording**: the CSV the UR RTDE client writes, with columns
  `timestamp`, `target_q_0..5`, `actual_q_0..5`, `actual_TCP_pose_0..5`.
  Recognised automatically. Record it with the client's `record.py`, adding
  `target_q` to the recipe.
* **SONAIR robot log**: the `ur_*.csv` the console writes. Recognised
  automatically.
* **Any other CSV**: write a mapping file and pass it as `--mapping map.json`:

```json
{"time": "time_s", "time_unit": "s",
 "angle_unit": "rad",
 "target_q": ["cmd_j1", "cmd_j2", "cmd_j3", "cmd_j4", "cmd_j5", "cmd_j6"],
 "q": ["pos_j1", "pos_j2", "pos_j3", "pos_j4", "pos_j5", "pos_j6"],
 "tcp_pos": ["tcp_x", "tcp_y", "tcp_z"], "tcp_unit": "m",
 "tcp_rot": ["tcp_rx", "tcp_ry", "tcp_rz"]}
```

`time_unit` is `s`, `ms`, `us` or `ns`. `angle_unit` is `rad` or `deg`.
`tcp_unit` is `m` or `mm`. The tool rotation is a UR rotation vector.

## What happens to the log

1. **Read and checked.** Problems that make the result meaningless stop it,
   and the message says what is wrong: no commanded joints; time going
   backwards; under 20 Hz; angles that look like degrees but were declared as
   radians. Smaller problems are listed in the report under *About the log*.
2. **Cut into motions.** Each stretch where the commanded joints move, plus
   0.5 s before it and 1 s after it to include the settle. Recording gaps cut
   motions too.
3. **Replayed through S0.** S0 is the MuJoCo Menagerie model of your arm,
   unchanged, driven by your controller's own commanded joints. It is the
   same reference the benchmark uses. The tool offset is worked out from the
   log's first row, so there is nothing to type in. Give `--payload` (kg on
   the flange) if there was one: it changes the result.
4. **Scored** by the benchmark's rule. For each motion: the real tool point
   against the simulated one, median and 95th percentile, plus each joint's
   error. The report groups the motions by their peak commanded joint speed,
   because the gap grows with speed.

## Robots

| | Read and checked | Replayed through S0 |
|---|---|---|
| UR5e | yes | yes |
| UR10e | yes | yes, with the Menagerie's UR10e (`python install_sim.py`) |
| UR3e, UR16e | yes | not yet: the Menagerie has no model of them |

## Your data stays yours

Runs made from your log are labelled `experiment: "U"`. The benchmark's
scoring leaves them out, and the dataset release refuses them. Nothing is
uploaded: the whole analysis runs on your own computer.

## Next

Today the report shows the reference gap. The corrected models, S2
(identified on the E1 set) and B3 (a learned residual), will be applied to
the same motions once they have been trained on the SONAIR dataset, so the
report will also show how much of your gap they close.
