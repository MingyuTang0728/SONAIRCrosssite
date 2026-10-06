# The simulated cell and the live twin

There are two ways the platform runs with a simulator. Both use the same
MuJoCo model as the benchmark's S0 baseline, so all three use one simulator.

| | What it is | When to use it |
|---|---|---|
| **Simulated cell** (`sim_cell.py`) | The whole platform, with the robot simulated. URSim, Universal Robots' own controller software, runs the commands. MuJoCo is the arm and its IMU. | No robot available. Rehearsing a session, E1 or a new job. Testing the console. |
| **Live twin** (`twin.py`, in the agent) | MuJoCo running beside the real arm, fed the robot's own commanded joints. It is drawn as an amber ghost over the arm in the Live arm column. | Every real run: you see the sim-to-real gap as it happens. |

The **scored** simulation is neither of these. It is the offline replay of each
recorded run (`sim_mujoco.py`, [Replaying a run](Replaying_A_Run_In_MuJoCo.md)),
because the replay is reproducible. The twin runs the same model on the same
input, so the two agree, but only the replay is the record.

## Why URSim, and not MuJoCo alone

The platform does not move the arm itself. It sends URScript to the controller:
`movej`, `speedj` loops, the E1 programs. The controller's own motion planner
turns these into the commanded trajectory, `target_q`. MuJoCo has no UR
controller in it. If we re-implemented one, we would get a second, different
controller, and its differences would be charged to the sim-to-real gap.
URSim **is** UR's controller, version 5.11.11, the same generation as the
cell's. It executes our programs exactly as the real one does. MuJoCo only
supplies what URSim does not: the arm's dynamics and the inertial sensor.

```
console/agent ──RTDE──► sim_cell (127.0.0.2:30004) ◄──RTDE── URSim (127.0.0.1)
              ──scripts, dashboard──► sim_cell ──relay──► URSim
                                       │ MuJoCo follows URSim's target_q
              ◄──UDP IMU :5005─────────┘ measured joints, tool point, IMU
```

## Setting up (once)

1. Install **Docker Desktop**, start it, and let it finish starting.
2. Run `python install_sim.py`, or the PyCharm run configuration **4 - Install
   simulation**. It installs:
   * MuJoCo;
   * the UR5e model, a few MB, into `%LOCALAPPDATA%\SONAIR\mujoco_menagerie`;
   * URSim 5.11.11, about 1 GB, downloaded once.

   It says plainly what is ready and what is not.
3. Start the simulated cell once (next section). Then open URSim's own screen
   at <http://localhost:6080/vnc.html>, and when PolyScope asks, **confirm the
   safety configuration**. Until then URSim works, but it refuses the
   pendant speed slider (*"SafetySetup has not been confirmed yet"*).

## Running it

1. `.\start_sim_cell.ps1`, or the run configuration **5 - Simulated cell**.
   This starts URSim in Docker, waits for it to boot, powers the arm on,
   releases the brakes, and then says *Simulated cell ready*.
2. Start the agent with the cell as the robot:
   `.\start_agent.ps1 -UrIp 127.0.0.2`.
3. Serve the console as usual (`.\serve_console.ps1`) and connect.

The header then shows **SIMULATED CELL**. The Connect page names URSim and
MuJoCo. The motion sensor appears without any setup: the agent's UDP link
on port 5005 receives the simulated IMU.

## What is kept apart, so that it can never be mixed with real data

* Every run is labelled `side: "sim"`, its notes begin with
  *SIMULATED CELL*, and it is written to `bench_runs\simcell\`.
* The robot and IMU logs are named `simcell_ur_…csv` and `simcell_imu_…csv`.
* Campaign and E1 progress go into `campaign\state_simcell.json`. A rehearsal
  never marks a real run done. The taught configurations are copied from the
  real book the first time, so a rehearsal drives to the same positions the
  real cell would.
* What the cell learns about the IMU (units, which way round its quaternion
  is) is not remembered for later sessions. Only a real sensor's is.

The cell announces itself in the RTDE handshake (*"SONAIR simulated cell"*).
The agent decides from that, not from a setting, so nobody can forget to
switch a setting.

## The live twin

It starts by itself whenever the robot link is up and MuJoCo is installed. The
**Twin** button in the Live arm column shows or hides it. Beneath the view are
two readouts:

* **Twin − real**: the RMS difference per joint over the last 5 s. A joint
  above 1° turns amber.
* **Tool point**: the twin's tool point against the real one, now and as an
  RMS, in mm.

Against the real arm this is the gap itself. On the simulated cell the "real"
arm *is* the same model, so the twin reads 0.00. With
`sim_cell.py --plant ursim` (URSim's own, perfect joints) you can see what the
S0 model's servo lag looks like: about 3° at the base, mid-move, on a 1 rad/s
`movej`. At rest the twin also sits 12 mm low, because S0's servo sags under
gravity.

## What URSim showed that no test could

Running the platform's own programs in UR's controller software found three
faults, all now fixed:

* **Programs logged a compile error on every send.** Every program we generate
  ended with a line calling itself. The controller already runs a `def` block
  as the program. It then compiled the call as a second program and logged
  *"name 'sonair_sine' is not defined"*. The motion still ran, which is why
  this went unseen. The call lines are gone.
* **The contour ended off its start.** After two cycles at 0.9 rad/s, the
  integrated `speedj` steps left the elbow 15 mrad (0.9°) from its start. The
  program now records its start and drives back to it. It ends at 0.000 mrad.
* **The RTDE handshake broke on a controller message.** A controller text
  message during the handshake was taken for the reply. Every later reply was
  then read one step out of order, and the link fell back to port 30003. The
  5.11 and 5.2x message layouts also differ. Both are now handled.
