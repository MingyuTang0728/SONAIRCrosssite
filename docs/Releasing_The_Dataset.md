# Releasing the dataset

    python -m sonair_benchmark release --runs bench_runs --state campaign/state.json \
        --plan campaign/plan.json --tcp-offset <pendant TCP> --out release/v1
    python -m sonair_benchmark verify-release release/v1

In PyCharm this is the run configuration **6 - Build the public dataset release**.

## What it makes

| Folder | Contents | Publish? |
|---|---|---|
| `release/v1/E1/real/` | the identification set, in full | yes |
| `release/v1/E2/published/real/` + `sim_s0/` | the published cells: real runs and S0 | yes |
| `release/v1/E2/heldout/commands/` + `sim_s0/` | held-out cells: the **commanded trajectory only**, plus S0 | yes |
| `release/v1/plan.json`, `manifest.json`, `README.md` | which cells are held out; versions; a SHA-256 for every file | yes |
| `release/v1_PRIVATE/E2/heldout/real/` | the held-out real runs | **never** |
| `release/v1_PRIVATE/E3/real/` | the out-of-distribution set | **never** |

A held-out *command* file keeps the run's manifest. Each of its samples
carries only `t`, `target_q`, `target_qd`, `target_moment`, `speed_scaling`
and the controller's clock. These are what the controller was told, or what
it generated from what it was told. Nothing measured is kept: no joints, no
tool pose, no IMU.

## What never goes in

* The continuous robot and IMU logs (`ur_*.csv`, `imu_*.csv`). They cover
  every minute of the campaign, held-out runs included.
* Campaign state, superseded copies (`*.superseded`), and runs the campaign
  did not accept. With `--state`, only runs marked done under the current
  protocol are released.
* Rehearsals from the simulated cell.

## The check

`verify-release` reads the public folder back, independently of how it was
built. It refuses the folder if any of the following is true:

* it holds a real run of a held-out cell, or any real run from E3;
* a command file holds any measured channel;
* there is a log, a state file or an unexpected item;
* a file does not match its checksum;
* the folder is the PRIVATE set.

The build runs this check itself. Run it again on anything before it is
uploaded, and on a downloaded copy to confirm it arrived intact.

## S0 in the release

If no `--sim` folder is given, S0 is made for every E2 run, exactly as the
scored replay makes it: MuJoCo, the menagerie UR5e, the run's commanded
joints, the pendant's tool offset (`--tcp-offset`) and the IMU mounting from
`calib/imu_cal.json`. If a run cannot be simulated, the reason is listed,
and that run is left out of both folders.

## Scoring with the private set

Score with the public folder beside the private one. B1 is a *fitted*
baseline, so it must be fitted on the published cells and scored on the
held-out ones:

    python -m sonair_benchmark score \
        --real release/v1/E2/published/real release/v1_PRIVATE/E2/heldout/real \
        --sim  release/v1/E2/published/sim_s0 release/v1_PRIVATE/E2/heldout/sim_s0 \
        --plan release/v1/plan.json --track-a "..." --track-b "..." \
        --out site/leaderboard.json
