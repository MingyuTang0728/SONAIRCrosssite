"""
SONAIR — Sim2real Operational beNchmark for AI Robotics.

A scored sim-to-real gap benchmark built on quantities that exist on BOTH
sides of the gap: orientation, angular rate, acceleration, position and their
temporal derivatives. Those are the modalities Sam named first in the review,
for the reason that they are the only ones that are cheap to ground-truth on a
real UR5e and cheap to generate faithfully in Isaac Sim.

The package is deliberately split along the phases of the deployment plan:

    schema    Phase 3   canonical run manifest + sample record, one JSONL per run
    clock     Phase 1   single time master, channel offset estimation
    imu       Phase 0   FusionHub / D435i / consumer IMU ingestion
    phase0    Phase 0   noise floor, bias, tumble alignment -> error budget rows
    budget    Phase 2   the five-row error budget, i.e. the measurement floor
    campaign  Phase 3   the condition sweep (velocity x config x trajectory x repeat)
    isaac     Phase 4   command export to Isaac Sim, sim run import, sensor degradation
    metrics   Phase 5   the gap itself: position, orientation, temporal, distributional
    scoring   Phase 6   submission harness, trivial baselines, leaderboard

Nothing here imports the robot, the camera or Isaac. Acquisition lives in
multimodal_bridge.py / bench_recorder.py; this package only ever sees files.
"""

__version__ = "0.1.0"

SCHEMA_VERSION = "sonair-run-1"
SUBMISSION_VERSION = "sonair-submission-1"

__all__ = [
    "SCHEMA_VERSION",
    "SUBMISSION_VERSION",
    "__version__",
]
