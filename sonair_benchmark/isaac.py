"""
Phase 4 — the simulated counterpart, and the Sim2Real contract with Isaac Sim.

The simulated dataset is NOT a general model of the cell. It is the same runs,
generated from the same commands. Every deviation in the setup is a confound
rather than a research variable, so this module exists to make the four things
that must match impossible to get wrong by accident:

  1. same commanded trajectories, same units, same command rate
  2. the carrier included as MEASURED mass and centre of mass at the wrist,
     not the nominal CAD values
  3. sensor outputs at the same rate and the same MEASURED mounting offsets
  4. simulated sensors degraded with the noise and bias measured in Phase 0

Point 4 is the one people skip. An IDEAL simulated IMU makes the gap look
larger than it is, for a reason that has nothing to do with dynamics — you
would be measuring "Isaac has no sensor noise", which nobody needs a benchmark
to discover.

This module does not import Isaac. It writes a JSON contract that the Isaac
script reads, and it reads back what Isaac wrote. That keeps the benchmark
runnable on a machine without an RTX card, which matters when a reviewer wants
to re-score your leaderboard.
"""
from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, field, asdict
from pathlib import Path

from .schema import Run, RunManifest, RunWriter, Sample


@dataclass
class SimContract:
    """
    Everything the Isaac side must reproduce. Written next to the sim dataset
    and checked on import — a sim run generated under a different contract is
    refused rather than silently pooled.
    """

    calib_version: str
    command_rate_hz: float = 125.0
    sample_rate_hz: float = 125.0
    carrier_mass_kg: float = 0.0            # MEASURED in Phase 1
    carrier_com_m: tuple = (0.0, 0.0, 0.0)  # MEASURED, TCP frame
    imu_mount_offset_m: tuple = (0.0, 0.0, 0.0)   # MEASURED in Phase 2
    imu_mount_rpy_rad: tuple = (0.0, 0.0, 0.0)    # MEASURED in Phase 2
    # Phase 0 noise model, applied to the ideal simulator state
    gyro_noise_rad_s: tuple = (0.0, 0.0, 0.0)
    gyro_bias_rad_s: tuple = (0.0, 0.0, 0.0)
    accel_noise_m_s2: tuple = (0.0, 0.0, 0.0)
    accel_bias_m_s2: tuple = (0.0, 0.0, 0.0)
    tracker_noise_m: float = 0.0
    solver: str = "isaac-sim"
    physics_dt: float = 1.0 / 240.0
    notes: str = ""

    def save(self, path: str | Path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "SimContract":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def from_measurements(cls, calib_version: str, noise_floor,
                          carrier_mass_kg: float, carrier_com_m,
                          imu_mount_offset_m, imu_mount_rpy_rad,
                          tracker_noise_m: float = 0.0) -> "SimContract":
        """Build the contract from the Phase 0/1/2 numbers rather than by hand."""
        return cls(
            calib_version=calib_version,
            carrier_mass_kg=carrier_mass_kg,
            carrier_com_m=tuple(carrier_com_m),
            imu_mount_offset_m=tuple(imu_mount_offset_m),
            imu_mount_rpy_rad=tuple(imu_mount_rpy_rad),
            gyro_noise_rad_s=tuple(noise_floor.gyro_noise),
            gyro_bias_rad_s=tuple(noise_floor.gyro_bias),
            accel_noise_m_s2=tuple(noise_floor.accel_noise),
            accel_bias_m_s2=tuple(noise_floor.accel_bias[:3]),
            tracker_noise_m=tracker_noise_m,
        )

    def check_against(self, other: "SimContract") -> list[str]:
        problems = []
        if self.calib_version != other.calib_version:
            problems.append(
                f"calibration mismatch: contract {self.calib_version!r} vs run {other.calib_version!r}")
        if abs(self.sample_rate_hz - other.sample_rate_hz) > 1e-6:
            problems.append("sample rate mismatch between contract and sim run")
        if abs(self.carrier_mass_kg - other.carrier_mass_kg) > 1e-4:
            problems.append("carrier mass mismatch — the wrist payload is a confound, not a variable")
        return problems


def export_commands(planned_run, waypoints, path: str | Path,
                    contract: SimContract) -> Path:
    """
    Write the command file Isaac replays.

    `waypoints` is the list the Zig-Zag planner already produces in
    Remote_control.html: [{"type": ..., "coords": [x,y,z,rx,ry,rz]}, ...].
    Exporting the SAME structure the real robot executed is what makes point 1
    of the contract true by construction rather than by discipline.
    """
    doc = {
        "run_id": planned_run.run_id,
        "cell": {
            "joint_vel": planned_run.joint_vel,
            "arm_config": planned_run.arm_config,
            "traj_type": planned_run.traj_type,
        },
        "repeat_idx": planned_run.repeat_idx,
        "command_rate_hz": contract.command_rate_hz,
        "contract": asdict(contract),
        "waypoints": [
            {"type": w.get("type", ""), "coords": [float(c) for c in w["coords"]]}
            for w in waypoints
        ],
    }
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(doc, indent=2), encoding="utf-8")
    return p


# ----------------------------------------------------------------------------
# sensor degradation — turning ideal simulator state into a plausible reading
# ----------------------------------------------------------------------------

class SensorDegrader:
    """
    Applies the Phase 0 noise model to ideal simulator output.

    Bias is drawn ONCE per run and held, noise is drawn per sample. That split
    matters: a per-sample bias would average out over a run and the resulting
    sim data would be unrealistically well behaved, which flatters any model
    that is later scored against it.
    """

    def __init__(self, contract: SimContract, seed: int | None = None):
        self.c = contract
        self.rng = random.Random(seed)
        self._gyro_bias = [self.rng.gauss(b, abs(b) * 0.1 + 1e-9)
                           for b in contract.gyro_bias_rad_s]
        self._accel_bias = [self.rng.gauss(b, abs(b) * 0.1 + 1e-9)
                            for b in contract.accel_bias_m_s2]

    def gyro(self, ideal) -> list[float]:
        return [float(v) + self._gyro_bias[i] + self.rng.gauss(0.0, self.c.gyro_noise_rad_s[i])
                for i, v in enumerate(ideal[:3])]

    def accel(self, ideal) -> list[float]:
        return [float(v) + self._accel_bias[i] + self.rng.gauss(0.0, self.c.accel_noise_m_s2[i])
                for i, v in enumerate(ideal[:3])]

    def tracker(self, ideal) -> list[float]:
        s = self.c.tracker_noise_m
        return [float(v) + (self.rng.gauss(0.0, s) if s > 0 else 0.0) for v in ideal[:3]]


def import_isaac_run(path: str | Path, planned_run, contract: SimContract,
                     out_path: str | Path, degrade: bool = True,
                     seed: int | None = None) -> Path:
    """
    Convert an Isaac log into a canonical SONAIR run file.

    The Isaac log is expected as JSON Lines with at least
    {"t":..., "tcp_pos":[x,y,z], "tcp_rot":[rx,ry,rz]} and optionally
    {"gyro":[...], "accel":[...], "quat":[w,x,y,z]}. Anything else is ignored
    rather than guessed at.
    """
    deg = SensorDegrader(contract, seed=seed) if degrade else None
    manifest = planned_run.manifest(
        "sim", contract.calib_version,
        carrier_mass_kg=contract.carrier_mass_kg,
        carrier_com_m=tuple(contract.carrier_com_m),
        sample_rate_hz=contract.sample_rate_hz,
        notes=f"generated by {contract.solver}, degraded={degrade}",
    )
    n = 0
    with RunWriter(out_path, manifest) as w:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "t" not in row:
                continue
            imu_block = {}
            if "gyro" in row or "accel" in row or "quat" in row:
                rec = {}
                if "quat" in row:
                    rec["quat"] = [float(v) for v in row["quat"]]
                if "gyro" in row:
                    g = row["gyro"]
                    rec["gyro"] = deg.gyro(g) if deg else [float(v) for v in g]
                if "accel" in row:
                    a = row["accel"]
                    rec["accel"] = deg.accel(a) if deg else [float(v) for v in a]
                imu_block["ind0"] = rec
            em_block = {}
            if "em" in row and isinstance(row["em"], dict):
                for k, v in row["em"].items():
                    em_block[k] = deg.tracker(v) if deg else [float(c) for c in v]
            w.write(Sample(
                t=float(row["t"]),
                q=row.get("q"),
                qd=row.get("qd"),
                tcp_pos=row.get("tcp_pos"),
                tcp_rot=row.get("tcp_rot"),
                imu=imu_block, em=em_block,
            ))
            n += 1
    return Path(out_path)


ISAAC_REPLAY_TEMPLATE = '''\
# SONAIR Phase 4 — Isaac Sim replay stub.
#
# Run INSIDE Isaac Sim's python (./python.sh sonair_isaac_replay.py <commands.json>).
# It is a stub on purpose: it names the four contract points and the exact log
# format the benchmark expects, and leaves the scene wiring to your USD setup.
#
#   1. same commanded trajectory, same units, same rate   -> waypoints below
#   2. carrier mass and COM from the contract, NOT the CAD -> apply_payload()
#   3. sensor rate and mounting offsets from the contract  -> imu_prim_path
#   4. noise/bias applied OUTSIDE Isaac, by isaac.import_isaac_run(degrade=True)
#
# Log one JSON object per line to <run_id>.isaac.jsonl:
#   {"t": 0.008, "q": [...6...], "tcp_pos": [x,y,z], "tcp_rot": [rx,ry,rz],
#    "gyro": [x,y,z], "accel": [x,y,z], "quat": [w,x,y,z]}

import json, sys
from pathlib import Path

doc = json.loads(Path(sys.argv[1]).read_text())
contract = doc["contract"]
waypoints = doc["waypoints"]
dt = 1.0 / contract["sample_rate_hz"]

# --- your scene setup goes here -------------------------------------------
# from omni.isaac.core import World
# from omni.isaac.core.robots import Robot
# world = World(physics_dt=contract["physics_dt"], rendering_dt=dt)
# ur = world.scene.add(Robot(prim_path="/World/ur5e", name="ur5e"))
# apply_payload(ur, mass=contract["carrier_mass_kg"], com=contract["carrier_com_m"])
# imu = attach_imu(ur, offset=contract["imu_mount_offset_m"],
#                      rpy=contract["imu_mount_rpy_rad"])
# --------------------------------------------------------------------------

out = Path(doc["run_id"] + ".isaac.jsonl").open("w")
t = 0.0
for wp in waypoints:
    x, y, z, rx, ry, rz = wp["coords"]
    # move_to(ur, (x, y, z), (rx, ry, rz), max_joint_vel=doc["cell"]["joint_vel"])
    # while not at_target(ur):
    #     world.step(render=False)
    #     out.write(json.dumps({
    #         "t": round(t, 6),
    #         "q": ur.get_joint_positions().tolist(),
    #         "tcp_pos": tcp_position(ur), "tcp_rot": tcp_rotvec(ur),
    #         "gyro": imu.angular_velocity(), "accel": imu.linear_acceleration(),
    #         "quat": imu.orientation(),
    #     }) + "\\n")
    #     t += dt
    pass
out.close()
'''


def write_isaac_stub(path: str | Path) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(ISAAC_REPLAY_TEMPLATE, encoding="utf-8")
    return p
