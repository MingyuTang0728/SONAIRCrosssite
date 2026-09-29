"""
Phase 3 logging schema — the canonical on-disk form of a SONAIR run.

One run is one file. The file is JSON Lines: the first line is the manifest,
every subsequent line is one sample. That choice matters more than it looks:
a run that is killed halfway through is still a valid, readable run up to the
point it died, which is what you want during a four-week acquisition campaign.

The field that people leave out and then need is `calib_version`. A dataset
recorded either side of a recalibration cannot safely be pooled, and without
the field there is no way to find out afterwards which side a run fell on.
It is therefore mandatory in the manifest and validated on load.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Iterator, Sequence

from . import SCHEMA_VERSION

# The four sweep factors of Phase 3. A run sits in exactly one cell of this grid.
FACTORS = ("joint_vel", "arm_config", "traj_type", "repeat_idx")

ARM_CONFIGS = ("near_singular", "mid_workspace", "extended")
TRAJ_TYPES = ("point_to_point", "contour", "stop_start")

# Which side of the gap a run was recorded on.
SIDES = ("real", "sim")


@dataclass
class RunManifest:
    """Line 0 of every run file. Everything needed to place the run in the sweep."""

    run_id: str
    side: str                      # "real" | "sim"
    calib_version: str             # points at the Phase 2 residuals in force
    joint_vel: float               # commanded elbow angular velocity, rad/s
    arm_config: str                # one of ARM_CONFIGS
    traj_type: str                 # one of TRAJ_TYPES
    repeat_idx: int                # which repetition of this cell
    carrier_id: str = "carrier-v1"  # which physical carrier / refit generation
    carrier_mass_kg: float = 0.0    # MEASURED, not from CAD (Phase 1)
    carrier_com_m: tuple = (0.0, 0.0, 0.0)  # measured centre of mass in TCP frame
    sample_rate_hz: float = 125.0
    started_utc: str = ""
    operator: str = ""
    notes: str = ""
    schema: str = SCHEMA_VERSION

    def cell(self) -> tuple:
        """The condition cell, ignoring repeat index. Used for held-out splits."""
        return (round(self.joint_vel, 4), self.arm_config, self.traj_type)

    def cell_key(self) -> str:
        v, c, t = self.cell()
        return f"{v:.3f}|{c}|{t}"

    def validate(self) -> list[str]:
        problems = []
        if self.side not in SIDES:
            problems.append(f"side must be one of {SIDES}, got {self.side!r}")
        if self.arm_config not in ARM_CONFIGS:
            problems.append(f"arm_config must be one of {ARM_CONFIGS}, got {self.arm_config!r}")
        if self.traj_type not in TRAJ_TYPES:
            problems.append(f"traj_type must be one of {TRAJ_TYPES}, got {self.traj_type!r}")
        if not self.calib_version:
            problems.append("calib_version is mandatory — a run without it cannot be pooled later")
        if self.sample_rate_hz <= 0:
            problems.append("sample_rate_hz must be positive")
        return problems


@dataclass
class Sample:
    """
    One row. Every field is optional except `t` so that a channel which is not
    fitted on a given run simply is not written, rather than being written as
    zeros that later read as real measurements.

    `t` is seconds against the MASTER clock (the Teensy), never against the
    host clock of whichever machine happened to log the row.
    """

    t: float
    # robot state, as MEASURED
    q: Sequence[float] | None = None            # 6 joint angles, rad
    qd: Sequence[float] | None = None           # 6 joint velocities, rad/s
    tcp_pos: Sequence[float] | None = None      # x y z, m, robot base frame
    tcp_rot: Sequence[float] | None = None      # rotation vector rx ry rz, rad
    # robot state, as COMMANDED — the controller's own setpoint stream.
    #
    # THIS IS WHAT THE SIMULATOR MUST BE FED, and it is the field the whole
    # comparison hinges on. Replaying the MEASURED trajectory in simulation
    # produces a gap of zero by construction: the simulator is being told the
    # answer. Reconstructing the command from the waypoints instead is
    # ambiguous, because a UR generates its own joint trajectory from a
    # Cartesian target, with its own blending, its own acceleration limits and
    # whatever the speed slider was set to at the time — none of which is
    # recoverable after the fact.
    #
    # `target_q` is that generated trajectory, straight from the controller at
    # the control rate. Give the simulator this, let it produce its own
    # `q`, and the difference against the real `q` is the plant gap: friction,
    # drive flexibility, payload inertia. That is a measurement. Anything else
    # is a number whose meaning depends on what you happened to feed in.
    target_q: Sequence[float] | None = None     # 6 commanded joint angles, rad
    target_qd: Sequence[float] | None = None    # 6 commanded joint vels, rad/s
    target_moment: Sequence[float] | None = None  # 6 commanded torques, Nm
    # The controller was asked for target_q but only allowed to run this
    # fraction of it. A run captured at 50% speed scaling executed a different
    # trajectory from the one commanded, and without this nothing downstream
    # can tell.
    speed_scaling: float | None = None
    robot_age_s: float | None = None
    # inertial channels, keyed by unit id ("ind0" industrial, "con0" consumer,
    # "d435i" the camera's own BMI055)
    imu: dict[str, dict[str, Sequence[float]]] = field(default_factory=dict)
    # electromagnetic trackers, keyed by sensor id
    em: dict[str, Sequence[float]] = field(default_factory=dict)
    # third modality placeholder (force, if Section 11 resolves that way)
    aux: dict[str, float] = field(default_factory=dict)
    # every other registered modality, keyed by sensor_hub channel id. Kept as
    # a free-form block on purpose: a channel that arrives mid-campaign must
    # not require a schema change, because a schema change splits the campaign
    # into files that cannot be compared with one another.
    sensors: dict[str, dict] = field(default_factory=dict)

    def to_json(self) -> dict:
        d: dict[str, Any] = {"t": round(self.t, 6)}
        if self.q is not None:
            d["q"] = [round(float(v), 6) for v in self.q]
        if self.qd is not None:
            d["qd"] = [round(float(v), 6) for v in self.qd]
        if self.tcp_pos is not None:
            d["tcp_pos"] = [round(float(v), 6) for v in self.tcp_pos]
        if self.tcp_rot is not None:
            d["tcp_rot"] = [round(float(v), 6) for v in self.tcp_rot]
        # The commanded channels. Written when present and omitted when not,
        # like every other optional field, so a run from a cell that cannot
        # report them is still a valid run -- it just is not replayable.
        if self.target_q is not None:
            d["target_q"] = [round(float(v), 6) for v in self.target_q]
        if self.target_qd is not None:
            d["target_qd"] = [round(float(v), 6) for v in self.target_qd]
        if self.target_moment is not None:
            d["target_moment"] = [round(float(v), 4) for v in self.target_moment]
        if self.speed_scaling is not None:
            d["speed_scaling"] = round(float(self.speed_scaling), 4)
        # How old the robot reading in this sample was when it was written.
        # Near zero on a healthy link. Anything above a sample interval means
        # this row repeats the previous one's robot state, and a file full of
        # those is a frozen instant wearing the shape of a trajectory.
        if self.robot_age_s is not None:
            d["robot_age_s"] = round(float(self.robot_age_s), 4)
        if self.imu:
            d["imu"] = self.imu
        if self.em:
            d["em"] = self.em
        if self.aux:
            d["aux"] = self.aux
        if self.sensors:
            d["sensors"] = self.sensors
        return d


class RunWriter:
    """
    Streaming writer. Open it, write the manifest, then push samples as they
    arrive. Flushes every `flush_every` samples so that a crash costs at most
    that many rows rather than the whole run.
    """

    def __init__(self, path: str | Path, manifest: RunManifest, flush_every: int = 250):
        problems = manifest.validate()
        if problems:
            raise ValueError("invalid manifest: " + "; ".join(problems))
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", encoding="utf-8")
        self._fh.write(json.dumps({"_manifest": asdict(manifest)}) + "\n")
        self.manifest = manifest
        self.n = 0
        self._flush_every = flush_every

    def write(self, sample: Sample) -> None:
        self._fh.write(json.dumps(sample.to_json()) + "\n")
        self.n += 1
        if self.n % self._flush_every == 0:
            self._fh.flush()

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.flush()
            self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@dataclass
class Run:
    """A loaded run: manifest plus the samples as plain dicts."""

    manifest: RunManifest
    samples: list[dict]

    @property
    def duration(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return float(self.samples[-1]["t"] - self.samples[0]["t"])

    def series(self, key: str) -> list:
        """Every value of `key`, skipping samples where the channel is absent."""
        return [s[key] for s in self.samples if key in s]

    def times(self, key: str | None = None) -> list[float]:
        if key is None:
            return [float(s["t"]) for s in self.samples]
        return [float(s["t"]) for s in self.samples if key in s]

    def imu_series(self, unit: str, channel: str) -> tuple[list[float], list[list[float]]]:
        """(timestamps, values) for one IMU channel, e.g. ("ind0", "gyro")."""
        ts, vals = [], []
        for s in self.samples:
            block = s.get("imu", {}).get(unit)
            if block and channel in block:
                ts.append(float(s["t"]))
                vals.append([float(v) for v in block[channel]])
        return ts, vals


def read_run(path: str | Path) -> Run:
    """Load one run file. Tolerates a truncated final line from a killed capture."""
    path = Path(path)
    manifest: RunManifest | None = None
    samples: list[dict] = []
    with path.open("r", encoding="utf-8") as fh:
        for lineno, line in enumerate(fh):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                # Only forgivable on the last line of a capture that was killed.
                continue
            if lineno == 0 and "_manifest" in obj:
                manifest = RunManifest(**obj["_manifest"])
                continue
            if "t" in obj:
                samples.append(obj)
    if manifest is None:
        raise ValueError(f"{path} has no manifest on line 0")
    return Run(manifest=manifest, samples=samples)


def read_dataset(root: str | Path, side: str | None = None) -> list[Run]:
    """Every *.jsonl under root, optionally filtered to one side of the gap."""
    runs = []
    for p in sorted(Path(root).rglob("*.jsonl")):
        try:
            r = read_run(p)
        except Exception:
            continue
        if side is None or r.manifest.side == side:
            runs.append(r)
    return runs


def pair_runs(real: Sequence[Run], sim: Sequence[Run]) -> list[tuple[Run, Run]]:
    """
    Match each real run to its simulated counterpart. Pairing is on the full
    cell plus repeat index, because Phase 4 generates the SAME runs from the
    SAME commands — an unpaired run is a setup error, not a research variable.
    """
    index = {
        (r.manifest.cell_key(), r.manifest.repeat_idx): r for r in sim
    }
    pairs = []
    for r in real:
        k = (r.manifest.cell_key(), r.manifest.repeat_idx)
        if k in index:
            pairs.append((r, index[k]))
    return pairs


def unpaired(real: Sequence[Run], sim: Sequence[Run]) -> tuple[list[str], list[str]]:
    """Run ids that failed to pair, in each direction. Worth printing loudly."""
    sim_keys = {(r.manifest.cell_key(), r.manifest.repeat_idx) for r in sim}
    real_keys = {(r.manifest.cell_key(), r.manifest.repeat_idx) for r in real}
    lonely_real = [r.manifest.run_id for r in real
                   if (r.manifest.cell_key(), r.manifest.repeat_idx) not in sim_keys]
    lonely_sim = [r.manifest.run_id for r in sim
                  if (r.manifest.cell_key(), r.manifest.repeat_idx) not in real_keys]
    return lonely_real, lonely_sim
