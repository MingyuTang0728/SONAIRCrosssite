"""
Phase 0 — inertial ingestion.

Three tiers of inertial unit are supported, on purpose:

  ind0   the industrial unit, streamed out of FusionHub
  con0   a low-cost consumer module on the Teensy (the "accessible tier" —
         so a team without an industrial IMU can still reproduce the benchmark)
  d435i  the camera's own BMI055, which you already own and which is wired up
         the moment the RealSense is plugged in

All three produce the same record shape, so downstream code never branches on
which unit it is reading:

    {"quat": [w,x,y,z], "gyro": [x,y,z] rad/s, "accel": [x,y,z] m/s^2}

FusionHub ingestion has two modes. Live mode listens on a UDP port for the
JSON stream FusionHub can be configured to push; replay mode reads a CSV or
JSONL that FusionHub recorded. Replay is what you use for the first datasets:
it removes the live-integration risk from the critical path, and the schema is
identical either way, so nothing downstream has to change when you switch.
"""
from __future__ import annotations

import csv
import json
import math
import socket
import statistics
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

GRAVITY = 9.80665


# ----------------------------------------------------------------------------
# record helpers
# ----------------------------------------------------------------------------

def make_record(quat=None, gyro=None, accel=None, mag=None) -> dict:
    rec: dict[str, list[float]] = {}
    if quat is not None:
        rec["quat"] = [float(v) for v in quat]
    if gyro is not None:
        rec["gyro"] = [float(v) for v in gyro]
    if accel is not None:
        rec["accel"] = [float(v) for v in accel]
    if mag is not None:
        rec["mag"] = [float(v) for v in mag]
    return rec


def quat_normalise(q):
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [w / n, x / n, y / n, z / n]


# ----------------------------------------------------------------------------
# FusionHub
# ----------------------------------------------------------------------------

# FusionHub's field naming varies with the output profile, so accept the
# spellings seen in practice rather than demanding one.
_QUAT_KEYS = (("qw", "qx", "qy", "qz"), ("w", "x", "y", "z"),
              ("quat_w", "quat_x", "quat_y", "quat_z"))
_GYRO_KEYS = (("gx", "gy", "gz"), ("gyro_x", "gyro_y", "gyro_z"),
              ("wx", "wy", "wz"))
_ACC_KEYS = (("ax", "ay", "az"), ("acc_x", "acc_y", "acc_z"),
             ("accel_x", "accel_y", "accel_z"))
_TIME_KEYS = ("timestamp", "t", "time", "ts", "time_s", "host_time")


def _pick(row: dict, groups) -> list[float] | None:
    lower = {str(k).strip().lower(): v for k, v in row.items()}
    for keys in groups:
        if all(k in lower for k in keys):
            try:
                return [float(lower[k]) for k in keys]
            except (TypeError, ValueError):
                return None
    return None


def parse_fusionhub_row(row: dict) -> tuple[float | None, dict]:
    """
    One FusionHub sample -> (source timestamp, canonical record).

    The timestamp comes back in FusionHub's OWN clock. It is the caller's job
    to push it through MasterClock.to_master() — this function deliberately
    does not pretend the two clocks agree.
    """
    lower = {str(k).strip().lower(): v for k, v in row.items()}
    t_src = None
    for k in _TIME_KEYS:
        if k in lower:
            try:
                t_src = float(lower[k])
                break
            except (TypeError, ValueError):
                continue
    quat = _pick(row, _QUAT_KEYS)
    gyro = _pick(row, _GYRO_KEYS)
    accel = _pick(row, _ACC_KEYS)
    if quat:
        quat = quat_normalise(quat)
    return t_src, make_record(quat=quat, gyro=gyro, accel=accel)


def read_fusionhub_file(path: str | Path) -> list[tuple[float, dict]]:
    """Replay mode: read a FusionHub CSV or JSONL export."""
    path = Path(path)
    out: list[tuple[float, dict]] = []
    if path.suffix.lower() in (".jsonl", ".ndjson", ".json"):
        with path.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(row, dict):
                    t, rec = parse_fusionhub_row(row)
                    if t is not None and rec:
                        out.append((t, rec))
    else:
        with path.open("r", encoding="utf-8", newline="") as fh:
            for row in csv.DictReader(fh):
                t, rec = parse_fusionhub_row(row)
                if t is not None and rec:
                    out.append((t, rec))
    out.sort(key=lambda p: p[0])
    return out


class FusionHubUdpSource:
    """
    Live mode: listen for FusionHub's JSON-over-UDP output.

    Runs a daemon thread and hands every parsed sample to `on_sample(t_src, rec)`.
    Deliberately never blocks the caller and never raises on a malformed
    datagram — during a four-week campaign, one bad packet must not end a run.
    """

    def __init__(self, port: int = 5005, host: str = "0.0.0.0",
                 unit_id: str = "ind0",
                 on_sample: Callable[[float, dict], None] | None = None):
        self.port = port
        self.host = host
        self.unit_id = unit_id
        self.on_sample = on_sample
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.last: tuple[float, dict] | None = None
        self.n_packets = 0
        self.n_bad = 0

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name=f"fusionhub-{self.unit_id}")
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                row = json.loads(data.decode("utf-8", "ignore"))
            except Exception:
                self.n_bad += 1
                continue
            if not isinstance(row, dict):
                self.n_bad += 1
                continue
            t_src, rec = parse_fusionhub_row(row)
            if t_src is None:
                # No timestamp in the stream: fall back to arrival time and
                # record that fact, because arrival time carries network jitter.
                t_src = time.time()
                rec.setdefault("_arrival_time_used", [1.0])
            if not rec:
                self.n_bad += 1
                continue
            self.n_packets += 1
            self.last = (t_src, rec)
            if self.on_sample:
                try:
                    self.on_sample(t_src, rec)
                except Exception:
                    pass

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=1.0)

    def health(self) -> dict:
        return {"unit": self.unit_id, "packets": self.n_packets,
                "bad": self.n_bad, "listening": self._thread is not None
                and self._thread.is_alive()}


# ----------------------------------------------------------------------------
# Phase 0 characterisation — the numbers every later error claim rests on
# ----------------------------------------------------------------------------

@dataclass
class NoiseFloor:
    """Result of the one-hour stationary log. Per-axis, per-channel."""

    unit: str
    gyro_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro_noise: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel_noise: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    n_samples: int = 0
    duration_s: float = 0.0
    dropped_frac: float = 0.0

    def orientation_noise_deg(self) -> float:
        """Worst-axis gyro noise expressed in degrees per second."""
        return math.degrees(max(self.gyro_noise)) if self.gyro_noise else 0.0

    def as_dict(self) -> dict:
        d = self.__dict__.copy()
        d["orientation_noise_deg_s"] = self.orientation_noise_deg()
        return d


def stationary_stats(samples: list[tuple[float, dict]], unit: str = "ind0",
                     expected_hz: float | None = None) -> NoiseFloor:
    """
    Bias and noise from a stationary log.

    Accelerometer bias is reported after removing the gravity vector's
    magnitude from the norm, not per axis, because a clamped unit's axes are
    not aligned with gravity and a per-axis "bias" would mostly be measuring
    how the unit was sitting.
    """
    if not samples:
        return NoiseFloor(unit=unit)
    ts = [t for t, _ in samples]
    duration = ts[-1] - ts[0] if len(ts) > 1 else 0.0

    gyro = [r["gyro"] for _, r in samples if "gyro" in r]
    accel = [r["accel"] for _, r in samples if "accel" in r]

    def per_axis(vecs, fn):
        if not vecs:
            return [0.0, 0.0, 0.0]
        return [fn([v[i] for v in vecs]) for i in range(3)]

    nf = NoiseFloor(unit=unit, n_samples=len(samples), duration_s=duration)
    if gyro:
        nf.gyro_bias = per_axis(gyro, statistics.fmean)
        nf.gyro_noise = per_axis(gyro, lambda c: statistics.pstdev(c) if len(c) > 1 else 0.0)
    if accel:
        nf.accel_bias = per_axis(accel, statistics.fmean)
        nf.accel_noise = per_axis(accel, lambda c: statistics.pstdev(c) if len(c) > 1 else 0.0)
        norms = [math.sqrt(sum(v * v for v in a)) for a in accel]
        # Store the scale error on the gravity norm in the unused 4th slot's place
        nf.accel_bias = nf.accel_bias + [statistics.fmean(norms) - GRAVITY]

    if expected_hz and duration > 0:
        expected = expected_hz * duration
        nf.dropped_frac = max(0.0, 1.0 - len(samples) / expected) if expected > 0 else 0.0
    return nf


def sample_rate_stability(samples: list[tuple[float, dict]]) -> dict:
    """
    Does the reported sample rate drift? Ten minutes stationary is enough to
    see it. A drifting rate is the failure that turns into a fake
    velocity-dependent gap in Phase 5, so it is checked before anything else.
    """
    ts = [t for t, _ in samples]
    if len(ts) < 3:
        return {"n": len(ts), "mean_hz": 0.0, "jitter_ms": 0.0, "max_gap_ms": 0.0}
    dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    if not dts:
        return {"n": len(ts), "mean_hz": 0.0, "jitter_ms": 0.0, "max_gap_ms": 0.0}
    mean_dt = statistics.fmean(dts)
    return {
        "n": len(ts),
        "mean_hz": 1.0 / mean_dt if mean_dt > 0 else 0.0,
        "jitter_ms": statistics.pstdev(dts) * 1000.0 if len(dts) > 1 else 0.0,
        "max_gap_ms": max(dts) * 1000.0,
        "drift_ppm": ((dts[-1] - dts[0]) / mean_dt * 1e6) if mean_dt > 0 else 0.0,
    }


def tumble_check(positions: list[list[float]]) -> dict:
    """
    Six-position tumble test, gravity as reference.

    Each entry is the mean accelerometer vector held in one of six orientations.
    A correctly scaled triad gives |a| = g in every position; the spread of
    |a| across the six is the scale-factor error, and the departure of opposite
    pairs from cancelling is the bias.
    """
    if len(positions) < 2:
        return {"n_positions": len(positions), "scale_error": 0.0, "bias_est": [0.0, 0.0, 0.0]}
    norms = [math.sqrt(sum(v * v for v in p)) for p in positions]
    mean_norm = statistics.fmean(norms)
    bias = [statistics.fmean([p[i] for p in positions]) for i in range(3)]
    return {
        "n_positions": len(positions),
        "mean_norm": mean_norm,
        "scale_error": (mean_norm - GRAVITY) / GRAVITY,
        "norm_spread": max(norms) - min(norms),
        "bias_est": bias,
    }
