"""
Phase 1 — the single time master.

Every channel that will be compared against simulation is timestamped against
one clock. Anything else and a temporal misalignment shows up downstream as a
position error, and gets attributed to the sim-to-real gap it is not.

Two jobs here:

  MasterClock      turn each source's own timestamps into master-frame seconds
  tap_alignment    verify it, using one sharp mechanical event seen by every
                   channel, and report the residual spread that goes in the
                   error budget and gets quoted in every later result
"""
from __future__ import annotations

import bisect
import statistics
from dataclasses import dataclass, field


@dataclass
class ChannelOffset:
    """Affine map from a source clock to the master clock: t_master = a*t_src + b."""

    channel: str
    b: float = 0.0          # offset, seconds
    a: float = 1.0          # rate ratio; 1.0 unless the source clock drifts
    residual_s: float = 0.0  # spread of the fit, seconds
    n_events: int = 0

    def to_master(self, t_src: float) -> float:
        return self.a * t_src + self.b


class MasterClock:
    """
    Holds one ChannelOffset per channel.

    The trackers and FusionHub both timestamp in their own software. The plan
    is explicit that the offset between those clocks and the master is to be
    measured rather than assumed zero, so the default here is deliberately
    *not* an identity map you can forget about: a channel with no measured
    offset is flagged, and `unmeasured()` lists them.
    """

    def __init__(self, master_name: str = "teensy"):
        self.master_name = master_name
        self.offsets: dict[str, ChannelOffset] = {
            master_name: ChannelOffset(master_name, 0.0, 1.0, 0.0, -1)
        }

    def set_offset(self, channel: str, b: float, a: float = 1.0,
                   residual_s: float = 0.0, n_events: int = 0) -> None:
        self.offsets[channel] = ChannelOffset(channel, b, a, residual_s, n_events)

    def to_master(self, channel: str, t_src: float) -> float:
        off = self.offsets.get(channel)
        if off is None:
            raise KeyError(f"channel {channel!r} has no measured offset to the master clock")
        return off.to_master(t_src)

    def unmeasured(self, channels) -> list[str]:
        return [c for c in channels if c not in self.offsets]

    def worst_residual(self) -> float:
        vals = [o.residual_s for o in self.offsets.values() if o.n_events >= 0]
        return max(vals) if vals else 0.0

    def as_dict(self) -> dict:
        return {
            "master": self.master_name,
            "channels": {
                k: {"a": v.a, "b": v.b, "residual_s": v.residual_s, "n_events": v.n_events}
                for k, v in self.offsets.items()
            },
            "worst_residual_s": self.worst_residual(),
        }


def fit_offset(master_events: list[float], source_events: list[float],
               channel: str) -> ChannelOffset:
    """
    Least-squares fit of t_master = a*t_src + b from matched event pairs.

    With a single event pair you only get b, which is fine for a short run;
    with three or more spread across the run you also get a, which is what
    catches a source clock that runs slightly fast — the failure mode that
    looks exactly like a velocity-dependent gap.
    """
    n = min(len(master_events), len(source_events))
    if n == 0:
        raise ValueError("no matched events")
    xs = [float(v) for v in source_events[:n]]
    ys = [float(v) for v in master_events[:n]]
    if n == 1:
        return ChannelOffset(channel, b=ys[0] - xs[0], a=1.0, residual_s=0.0, n_events=1)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return ChannelOffset(channel, b=my - mx, a=1.0, residual_s=0.0, n_events=n)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx
    b = my - a * mx
    resid = [y - (a * x + b) for x, y in zip(xs, ys)]
    spread = statistics.pstdev(resid) if n > 1 else 0.0
    return ChannelOffset(channel, b=b, a=a, residual_s=spread, n_events=n)


def detect_tap(ts: list[float], mag: list[float], k: float = 6.0) -> float | None:
    """
    Find the timestamp of one sharp mechanical event — the light tap on the
    carrier from the Phase 1 verification step.

    Returns the time of the first sample exceeding median + k*MAD. MAD rather
    than standard deviation because the tap itself would inflate an SD-based
    threshold and hide the very thing being looked for.
    """
    if len(mag) < 10:
        return None
    med = statistics.median(mag)
    devs = sorted(abs(v - med) for v in mag)
    mad = devs[len(devs) // 2] or 1e-9
    thresh = med + k * 1.4826 * mad
    for t, v in zip(ts, mag):
        if v > thresh:
            return float(t)
    return None


def tap_alignment(channel_events: dict[str, float]) -> dict:
    """
    Given the tap time as seen by each channel (already mapped to master),
    report the spread. This single number is quoted in every later result.
    """
    if len(channel_events) < 2:
        return {"spread_s": 0.0, "n_channels": len(channel_events), "per_channel": channel_events}
    vals = list(channel_events.values())
    ref = statistics.median(vals)
    return {
        "spread_s": max(vals) - min(vals),
        "max_abs_dev_s": max(abs(v - ref) for v in vals),
        "n_channels": len(vals),
        "per_channel": {k: v - ref for k, v in channel_events.items()},
    }


def resample_to(ref_ts: list[float], src_ts: list[float],
                src_vals: list[list[float]]) -> list[list[float]]:
    """
    Linear resample of a vector-valued channel onto reference timestamps.

    Used to put a 200 Hz IMU and a 125 Hz robot state on a common time base
    before differencing them. Clamps rather than extrapolates at the ends,
    because an extrapolated endpoint is indistinguishable from a real gap.
    """
    if not src_ts or not src_vals:
        return []
    dim = len(src_vals[0])
    out = []
    for t in ref_ts:
        if t <= src_ts[0]:
            out.append(list(src_vals[0]))
            continue
        if t >= src_ts[-1]:
            out.append(list(src_vals[-1]))
            continue
        i = bisect.bisect_left(src_ts, t)
        t0, t1 = src_ts[i - 1], src_ts[i]
        v0, v1 = src_vals[i - 1], src_vals[i]
        w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        out.append([v0[d] + (v1[d] - v0[d]) * w for d in range(dim)])
    return out
