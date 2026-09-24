"""
imu_link.py — get inertial data OUT of FusionHub (and anything like it).

The previous integration assumed one transport and one wire format: FusionHub
pushing JSON to UDP 5005. When that assumption is wrong there is nothing to
debug against, because a UDP listener that receives nothing looks exactly like
a UDP listener pointed at the wrong port, a FusionHub that is not streaming,
and a firewall. That ambiguity is the whole problem, so this module removes it:

  DISCOVER   bind every plausible port at once and report which ones receive
             anything, plus the first bytes verbatim. After ten seconds you
             know whether data is arriving at all, and if so, where.
  SNIFF      classify a payload — JSON, CSV, key=value, whitespace columns,
             Xsens ASCII, or binary — and show which fields were recognised as
             inertial and which were ignored. A stream that arrives but parses
             to nothing is a different fault from a stream that never arrives,
             and it must LOOK different.
  TRANSPORTS udp-listen, tcp-client, tcp-listen, http-poll, websocket-client,
             serial, and file-tail. FusionHub can be configured for several of
             these; which one is available depends on the licence and version,
             so the integration supports all of them rather than betting on one.

Every transport is import-tolerant and non-fatal: an absent `pyserial` or
`websockets` disables that one transport and reports why, and a malformed
packet increments a counter instead of ending a four-week campaign.

The output contract is one shape, whatever came in:

    (t_src, {"quat": [w,x,y,z], "gyro": [x,y,z] rad/s, "accel": [x,y,z] m/s^2})

t_src is in the SOURCE's clock. Converting it to the master clock is the
caller's job, because this module has no way to measure that offset and
pretending it is zero is how a timing error becomes a position error.
"""
from __future__ import annotations

import json
import logging
import struct
import math
import os
import re
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable

log = logging.getLogger("imu_link")

try:
    from sonair_benchmark.attitude import q_to_euler_deg
except Exception:                                   # pragma: no cover
    def q_to_euler_deg(q):
        w, x, y, z = q
        sr, cr = 2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)
        sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        sy, cy = 2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        return [math.degrees(math.atan2(sr, cr)), math.degrees(math.asin(sp)),
                math.degrees(math.atan2(sy, cy))]

try:
    from sonair_benchmark.imu import parse_fusionhub_row, quat_normalise
    _HAS_BENCH = True
except Exception:                                   # pragma: no cover
    _HAS_BENCH = False

    def quat_normalise(q):
        w, x, y, z = (float(v) for v in q)
        n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
        return [w / n, x / n, y / n, z / n]

    def parse_fusionhub_row(row):                   # minimal fallback
        return None, {}

try:
    import serial as pyserial                       # type: ignore
    _HAS_SERIAL = True
    _SERIAL_ERR = ""
except Exception as e:                              # noqa: BLE001
    pyserial = None
    _HAS_SERIAL = False
    _SERIAL_ERR = str(e)

try:
    import urllib.request as _urlreq
    _HAS_HTTP = True
except Exception:                                   # pragma: no cover
    _HAS_HTTP = False

try:
    import websockets
    _HAS_WS = True
    _WS_ERR = ""
except Exception as e:                              # noqa: BLE001
    websockets = None
    _HAS_WS = False
    _WS_ERR = str(e)

try:
    import zmq
    _HAS_ZMQ = True
    _ZMQ_ERR = ""
except Exception as e:                              # noqa: BLE001
    zmq = None
    _HAS_ZMQ = False
    _ZMQ_ERR = str(e)


# Ports FusionHub and its neighbours are seen on in the field. Discovery binds
# all of them; the cost of one extra UDP socket is nothing next to an afternoon
# spent guessing.
CANDIDATE_UDP_PORTS = [5005, 5006, 5555, 6000, 8000, 8888, 9000, 9001, 9763, 4001]
CANDIDATE_TCP_PORTS = [5005, 8080, 9000, 9001, 502]

DEG = math.pi / 180.0
GRAVITY = 9.80665


def pip_hint(package: str) -> str:
    """
    Name the interpreter, in a form the operator's shell will actually run.

    A workstation has several Pythons — the system one, a virtual environment,
    whatever the IDE picked — and a bare "pip install pyzmq" lands in whichever
    is first on PATH, routinely not the one running this agent. So the path is
    quoted in.

    But a quoted path is not enough on Windows: PowerShell parses a command
    that BEGINS with a quoted string as a string expression, not as a command,
    and fails with "unexpected token '-m'". The call operator `&` is what makes
    it a command. Printing the POSIX form to a PowerShell user produces an
    error message that looks like the advice was wrong, which is worse than
    giving no path at all.
    """
    exe = sys.executable or "python"
    if os.name == "nt":
        return 'run:  & "%s" -m pip install %s' % (exe, package)
    return 'run:  "%s" -m pip install %s' % (exe, package)


# ---------------------------------------------------------------------------
# wire-format sniffing
# ---------------------------------------------------------------------------

_NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

# Name fragments that identify a column, checked longest-first so that
# "quat_w" is not matched by the bare "w" rule.
_FIELD_HINTS = [
    (("quatw", "quat_w", "qw", "q0", "orientation_w"), ("quat", 0)),
    (("quatx", "quat_x", "qx", "q1", "orientation_x"), ("quat", 1)),
    (("quaty", "quat_y", "qy", "q2", "orientation_y"), ("quat", 2)),
    (("quatz", "quat_z", "qz", "q3", "orientation_z"), ("quat", 3)),
    (("gyrox", "gyro_x", "gyr_x", "gx", "wx", "angularvelocityx",
      "angular_velocity_x", "rate_x"), ("gyro", 0)),
    (("gyroy", "gyro_y", "gyr_y", "gy", "wy", "angularvelocityy",
      "angular_velocity_y", "rate_y"), ("gyro", 1)),
    (("gyroz", "gyro_z", "gyr_z", "gz", "wz", "angularvelocityz",
      "angular_velocity_z", "rate_z"), ("gyro", 2)),
    (("accelx", "accel_x", "acc_x", "ax", "accx", "freeaccx",
      "acceleration_x", "linearaccelerationx"), ("accel", 0)),
    (("accely", "accel_y", "acc_y", "ay", "accy", "freeaccy",
      "acceleration_y", "linearaccelerationy"), ("accel", 1)),
    (("accelz", "accel_z", "acc_z", "az", "accz", "freeaccz",
      "acceleration_z", "linearaccelerationz"), ("accel", 2)),
    (("magx", "mag_x", "mx", "magneticfieldx"), ("mag", 0)),
    (("magy", "mag_y", "my", "magneticfieldy"), ("mag", 1)),
    (("magz", "mag_z", "mz", "magneticfieldz"), ("mag", 2)),
    (("roll",), ("euler", 0)),
    (("pitch",), ("euler", 1)),
    (("yaw", "heading"), ("euler", 2)),
]

_TIME_HINTS = ("timestamp", "time", "ts", "t", "sampletimefine", "host_time",
               "time_s", "utc", "packetcounter")


def _norm_key(k: str) -> str:
    return re.sub(r"[^a-z0-9_]", "", str(k).strip().lower())


def _classify_key(key: str):
    """
    Match a column name against the hint table, most-specific first.

    Nested JSON flattens to paths like "imu_accel_x" and "sensor_0_gyro_y", so
    the whole key is tried first and then progressively shorter tails of it.
    Matching on the tail rather than on a substring is what stops "max_gz"
    (a configured limit) being read as the gyro's z axis.
    """
    k = _norm_key(key)
    parts = [p for p in k.split("_") if p]
    candidates = [k]
    for take in (3, 2, 1):
        if len(parts) >= take:
            candidates.append("_".join(parts[-take:]))
            candidates.append("".join(parts[-take:]))
    seen = set()
    for cand in candidates:
        if cand in seen:
            continue
        seen.add(cand)
        for names, target in _FIELD_HINTS:
            if cand in names:
                return target
    return _classify_by_family(k)


_AXIS_INDEX = {"w": 0, "x": 1, "y": 2, "z": 3,
               "0": 0, "1": 1, "2": 2, "3": 3}

_FAMILIES = (
    (("quat", "orient", "rotation", "attitude"), "quat"),
    (("gyr", "angular", "rate", "turnrate"), "gyro"),
    (("acc",), "accel"),
    (("mag", "magnet"), "mag"),
)


def _classify_by_family(k: str):
    """
    The long-form fallback: "quaternion_w", "gyroscope_x", "accelerometer_z",
    "linearAcceleration.y" — a family name plus a trailing axis.

    Split as base + axis rather than by substring so a key like "max_gz" (a
    configured limit) or "gyro_accuracy" cannot be read as a data channel.
    """
    m = re.match(r"^(.*?)_?([wxyz0123])$", k)
    if not m:
        return None
    base, axis = m.group(1), m.group(2)
    if not base or "accur" in base or "cov" in base or "status" in base:
        return None
    for frags, family in _FAMILIES:
        if any(f in base for f in frags):
            idx = _AXIS_INDEX[axis]
            if family != "quat":
                if axis == "w":
                    return None
                idx = {"x": 0, "y": 1, "z": 2,
                       "0": 0, "1": 1, "2": 2}.get(axis)
                if idx is None:
                    return None
            return (family, idx)
    return None


def _flatten(obj, prefix: str = "", out: dict | None = None) -> dict:
    """
    FusionHub's JSON is sometimes nested ({"imu":{"accel":{"x":..}}}) and
    sometimes flat. Flatten so one parser handles both, and keep the leaf name
    as well as the dotted path so either spelling can match a hint.
    """
    if out is None:
        out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            key = f"{prefix}{k}"
            if isinstance(v, (dict, list)):
                _flatten(v, key + "_", out)
            else:
                out[key] = v
                if prefix and key.split("_")[-1] not in out:
                    out[k] = v
    elif isinstance(obj, list):
        # A bare triple or quad under a recognised name: accel:[x,y,z]
        base = prefix.rstrip("_")
        names = "wxyz" if len(obj) == 4 else "xyz"
        for i, v in enumerate(obj[:4]):
            if not isinstance(v, (dict, list)):
                out[f"{base}_{names[i]}"] = v
    return out


def _name_binary(raw: bytes) -> str:
    """
    Name the binary format rather than calling everything "binary".

    Each of these has a different fix, and "binary" has none: ZMTP means the
    transport is wrong (a raw socket pointed at a ZeroMQ endpoint), while
    MessagePack or CBOR mean the transport is right and the output profile is
    wrong.
    """
    if raw[:1] == b"\xff" and len(raw) >= 10 and raw[9:10] == b"\x7f":
        return "zmtp"                     # ZeroMQ greeting
    if raw[:4] == b"\xff\x00\x00\x00" or raw[:1] == b"\x04":
        return "zmtp"                     # ZMTP frame headers
    if raw[:1] in (b"\x80", b"\x81", b"\xde", b"\xdf"):
        return "msgpack"
    if raw[:1] in (b"\xa0", b"\xbf") or raw[:1] == b"\xd9":
        return "cbor"
    if raw[:2] == b"\x1f\x8b":
        return "gzip"
    return "binary"


def _binary_advice(fmt: str) -> str:
    if fmt == "zmtp":
        return ("This is ZeroMQ's own protocol, not your data. FusionHub's "
                "External Output is a ZeroMQ publisher, so it needs the "
                "\"External Output\" transport rather than a plain TCP one — "
                "a raw socket connects successfully and then receives this.")
    if fmt in ("msgpack", "cbor"):
        return (f"The transport is working — this is {fmt.upper()}-encoded "
                "data, not text. Change FusionHub's output format to JSON, "
                "which is what is parsed here.")
    if fmt == "gzip":
        return ("The data is compressed. Turn compression off on the output "
                "node, or set the format to plain JSON.")
    return ("This is not text. Set FusionHub's output format to JSON or CSV — "
            "binary sensor protocols are not parsed here.")


# ---------------------------------------------------------------------------
# protobuf
# ---------------------------------------------------------------------------

def _pb_varint(buf, i):
    v = shift = 0
    while True:
        if i >= len(buf):
            raise IndexError("truncated varint")
        b = buf[i]
        i += 1
        v |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return v, i
        if shift > 70:
            raise ValueError("varint too long")


def protobuf_walk(buf, path="", depth=0, out=None, lenient=False):
    """
    Walk Protocol Buffers wire format WITHOUT a .proto file.

    The wire format carries field numbers, wire types and lengths but no
    names and no type names, so a schema-less walk gets the structure and the
    values and nothing else. That turns out to be enough: what an inertial
    message contains can be recovered from the numbers themselves, which is
    what `ProtobufImu` below does. Waiting for a .proto file that the vendor
    may not publish is not a plan.

    Raises on anything that is not valid protobuf, so callers can use a
    successful walk as the format test.
    """
    if out is None:
        out = []
    if depth > 6:
        return out
    i = 0
    while i < len(buf):
        # One guard around the WHOLE field, not just the key. A truncated
        # value varint escaped the old key-only guard, so lenient mode still
        # raised on exactly the packets it existed to tolerate — and the
        # caller discarded a message whose readings it had already decoded.
        try:
            i = _pb_field(buf, i, path, depth, out, lenient)
        except Exception:
            if lenient:
                return out
            raise
    return out


def _pb_field(buf, i, path, depth, out, lenient):
    """Read one field, append what it holds, and return the new offset."""
    key, i = _pb_varint(buf, i)
    field, wt = key >> 3, key & 7
    if field == 0:
        raise ValueError("field number 0 is not valid protobuf")
    p = f"{path}.{field}" if path else str(field)
    if wt == 0:
        v, i = _pb_varint(buf, i)
        out.append((p, "varint", v))
    elif wt == 1:
        if i + 8 > len(buf):
            raise ValueError("truncated 64-bit field")
        out.append((p, "double", struct.unpack_from("<d", buf, i)[0]))
        i += 8
    elif wt == 2:
        n, i = _pb_varint(buf, i)
        if i + n > len(buf):
            if not lenient:
                raise ValueError("truncated length-delimited field")
            # A preview is a truncated message by definition. Descend into the
            # part that IS there: that prefix is where the vectors are, and
            # showing nothing would defeat the point of a preview.
            try:
                out.extend(protobuf_walk(buf[i:], p, depth + 1, [], True))
            except Exception:
                pass
            return len(buf)
        sub = buf[i:i + n]
        i += n
        try:
            nested = protobuf_walk(sub, p, depth + 1, [], lenient)
        except Exception:
            out.append((p, "bytes", sub))
            return i
        if nested:
            out.extend(nested)
        else:
            out.append((p, "bytes", sub))
    elif wt == 5:
        if i + 4 > len(buf):
            raise ValueError("truncated 32-bit field")
        out.append((p, "float", struct.unpack_from("<f", buf, i)[0]))
        i += 4
    else:
        raise ValueError(f"wire type {wt} is not valid protobuf")
    return i


def _wrap180(d: float) -> float:
    """Angle difference folded into -180..180, so 179 and -179 are 2 apart."""
    d = (float(d) + 180.0) % 360.0 - 180.0
    return d


def _pb_vectors(entries):
    """
    Group consecutive numeric fields sharing a parent into vectors.

    FusionHub packs each 3-axis reading as its own sub-message of three
    doubles, so the grouping is just "same parent path". A quaternion arrives
    the same way with four.
    """
    groups: dict[str, list] = {}
    order: list[str] = []
    for path, kind, value in entries:
        if kind not in ("double", "float"):
            continue
        parent = path.rpartition(".")[0] or path
        if parent not in groups:
            groups[parent] = []
            order.append(parent)
        groups[parent].append(float(value))
    return [(p, groups[p]) for p in order if len(groups[p]) in (3, 4)]


class ProtobufImu:
    """
    Read a protobuf inertial stream with no schema, by recognising the
    physics rather than the field names.

    FusionHub's External Output publishes protobuf. There are no field names
    on the wire and the .proto is not to hand, so the field NUMBERS have to be
    mapped to meanings some other way. The measurements themselves do it:

      a 4-vector of unit length            is an orientation quaternion
      a 3-vector averaging 9.8             is acceleration in m/s^2
      a 3-vector averaging 1.0             is acceleration in g
      a 3-vector averaging 20-90           is a magnetometer in microtesla
      whatever is left                     is the gyroscope

    Gravity is the anchor: a sensor on a bench or on an arm is under 1 g on
    average whatever else it is doing, and nothing else in an inertial message
    sits at that magnitude. Direction is not used, only magnitude, so it holds
    however the unit is mounted.

    Evidence is accumulated over several samples and then the mapping is
    LOCKED to the field numbers it found, because from then on the field
    numbers are exact and the physics is only a heuristic. The mapping is
    reported so it can be checked, and a wrong one is visible immediately:
    orientation that does not move when the sensor moves.
    """

    DECIDE_AFTER = 25

    def __init__(self):
        self.mapping: dict[str, str] | None = None
        self.time_path: str | None = None
        self.time_scale = 1.0
        self.n = 0
        self.n_decoded = 0
        self.partial = False
        self.partial_reason = ""
        self._norm_sum: dict[str, float] = {}
        self._euler_sum: dict[str, float] = {}
        self._euler_n: dict[str, int] = {}
        self._rate_stats: dict[str, list] = {}
        self._rate_track: dict[str, float] = {}
        self._prev_quat = None
        self._norm_n: dict[str, int] = {}
        self._len: dict[str, int] = {}
        self.error = ""

    # -- decoding ----------------------------------------------------------
    def feed(self, raw):
        # Lenient, deliberately. A real packet carries more than the inertial
        # message — a device name, a serial number, trailing fields this
        # decoder has never seen — and one field it cannot walk must not throw
        # away the accelerometer, gyroscope and orientation that were already
        # read out of the same packet. Strict parsing here meant every packet
        # arrived and every packet was discarded, which on screen is
        # indistinguishable from no packets arriving at all.
        #
        # The quality bar replaces the strictness: several entries AND at
        # least one 3- or 4-vector. Random bytes do not clear that.
        try:
            entries = protobuf_walk(raw, lenient=True)
        except Exception as e:                      # noqa: BLE001
            self.error = str(e)
            return None, {}
        vectors = _pb_vectors(entries)
        if len(entries) < 3 or not vectors:
            return None, {}
        self.n_decoded += 1
        try:
            protobuf_walk(raw)
            self.partial = False
        except Exception as e:                      # noqa: BLE001
            # Worth knowing, not worth failing on: it says this decoder does
            # not understand the whole message, which is the thing to report
            # if a channel ever turns out to be missing.
            self.partial = True
            self.partial_reason = str(e)

        if self.mapping is None:
            self._observe(entries, vectors)
            if self.n < self.DECIDE_AFTER:
                return None, {}
            self._decide(vectors)

        rec: dict[str, list] = {}
        for path, values in vectors:
            role = self.mapping.get(path)
            if role == "quat":
                rec["quat"] = quat_normalise(values[:4])
            elif role == "accel":
                rec["accel"] = values[:3]
            elif role == "accel_g":
                rec["accel"] = [v * GRAVITY for v in values[:3]]
            elif role == "gyro":
                rec["gyro"] = values[:3]
            elif role == "mag":
                rec["mag"] = values[:3]
            elif role == "euler":
                # Kept for comparison, never fed to the estimator: it is the
                # same measurement as the quaternion, so treating it as an
                # independent channel would double-count it.
                rec["euler_device_deg"] = values[:3]

        t_src = None
        if self.time_path:
            for path, kind, value in entries:
                if path == self.time_path and kind == "varint":
                    t_src = float(value) * self.time_scale
                    break
        return t_src, rec

    # -- learning ----------------------------------------------------------
    def _observe(self, entries, vectors):
        self.n += 1
        for path, values in vectors:
            n = math.sqrt(sum(v * v for v in values))
            self._norm_sum[path] = self._norm_sum.get(path, 0.0) + n
            self._norm_n[path] = self._norm_n.get(path, 0) + 1
            self._len[path] = len(values)

        # Is any 3-vector simply the orientation written out as Euler angles?
        # FusionHub sends both, and the Euler triple has no magnitude that
        # marks it out — its numbers span +/-180, so it is not gravity, not a
        # magnetometer, and it is NOT the gyroscope even though that is what
        # anything left over would otherwise be taken for. Getting this wrong
        # does not merely lose a channel: the fallback overwrites the real
        # gyroscope with angles, and 180 "deg/s" of turn rate on a unit lying
        # still is worse than no reading.
        #
        # Deciding it is easy once asked properly: convert the quaternion and
        # see which triple matches. Degrees are degrees; this needs no bands.
        quat = None
        for path, values in vectors:
            if len(values) == 4 and abs(
                    math.sqrt(sum(v * v for v in values)) - 1.0) < 0.05:
                quat = quat_normalise(values)
                break
        if quat is None:
            return
        euler = q_to_euler_deg(quat)
        # Only judge on samples where the orientation is well away from level.
        # Near zero EVERYTHING matches: a gyroscope reading 0.1 rad/s and an
        # Euler triple of [0.1, 0.0, 0.0] degrees are the same three numbers,
        # and a rule that cannot tell them apart there will happily call the
        # gyroscope an orientation. Away from level they are unmistakable.
        # How well does each 3-vector's size follow the orientation's turn
        # rate? Kept for the tie-break above, and measured as the spread of
        # the ratio between them: for the real gyroscope that ratio is a
        # constant (1, or 57.3 if it reports degrees), whatever the motion.
        if self._prev_quat is not None:
            w = 2.0 * math.acos(max(-1.0, min(1.0, abs(
                sum(a * b for a, b in zip(self._prev_quat, quat))))))
            if w > 1e-4:
                for path, values in vectors:
                    if len(values) != 3:
                        continue
                    m = math.sqrt(sum(v * v for v in values))
                    r = m / w
                    st = self._rate_stats.setdefault(path, [0.0, 0.0, 0])
                    st[0] += r
                    st[1] += r * r
                    st[2] += 1
                    if st[2] >= 8:
                        mean = st[0] / st[2]
                        var = max(0.0, st[1] / st[2] - mean * mean)
                        # coefficient of variation: scale-free, so it does not
                        # care whether the unit reports radians or degrees
                        self._rate_track[path] = (math.sqrt(var) / mean
                                                  if mean > 1e-9 else 1e9)
        self._prev_quat = quat

        if max(abs(v) for v in euler) < 10.0:
            return
        for path, values in vectors:
            if len(values) != 3:
                continue
            d = sum(abs(_wrap180(a - b)) for a, b in zip(values, euler)) / 3.0
            self._euler_sum[path] = self._euler_sum.get(path, 0.0) + d
            self._euler_n[path] = self._euler_n.get(path, 0) + 1
        if self.time_path is None:
            # The biggest varint that looks like a wall-clock time. Nanosecond
            # epochs are ~1.8e18 now, microseconds ~1.8e15, milliseconds
            # ~1.8e12 — the magnitude names the unit, so the scale comes from
            # the same observation rather than from an assumption.
            best = None
            for path, kind, value in entries:
                if kind != "varint" or value < 1e11:
                    continue
                if best is None or value > best[1]:
                    best = (path, value)
            if best:
                self.time_path = best[0]
                v = best[1]
                self.time_scale = (1e-9 if v > 1e17 else
                                   1e-6 if v > 1e14 else
                                   1e-3 if v > 1e11 else 1.0)

    def _decide(self, vectors):
        # Classify every path seen SO FAR, not just the ones in the packet
        # that happened to trip the counter. A stream can interleave message
        # shapes, and a mapping built from one packet would then silently drop
        # whatever that packet lacked.
        mean = {p: self._norm_sum[p] / max(1, self._norm_n[p])
                for p in self._norm_sum}
        mapping: dict[str, str] = {}

        # Orientation first: a unit-length 4-vector is unambiguous.
        for path in mean:
            if self._len.get(path) == 4 and abs(mean[path] - 1.0) < 0.05:
                mapping[path] = "quat"

        # Then the Euler copy of it, BEFORE any magnitude band gets a look.
        # It is identified by agreement with the quaternion, which is exact,
        # and taking it out first is what stops it being mistaken for the
        # gyroscope by process of elimination.
        for path, total in self._euler_sum.items():
            n = self._euler_n.get(path, 0)
            if n >= 5 and total / n < 2.0 and path not in mapping:
                mapping[path] = "euler"

        threes = [p for p in mean
                  if self._len.get(p) == 3 and p not in mapping]

        # Gravity, in whichever unit it arrived in. Closest to the expected
        # magnitude wins, so a second vector that merely overlaps the band
        # cannot steal it.
        def claim(band, role):
            cands = [p for p in threes
                     if p not in mapping and band[0] <= mean.get(p, 0) <= band[1]]
            if not cands:
                return
            target = (band[0] + band[1]) / 2.0
            mapping[min(cands, key=lambda p: abs(mean[p] - target))] = role

        claim((8.5, 11.5), "accel")
        if "accel" not in mapping.values():
            claim((0.85, 1.15), "accel_g")
        claim((15.0, 90.0), "mag")

        left = [p for p in threes if p not in mapping]
        if len(left) <= 1:
            for p in left:
                mapping[p] = "gyro"
        else:
            # Two or more candidates and no magnitude tells them apart. The
            # gyroscope is the one whose size follows how fast the orientation
            # is actually turning; nothing else in the message does.
            best, best_score = None, None
            for p in left:
                score = self._rate_track.get(p)
                if score is None:
                    continue
                if best_score is None or score < best_score:
                    best, best_score = p, score
            for p in left:
                mapping[p] = "gyro" if p == (best or left[0]) else "unknown"
        self.mapping = mapping

    # -- reporting ---------------------------------------------------------
    def status(self) -> dict:
        if self.mapping is None:
            return {"protobuf_mapping": "working it out",
                    "protobuf_samples": self.n,
                    "protobuf_decoded": self.n_decoded}
        names = {"quat": "orientation", "accel": "acceleration (m/s^2)",
                 "accel_g": "acceleration (g)", "gyro": "turn rate",
                 "mag": "magnetic field",
                 "euler": "the orientation again, as angles (not used)",
                 "unknown": "not identified — not used"}
        out_extra = {}
        if self.partial:
            out_extra["protobuf_partial"] = (
                "Part of each packet could not be read: " + self.partial_reason
                + ". The channels listed were still decoded; if one you expect "
                  "is missing, that is where it went.")
        return {
            "protobuf_mapping": {f"field {p}": names.get(r, r)
                                 for p, r in sorted(self.mapping.items())},
            "protobuf_decoded": self.n_decoded,
            **out_extra,
            "protobuf_time_field": self.time_path,
            "protobuf_time_unit": {1e-9: "nanoseconds", 1e-6: "microseconds",
                                   1e-3: "milliseconds",
                                   1.0: "seconds"}.get(self.time_scale, "?"),
            "protobuf_samples": self.n,
        }


def parse_payload(payload, gyro_units: str = "auto") -> tuple[float | None, dict, str]:
    """
    One datagram or line -> (t_src, canonical record, format name).

    `gyro_units="auto"` decides between deg/s and rad/s by magnitude: a rate
    channel whose 95th-percentile magnitude sits above ~7 is degrees, because a
    7 rad/s (400 deg/s) hand movement is not something an arm-mounted unit sees
    continuously. It is a heuristic, it is REPORTED in the health block, and it
    can be pinned once you know which your unit sends.
    """
    if isinstance(payload, (bytes, bytearray)):
        try:
            text = payload.decode("utf-8", "strict")
        except UnicodeDecodeError:
            return None, {}, "binary"
    else:
        text = str(payload)
    text = text.strip()
    if not text:
        return None, {}, "empty"

    row = None
    fmt = ""
    if text[0] in "{[":
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            return None, {}, "json-malformed"
        if isinstance(obj, list):
            obj = obj[0] if obj and isinstance(obj[0], dict) else {}
        row = _flatten(obj)
        fmt = "json"
    elif "=" in text and "," in text:
        row = {}
        for part in text.split(","):
            if "=" in part:
                k, _, v = part.partition("=")
                row[k.strip()] = v.strip()
        fmt = "keyvalue"
    else:
        return None, {}, "unlabelled"

    return (*_row_to_record(row, gyro_units), fmt)


def _row_to_record(row: dict, gyro_units: str = "auto"):
    acc: dict[str, list] = {}
    t_src = None
    for k, v in row.items():
        if v is None or isinstance(v, (dict, list)):
            continue
        try:
            fv = float(v)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(fv):
            continue
        target = _classify_key(k)
        if target:
            name, idx = target
            vec = acc.setdefault(name, [None, None, None, None])
            vec[idx] = fv
        elif t_src is None and _norm_key(k) in _TIME_HINTS:
            t_src = fv

    rec: dict[str, list] = {}
    if acc.get("quat") and all(v is not None for v in acc["quat"]):
        rec["quat"] = quat_normalise(acc["quat"])
    elif acc.get("euler") and all(v is not None for v in acc["euler"][:3]):
        rec["quat"] = euler_deg_to_quat(*acc["euler"][:3])
        rec["_quat_from_euler"] = [1.0]
    for name in ("gyro", "accel", "mag"):
        vec = acc.get(name)
        if vec and all(v is not None for v in vec[:3]):
            rec[name] = [float(v) for v in vec[:3]]
    if "gyro" in rec:
        rec["gyro"] = _to_rad_s(rec["gyro"], gyro_units)
    return t_src, rec


def _to_rad_s(gyro, units: str):
    if units == "deg":
        return [v * DEG for v in gyro]
    if units == "rad":
        return gyro
    # "auto" at the row level can only guess from magnitude, and it guesses
    # WRONG for slow motion: 6 deg/s and 6 rad/s are the same number. Rows
    # parsed through a link go through GyroUnits below, which decides once
    # from accumulated evidence instead. This path is the fallback for a
    # one-off parse with no link behind it, and it is deliberately timid.
    return gyro


class GyroUnits:
    """
    Decide ONCE, per link, whether the gyroscope is in degrees or radians.

    Guessing per sample from magnitude is wrong on exactly the data that
    matters: a tool moving slowly reads 5 deg/s, which is also a plausible
    5 rad/s, so a per-sample rule flips back and forth mid-run and the
    recorded rates are a mixture of two unit systems. That is not a small
    error — it is a factor of 57 on an unknown subset of the rows.

    Two sources of evidence, in order of strength:

      ORIENTATION. If the unit also streams a quaternion, the angle between
      consecutive quaternions over the elapsed time IS the angular speed, in
      rad/s, measured independently of the gyro's own scaling. The ratio to
      the reported gyro magnitude is then either about 1 (radians) or about
      57 (degrees), and nothing else. This is decisive, so it is used first.

      MAGNITUDE. With no orientation to compare against, the peak magnitude
      over the first few seconds is all there is: a unit on a moving arm that
      never exceeds 7 is reporting radians, because 7 rad/s is 400 deg/s.

    Until it has decided, the raw value is passed through unscaled and the
    link REPORTS that it is undecided, rather than silently applying a guess.
    """

    DECIDE_AFTER = 40          # samples of evidence before committing
    RATIO_DEG = 20.0           # anything above this is degrees, not radians

    def __init__(self, pinned: str = "auto"):
        self.pinned = pinned if pinned in ("deg", "rad") else ""
        self.decided = self.pinned or ""
        self.evidence = 0
        self.ratio_sum = 0.0
        self.ratio_n = 0
        self.peak = 0.0
        self._prev = None      # (t, quat)
        self.basis = "pinned by the operator" if self.pinned else ""

    def feed(self, rec: dict, t: float | None) -> dict:
        gyro = rec.get("gyro")
        if not gyro:
            return rec
        mag = math.sqrt(sum(float(v) * float(v) for v in gyro))
        self.peak = max(self.peak, mag)
        quat = rec.get("quat")

        if not self.decided:
            if quat and t is not None:
                if self._prev is not None:
                    pt, pq = self._prev
                    dt = t - pt
                    if 1e-4 < dt < 0.5 and mag > 1e-3:
                        d = abs(sum(a * b for a, b in zip(pq, quat)))
                        d = max(-1.0, min(1.0, d))
                        w = 2.0 * math.acos(d) / dt      # rad/s, from orientation
                        if w > 1e-3:
                            self.ratio_sum += mag / w
                            self.ratio_n += 1
                self._prev = (t, quat)
            self.evidence += 1
            if self.ratio_n >= 12:
                r = self.ratio_sum / self.ratio_n
                self.decided = "deg" if r > self.RATIO_DEG else "rad"
                self.basis = (f"compared against the unit's own orientation "
                              f"(ratio {r:.1f})")
            elif self.evidence >= self.DECIDE_AFTER:
                self.decided = "deg" if self.peak > 7.0 else "rad"
                self.basis = (f"from the peak reading of {self.peak:.1f} over "
                              f"{self.evidence} samples")

        if self.decided == "deg":
            rec = dict(rec)
            rec["gyro"] = [float(v) * DEG for v in gyro]
        elif not self.decided:
            # Mark the rows taken before the verdict. Downstream orientation
            # estimators must not integrate a rate whose units are still
            # unknown: being wrong by 57x for the first second throws the
            # filter so far off that it takes tens of seconds to recover, and
            # the operator sees a wildly wrong attitude with no explanation.
            rec = dict(rec)
            rec["_units_pending"] = [1.0]
        return rec

    def status(self) -> dict:
        return {"gyro_units": self.decided or "deciding",
                "gyro_units_basis": self.basis,
                "gyro_peak_raw": round(self.peak, 3)}


def euler_deg_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return quat_normalise([cr * cp * cy + sr * sp * sy,
                           sr * cp * cy - cr * sp * sy,
                           cr * sp * cy + sr * cp * sy,
                           cr * cp * sy - sr * sp * cy])


class CsvParser:
    """
    Delimited text with a header line somewhere at the start.

    FusionHub's CSV export puts the header first; some UDP profiles send only
    rows. Both work: a row of pure numbers arriving before any header is
    buffered until a header shows up, rather than silently discarded, because
    a user who sees "0 parsed, 0 bad" cannot tell which of those happened.
    """

    def __init__(self, gyro_units: str = "auto"):
        self.header: list[str] | None = None
        self.delim = ","
        self.gyro_units = gyro_units
        self.pending = 0

    def feed(self, line: str):
        line = line.strip().lstrip("﻿")
        if not line:
            return None, {}
        if self.header is None:
            for d in (",", ";", "\t", " "):
                parts = [p for p in line.split(d) if p != ""]
                if len(parts) >= 4 and any(_classify_key(p) for p in parts):
                    self.header = [p.strip() for p in parts]
                    self.delim = d
                    return None, {}
            self.pending += 1
            return None, {}
        parts = [p for p in line.split(self.delim) if p != ""]
        if len(parts) < len(self.header):
            return None, {}
        row = dict(zip(self.header, parts))
        return _row_to_record(row, self.gyro_units)


def sniff(payload) -> dict:
    """
    What is this, and what did we get out of it? Shown verbatim in the UI so a
    misconfigured FusionHub output profile is visible rather than inferred.
    """
    raw = payload if isinstance(payload, (bytes, bytearray)) else str(payload).encode()
    preview = raw[:220]
    try:
        text = preview.decode("utf-8")
        printable = sum(1 for c in text if c.isprintable() or c in "\r\n\t")
        is_text = printable / max(1, len(text)) > 0.9
    except UnicodeDecodeError:
        text = repr(preview)
        is_text = False

    out = {"bytes": len(raw), "text": is_text,
           "preview": text if is_text else " ".join(f"{b:02x}" for b in raw[:160])}
    if not is_text:
        fmt = _name_binary(raw)
        # Protobuf carries no field names, so the only way to show what is in
        # it is to decode it. Leniently: a preview is routinely a truncated
        # message, and refusing to show the part that IS readable helps nobody.
        try:
            entries = protobuf_walk(raw, lenient=True)
        except Exception:
            entries = []
        vectors = _pb_vectors(entries)
        if len(entries) >= 3 and vectors:
            out["format"] = "protobuf"
            out["protobuf"] = [
                {"field": p, "type": k,
                 "value": (round(v, 6) if isinstance(v, float)
                           else v if not isinstance(v, bytes) else f"<{len(v)} bytes>")}
                for p, k, v in entries[:24]]
            out["vectors"] = [
                {"field": p, "n": len(v),
                 "values": [round(x, 5) for x in v],
                 "magnitude": round(math.sqrt(sum(x * x for x in v)), 4)}
                for p, v in vectors]
            out["advice"] = (
                "This is Protocol Buffers — FusionHub's External Output "
                "publishes binary, not text. It is decoded here without a "
                "schema: the magnitudes above identify the channels, since a "
                "unit-length 4-vector is an orientation and a 3-vector "
                "averaging 9.8 (or 1.0) is gravity. Connect and let it run "
                "for a second; the mapping it settles on is shown on the "
                "link.")
            return out
        out["format"] = fmt
        out["advice"] = _binary_advice(fmt)
        return out

    t, rec, fmt = parse_payload(raw)
    if not rec:
        csvp = CsvParser()
        for line in text.splitlines():
            t2, rec2 = csvp.feed(line)
            if rec2:
                t, rec, fmt = t2, rec2, "csv"
                break
        if not rec and csvp.header:
            fmt = "csv-header-only"
    out["format"] = fmt or "unrecognised"
    out["fields"] = sorted(k for k in rec if not k.startswith("_"))
    out["timestamp_found"] = t is not None
    out["record"] = {k: [round(float(x), 5) for x in v]
                     for k, v in rec.items() if not k.startswith("_")}
    if not rec:
        out["advice"] = ("Data is arriving and it is text, but no inertial "
                         "fields were recognised. The preview above shows "
                         "exactly what arrived — the field names need to "
                         "contain quaternion (qw/qx/qy/qz), gyroscope "
                         "(gx/gy/gz) or accelerometer (ax/ay/az) columns.")
    return out


# ---------------------------------------------------------------------------
# transports
# ---------------------------------------------------------------------------

class _Base:
    kind = "base"

    def __init__(self, unit: str, on_sample: Callable[[float, dict], None] | None = None,
                 gyro_units: str = "auto"):
        self.unit = unit
        self.on_sample = on_sample
        self.gyro_units = gyro_units
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.n = 0
        self.n_bad = 0
        self.error = ""
        self.last_raw = b""
        self.last_fmt = ""
        self.units = GyroUnits(gyro_units)
        self.pb = ProtobufImu()
        self.t_first = None
        self.t_last = None
        self._recent = deque(maxlen=64)

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> dict:
        if self._thread and self._thread.is_alive():
            return {"ok": True, "already": True}
        try:
            self._open()
        except Exception as e:                       # noqa: BLE001
            self.error = str(e)
            return {"ok": False, "error": self.error}
        self._stop.clear()
        self.error = ""
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"imulink-{self.unit}")
        self._thread.start()
        return {"ok": True}

    def stop(self) -> None:
        self._stop.set()
        try:
            self._close()
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.5)

    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    # -- subclass hooks ----------------------------------------------------
    def _open(self):
        raise NotImplementedError

    def _close(self):
        pass

    def _run(self):
        raise NotImplementedError

    # -- shared ------------------------------------------------------------
    def _parse(self, data, csvp):
        """
        One parse path for every transport: text, then protobuf, then CSV.

        Ordering is by certainty, not by convenience. The text parse is
        unambiguous when it works. Protobuf comes next because a successful
        schema-less walk is strong evidence — random bytes essentially never
        parse as valid wire format. A bare CSV row is last, because a row of
        numbers will happily "parse" as almost anything and must not get first
        refusal.
        """
        t, rec, fmt = parse_payload(data, "raw")
        if rec:
            return t, rec, fmt
        raw = data if isinstance(data, (bytes, bytearray)) else str(data).encode()
        t2, rec2 = self.pb.feed(raw)
        if rec2:
            return t2, rec2, "protobuf"
        if csvp is not None:
            for line in raw.decode("utf-8", "ignore").splitlines():
                t3, rec3 = csvp.feed(line)
                if rec3:
                    return t3, rec3, "csv"
        return t, rec, fmt

    def _emit(self, raw, t_src, rec, fmt):
        self.last_raw = raw if isinstance(raw, (bytes, bytearray)) else str(raw).encode()
        if not rec:
            # Packets consumed while the protobuf decoder is still working out
            # which field is which are not bad packets. Counting them as bad
            # makes a healthy binary link open with a burst of failures.
            if not (self.pb.n_decoded and self.pb.mapping is None):
                self.n_bad += 1
            return
        self.last_fmt = fmt
        if t_src is None:
            t_src = time.time()
            rec = dict(rec)
            rec["_arrival_time_used"] = [1.0]
        now = time.monotonic()
        # Units are decided per link, from accumulated evidence, not per row.
        rec = self.units.feed(rec, float(t_src))
        if self.t_first is None:
            self.t_first = now
        self.t_last = now
        self._recent.append(now)
        self.n += 1
        if self.on_sample:
            try:
                self.on_sample(float(t_src), rec)
            except Exception:
                pass

    def rate_hz(self) -> float:
        if len(self._recent) < 2:
            return 0.0
        span = self._recent[-1] - self._recent[0]
        return (len(self._recent) - 1) / span if span > 0 else 0.0

    def health(self) -> dict:
        age = (time.monotonic() - self.t_last) if self.t_last else None
        return {"unit": self.unit, "kind": self.kind, "running": self.running(),
                "samples": self.n, "bad": self.n_bad,
                "rate_hz": round(self.rate_hz(), 1),
                "age_s": round(age, 2) if age is not None else None,
                "format": self.last_fmt, "error": self.error,
                **self.units.status(), **self.pb.status(),
                "last_raw": self.last_raw[:200].decode("utf-8", "replace")
                if self.last_raw else ""}


class UdpListen(_Base):
    kind = "udp-listen"

    def __init__(self, port: int = 5005, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self.port, self.host = int(port), host
        self._sock = None

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.settimeout(0.4)
        self._sock = s

    def _close(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def _run(self):
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            t, rec, fmt = self._parse(data, csvp)
            self._emit(data, t, rec, fmt)


class TcpClient(_Base):
    """Connect out to a FusionHub TCP output and read newline-delimited rows."""
    kind = "tcp-client"

    def __init__(self, host: str = "127.0.0.1", port: int = 5005, **kw):
        super().__init__(**kw)
        self.host, self.port = host, int(port)
        self._sock = None

    def _open(self):
        self._sock = socket.create_connection((self.host, self.port), timeout=3.0)
        self._sock.settimeout(0.5)

    def _close(self):
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    def _run(self):
        buf = b""
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            try:
                chunk = self._sock.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                self.error = "stream closed by FusionHub"
                break
            buf += chunk
            # A stream with no newlines at all would grow without bound; cap it
            # rather than consume the host's memory over a long campaign.
            if len(buf) > 1 << 20:
                buf = buf[-4096:]
                self.n_bad += 1
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                t, rec, fmt = self._parse(line, csvp)
                self._emit(line, t, rec, fmt)


class TcpListen(_Base):
    """Accept a connection from FusionHub when it is configured to dial out."""
    kind = "tcp-listen"

    def __init__(self, port: int = 5005, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self.host, self.port = host, int(port)
        self._srv = None
        self._conn = None

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(1)
        s.settimeout(0.5)
        self._srv = s

    def _close(self):
        for s in (self._conn, self._srv):
            if s:
                try:
                    s.close()
                except OSError:
                    pass
        self._conn = self._srv = None

    def _run(self):
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            if self._conn is None:
                try:
                    self._conn, addr = self._srv.accept()
                    self._conn.settimeout(0.5)
                    log.info("FusionHub connected from %s", addr)
                except socket.timeout:
                    continue
                except OSError:
                    break
            buf = b""
            while not self._stop.is_set():
                try:
                    chunk = self._conn.recv(65535)
                except socket.timeout:
                    continue
                except OSError:
                    chunk = b""
                if not chunk:
                    try:
                        self._conn.close()
                    except OSError:
                        pass
                    self._conn = None
                    break
                buf += chunk
                if len(buf) > 1 << 20:
                    buf = buf[-4096:]
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    t, rec, fmt = self._parse(line, csvp)
                    self._emit(line, t, rec, fmt)


class HttpPoll(_Base):
    """
    Poll a JSON endpoint — FusionHub's REST interface, or any gateway that
    exposes the latest sample over HTTP.

    Polling cannot beat the endpoint's own update rate, and the achieved rate
    is REPORTED, so a 10 Hz poll is never mistaken for a 200 Hz log.
    """
    kind = "http-poll"

    def __init__(self, url: str = "http://127.0.0.1:8080/api/imu",
                 rate_hz: float = 50.0, **kw):
        super().__init__(**kw)
        self.url = url
        self.rate_hz_target = float(rate_hz)

    def _open(self):
        if not _HAS_HTTP:
            raise RuntimeError("urllib unavailable")
        with _urlreq.urlopen(self.url, timeout=3.0) as r:
            r.read(1)

    def _run(self):
        period = 1.0 / max(1.0, self.rate_hz_target)
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                with _urlreq.urlopen(self.url, timeout=2.0) as r:
                    data = r.read(65535)
                t, rec, fmt = self._parse(data, None)
                self._emit(data, t, rec, fmt)
            except Exception as e:                   # noqa: BLE001
                self.error = str(e)
                self.n_bad += 1
                time.sleep(0.5)
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                self._stop.wait(dt)


class SerialLink(_Base):
    """A unit on a COM port, or FusionHub's virtual serial output."""
    kind = "serial"

    def __init__(self, port: str = "COM3", baud: int = 115200, **kw):
        super().__init__(**kw)
        self.port, self.baud = port, int(baud)
        self._ser = None

    def _open(self):
        if not _HAS_SERIAL:
            raise RuntimeError(f"pyserial not installed ({_SERIAL_ERR}) — "
                               f"{pip_hint('pyserial')}")
        self._ser = pyserial.Serial(self.port, self.baud, timeout=0.4)

    def _close(self):
        if self._ser:
            self._ser.close()
            self._ser = None

    def _run(self):
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            try:
                line = self._ser.readline()
            except Exception as e:                   # noqa: BLE001
                self.error = str(e)
                break
            if not line.strip():
                continue
            t, rec, fmt = self._parse(line, csvp)
            self._emit(line, t, rec, fmt)


class FileTail(_Base):
    """
    Follow a file FusionHub is still writing.

    This is the escape hatch that always works: if no live transport can be
    configured, point FusionHub at a recording and point this at the file. The
    record shape is identical, so nothing downstream changes — which is the
    whole reason the live integration is not on the critical path.
    """
    kind = "file-tail"

    def __init__(self, path: str = "", from_start: bool = False, **kw):
        super().__init__(**kw)
        self.path = Path(path)
        self.from_start = bool(from_start)
        self._fh = None

    def _open(self):
        if not self.path.exists():
            raise FileNotFoundError(f"{self.path} does not exist")
        self._fh = self.path.open("r", encoding="utf-8", errors="ignore")
        if not self.from_start:
            self._fh.seek(0, 2)

    def _close(self):
        if self._fh:
            self._fh.close()
            self._fh = None

    def _run(self):
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            line = self._fh.readline()
            if not line:
                self._stop.wait(0.05)
                continue
            t, rec, fmt = self._parse(line.encode(), csvp)
            self._emit(line.encode(), t, rec, fmt)


class WebSocketClient(_Base):
    """
    Connect to a WebSocket that FusionHub is serving.

    FusionHub's WebSocket Sink accepts any data type, which makes it the one
    output that needs no thought about what the graph is carrying — so it is
    often the easiest to get working, and worth supporting even though TCP
    would do the same job.

    Runs its own asyncio loop on its own thread. The bridge's loop is never
    touched: a blocking read here would stall the jog cadence, and the whole
    point of moving that cadence to the host was to stop robot motion waiting
    on anything that can stall.
    """
    kind = "websocket-client"

    def __init__(self, url: str = "ws://127.0.0.1:8080", **kw):
        super().__init__(**kw)
        self.url = url
        self._loop = None
        self._ws = None

    def _open(self):
        if not _HAS_WS:
            raise RuntimeError(f"the websockets package is not installed "
                               f"({_WS_ERR}) — {pip_hint('websockets')}")

    def _close(self):
        """
        Ask the reader to finish, from the caller's thread.

        Deliberately NOT loop.stop(): stopping a loop out from under a running
        coroutine leaves its tasks pending, and closing the loop then raises
        out of them at interpreter level — a pile of tracebacks on a clean
        disconnect, which trains the operator to ignore tracebacks. Closing the
        socket instead lets the reader unwind normally.
        """
        loop, ws = self._loop, self._ws
        if loop is None or loop.is_closed():
            return
        if ws is not None:
            try:
                import asyncio
                asyncio.run_coroutine_threadsafe(ws.close(), loop)
            except Exception:
                pass

    def _run(self):
        import asyncio
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._reader())
        except Exception as e:                      # noqa: BLE001
            self.error = str(e)
        finally:
            # Let everything still in flight cancel and unwind before the loop
            # is closed, rather than closing under it.
            try:
                pending = [t for t in asyncio.all_tasks(loop) if not t.done()]
                for t in pending:
                    t.cancel()
                if pending:
                    loop.run_until_complete(
                        asyncio.gather(*pending, return_exceptions=True))
                loop.run_until_complete(loop.shutdown_asyncgens())
            except Exception:
                pass
            try:
                loop.close()
            except Exception:
                pass
            self._loop = None
            self._ws = None

    async def _reader(self):
        import asyncio
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            try:
                async with websockets.connect(self.url, open_timeout=4.0,
                                              max_size=8 * 1024 * 1024) as ws:
                    self._ws = ws
                    self.error = ""
                    log.info("websocket connected: %s", self.url)
                    while not self._stop.is_set():
                        try:
                            msg = await asyncio.wait_for(ws.recv(), timeout=0.25)
                        except asyncio.TimeoutError:
                            continue
                        t, rec, fmt = self._parse(msg, csvp)
                        self._emit(msg, t, rec, fmt)
            except asyncio.CancelledError:
                break
            except Exception as e:                  # noqa: BLE001
                # A dropped socket is reconnected, not reported as fatal: over
                # a four-week campaign the link will drop, and the run in
                # progress is still worth having.
                self._ws = None
                if self._stop.is_set():
                    break
                self.error = str(e)
                try:
                    await asyncio.sleep(2.0)
                except asyncio.CancelledError:
                    break
            finally:
                self._ws = None


class ZmqSub(_Base):
    """
    Subscribe to a ZeroMQ publisher — FusionHub's "External Output" node.

    That node's endpoint reads `tcp://*:8901`, which is ZeroMQ's address
    syntax, not a raw socket: `*` means bind every interface, and the node is
    therefore the SERVER. A plain TCP client connecting to it completes the
    TCP handshake and then receives ZMTP protocol frames — a greeting, a
    handshake, then length-prefixed frames — so the connection LOOKS fine and
    delivers nothing parseable. That failure mode is why this transport exists
    rather than reusing tcp-client: the two are indistinguishable at the
    socket layer and completely different on the wire.

    `topic` is the subscription filter. ZeroMQ PUB sockets deliver nothing at
    all until a SUB socket subscribes, so the empty-string default — subscribe
    to everything — is the one that cannot silently deliver zero messages.
    Multipart messages are handled: publishers commonly send [topic, payload],
    and taking the first frame would give you the topic name forever.
    """
    kind = "zmq-sub"

    def __init__(self, endpoint: str = "tcp://127.0.0.1:8901",
                 topic: str = "", **kw):
        super().__init__(**kw)
        self.endpoint = _zmq_connect_endpoint(endpoint)
        self.topic = topic or ""
        self._ctx = None
        self._sock = None

    def _open(self):
        """
        Validate only. The socket itself is created on the reader thread.

        ZeroMQ sockets are NOT thread-safe, and closing one from a second
        thread while the first is blocked in recv aborts the process outright
        — a C-level assertion, not a Python exception, so nothing upstream can
        catch it and the whole agent dies on a Disconnect button. So the
        socket is created, used and destroyed on one thread and never touched
        from anywhere else.

        Nothing is lost by validating here: a ZeroMQ tcp:// connect is
        asynchronous and succeeds against a dead peer anyway, so an early
        connect could not have reported a wrong port either.
        """
        if not _HAS_ZMQ:
            raise RuntimeError(f"the pyzmq package is not installed "
                               f"({_ZMQ_ERR}) — {pip_hint('pyzmq')}")
        if not self.endpoint.startswith(("tcp://", "ipc://", "inproc://")):
            raise ValueError(f"{self.endpoint!r} is not a ZeroMQ endpoint — "
                             "it should look like tcp://127.0.0.1:8901")
        if self.endpoint.startswith("tcp://"):
            host_port = self.endpoint[len("tcp://"):]
            port = host_port.rpartition(":")[2]
            if not port.isdigit():
                # ZeroMQ accepts the connect and then fails asynchronously, so
                # without this the link reports "connected" and delivers
                # nothing — the one outcome that gives the operator no clue.
                raise ValueError(
                    f"{self.endpoint!r} has no port. Copy the whole endpoint "
                    "from FusionHub's External Output node, including the "
                    "number after the colon — for example tcp://*:8901.")

    def _close(self):
        # Deliberately empty: the reader thread owns the socket and shuts it
        # down itself once _stop is set, within one receive timeout.
        return

    def _run(self):
        ctx = sock = None
        try:
            # A private context, not Context.instance(): a shared context is
            # terminated by whichever link tears down first, which would stop
            # every other link with it.
            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            # Never queue an unbounded backlog. A late inertial sample is
            # worthless, and a backlog turns a momentary stall into minutes of
            # stale data arriving as though it were live.
            sock.setsockopt(zmq.RCVHWM, 1000)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.RCVTIMEO, 250)
            sock.setsockopt_string(zmq.SUBSCRIBE, self.topic)
            sock.connect(self.endpoint)
            log.info("zmq SUB connected: %s (topic %r)", self.endpoint, self.topic)
            self._loop_recv(sock)
        except Exception as e:                      # noqa: BLE001
            self.error = str(e)
            log.warning("zmq subscriber failed: %s", e)
        finally:
            try:
                if sock is not None:
                    sock.close(linger=0)
            except Exception:
                pass
            try:
                if ctx is not None:
                    ctx.term()
            except Exception:
                pass

    def _loop_recv(self, sock):
        csvp = CsvParser("raw")
        while not self._stop.is_set():
            try:
                parts = sock.recv_multipart()
            except zmq.Again:
                continue                            # receive timeout, normal
            except Exception as e:                  # noqa: BLE001
                if self._stop.is_set():
                    break
                self.error = str(e)
                time.sleep(0.2)
                continue
            if not parts:
                continue
            # [topic, payload] is the common shape; [payload] alone is also
            # common. Try the LAST frame first — a topic frame never parses,
            # and reporting "nothing recognised" because we read the topic
            # would send the operator hunting in FusionHub for a fault in here.
            data = parts[-1]
            t, rec, fmt = self._parse(data, csvp)
            if not rec and len(parts) > 1:
                data = parts[0]
                t, rec, fmt = self._parse(data, csvp)
            self._emit(data, t, rec, fmt)

    def health(self) -> dict:
        h = super().health()
        h["endpoint"] = self.endpoint
        h["topic"] = self.topic or "(everything)"
        return h


def _zmq_connect_endpoint(endpoint: str) -> str:
    """
    Turn a BIND address into a CONNECT address.

    FusionHub shows `tcp://*:8901` because that is what it binds. A subscriber
    cannot connect to `*` — it has to name a host — so the wildcard is
    rewritten to localhost. Copying the endpoint out of FusionHub verbatim is
    the obvious thing to do, and without this it fails with an error about the
    address rather than about the wildcard.
    """
    ep = str(endpoint or "").strip()
    if not ep:
        return "tcp://127.0.0.1:8901"
    if "://" not in ep:
        ep = "tcp://" + ep
    scheme, _, rest = ep.partition("://")
    if rest.startswith("*:"):
        rest = "127.0.0.1:" + rest[2:]
    elif rest.startswith("0.0.0.0:"):
        rest = "127.0.0.1:" + rest[len("0.0.0.0:"):]
    return f"{scheme}://{rest}"


TRANSPORTS = {
    "udp-listen": UdpListen,
    "zmq-sub": ZmqSub,
    "websocket-client": WebSocketClient,
    "tcp-client": TcpClient,
    "tcp-listen": TcpListen,
    "http-poll": HttpPoll,
    "serial": SerialLink,
    "file-tail": FileTail,
}


def make_link(kind: str, unit: str, on_sample=None, gyro_units: str = "auto",
              **kw) -> _Base:
    cls = TRANSPORTS.get(kind)
    if cls is None:
        raise ValueError(f"unknown transport {kind!r}; "
                         f"choose one of {sorted(TRANSPORTS)}")
    return cls(unit=unit, on_sample=on_sample, gyro_units=gyro_units, **kw)


# ---------------------------------------------------------------------------
# discovery
# ---------------------------------------------------------------------------

def discover_udp(ports=None, seconds: float = 6.0) -> dict:
    """
    Bind every candidate port and report what arrived.

    This is the first thing to run when FusionHub "is streaming" and the
    console shows nothing. It answers, in one step, the three questions that
    otherwise take an afternoon: is anything arriving, on which port, and in
    what format.
    """
    ports = list(ports or CANDIDATE_UDP_PORTS)
    socks = {}
    blocked = {}
    for p in ports:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", p))
            s.setblocking(False)
            socks[p] = s
        except OSError as e:
            blocked[p] = str(e)

    found: dict[int, dict] = {}
    deadline = time.monotonic() + max(0.5, seconds)
    import select
    while time.monotonic() < deadline:
        if not socks:
            break
        r, _, _ = select.select(list(socks.values()), [], [], 0.25)
        for s in r:
            port = next(p for p, sk in socks.items() if sk is s)
            try:
                data, addr = s.recvfrom(65535)
            except OSError:
                continue
            e = found.setdefault(port, {"packets": 0, "from": addr[0], "sniff": None})
            e["packets"] += 1
            if e["sniff"] is None:
                e["sniff"] = sniff(data)
    for s in socks.values():
        s.close()

    usable = [p for p, e in found.items()
              if (e["sniff"] or {}).get("fields")]
    out = {"listened": ports, "seconds": seconds, "found": found,
           "blocked": blocked, "usable_ports": usable}
    if usable:
        out["advice"] = (f"Inertial data is arriving on UDP {usable[0]}. "
                         f"Use transport 'udp-listen' on that port.")
    elif found:
        out["advice"] = ("Packets are arriving but no inertial fields were "
                         "recognised — see the preview for what FusionHub is "
                         "actually sending, then change its output profile to "
                         "JSON or CSV with quaternion/gyro/accel columns.")
    elif blocked:
        out["advice"] = ("Nothing arrived, and some ports could not even be "
                         "opened — another program already holds them "
                         "(FusionHub itself, or an earlier agent still running).")
    else:
        out["advice"] = ("Nothing arrived on any candidate port. In FusionHub, "
                         "enable a UDP/network output, set the destination to "
                         "this PC (127.0.0.1 if FusionHub runs here), and allow "
                         "Python through the Windows firewall on private networks.")
    return out


def probe_tcp(host: str = "127.0.0.1", ports=None, timeout: float = 0.4) -> dict:
    """Which TCP ports on the FusionHub host accept a connection right now."""
    ports = list(ports or CANDIDATE_TCP_PORTS)
    open_ports = []
    for p in ports:
        try:
            with socket.create_connection((host, p), timeout=timeout):
                open_ports.append(p)
        except OSError:
            pass
    return {"host": host, "probed": ports, "open": open_ports}


def list_serial_ports() -> dict:
    if not _HAS_SERIAL:
        return {"available": False,
                "error": f"pyserial not installed ({_SERIAL_ERR}) — "
                         f"{pip_hint('pyserial')}", "ports": []}
    try:
        from serial.tools import list_ports          # type: ignore
        return {"available": True, "ports": [
            {"device": p.device, "description": p.description,
             "hwid": p.hwid} for p in list_ports.comports()]}
    except Exception as e:                           # noqa: BLE001
        return {"available": False, "error": str(e), "ports": []}
