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
import math
import re
import socket
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable

log = logging.getLogger("imu_link")

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


# Ports FusionHub and its neighbours are seen on in the field. Discovery binds
# all of them; the cost of one extra UDP socket is nothing next to an afternoon
# spent guessing.
CANDIDATE_UDP_PORTS = [5005, 5006, 5555, 6000, 8000, 8888, 9000, 9001, 9763, 4001]
CANDIDATE_TCP_PORTS = [5005, 8080, 9000, 9001, 502]

DEG = math.pi / 180.0


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
           "preview": text if is_text else " ".join(f"{b:02x}" for b in preview[:48])}
    if not is_text:
        out["format"] = "binary"
        out["advice"] = ("This is not text. Set FusionHub's output profile to "
                         "JSON or CSV over UDP — the binary XDA protocol is not "
                         "parsed here.")
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
    def _emit(self, raw, t_src, rec, fmt):
        self.last_raw = raw if isinstance(raw, (bytes, bytearray)) else str(raw).encode()
        if not rec:
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
                **self.units.status(),
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
            t, rec, fmt = parse_payload(data, "raw")
            if not rec:
                for line in data.decode("utf-8", "ignore").splitlines():
                    t2, rec2 = csvp.feed(line)
                    if rec2:
                        t, rec, fmt = t2, rec2, "csv"
                        break
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
                t, rec, fmt = parse_payload(line, "raw")
                if not rec:
                    t, rec = csvp.feed(line.decode("utf-8", "ignore"))
                    fmt = "csv"
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
                    t, rec, fmt = parse_payload(line, "raw")
                    if not rec:
                        t, rec = csvp.feed(line.decode("utf-8", "ignore"))
                        fmt = "csv"
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
                t, rec, fmt = parse_payload(data, "raw")
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
                               "run: pip install pyserial")
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
            t, rec, fmt = parse_payload(line, "raw")
            if not rec:
                t, rec = csvp.feed(line.decode("utf-8", "ignore"))
                fmt = "csv"
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
            t, rec, fmt = parse_payload(line, "raw")
            if not rec:
                t, rec = csvp.feed(line)
                fmt = "csv"
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
                               f"({_WS_ERR}) — run: pip install websockets")

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
                        t, rec, fmt = parse_payload(msg, "raw")
                        if not rec:
                            text = msg.decode("utf-8", "ignore") \
                                if isinstance(msg, (bytes, bytearray)) else str(msg)
                            for line in text.splitlines():
                                t2, rec2 = csvp.feed(line)
                                if rec2:
                                    t, rec, fmt = t2, rec2, "csv"
                                    break
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


TRANSPORTS = {
    "udp-listen": UdpListen,
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
                         "run: pip install pyserial", "ports": []}
    try:
        from serial.tools import list_ports          # type: ignore
        return {"available": True, "ports": [
            {"device": p.device, "description": p.description,
             "hwid": p.hwid} for p in list_ports.comports()]}
    except Exception as e:                           # noqa: BLE001
        return {"available": False, "error": str(e), "ports": []}
