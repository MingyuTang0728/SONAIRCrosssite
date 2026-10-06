"""
A stand-in for LP-Research's `openzen` Python module -- the subset
openzen_bridge.py uses, with the same names and shapes as the real binding
(src/bindings/OpenZenPython.cpp): make_client, list_sensors_async,
poll_next_event, obtain_sensor(_by_name), get_any_component_of_type, the
event types, and ZenImuData's a / g1 / g2 / w / q / timestamp / frame_count
in g and degrees per second.

It behaves like an LPMS-B2 over Bluetooth in the ways that matter here,
each switchable from the environment:

  FAKE_OZ_RATE        output rate, Hz (100)
  FAKE_OZ_BURST_S     the radio delivers in bursts this far apart (0.04)
  FAKE_OZ_FC_STEP     frame counter step per sample sent (4: 400 Hz inside)
  FAKE_OZ_LOSE_EVERY  lose every Nth frame on the radio (0 = none)
  FAKE_OZ_DROP_AT     disconnect this long after connecting (0 = never)
  FAKE_OZ_STALL_AT    stop sending, silently, this long after connecting
  FAKE_OZ_DOWN_S      after a drop/stall, refuse to connect for this long
  FAKE_OZ_STATE       file recording that the one-off fault has happened
  FAKE_OZ_NONE        "1": no sensor can be found
"""
import os
import time
from collections import deque


def _f(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


class _Enum:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name

    __str__ = __repr__


class ZenError:
    NoError = _Enum("NoError")
    Unknown = _Enum("Unknown")


class ZenSensorInitError:
    NoError = _Enum("NoError")
    ConnectFailed = _Enum("ConnectFailed")


class ZenEventType:
    NoType = _Enum("NoType")
    SensorFound = _Enum("SensorFound")
    SensorListingProgress = _Enum("SensorListingProgress")
    SensorDisconnected = _Enum("SensorDisconnected")
    ImuData = _Enum("ImuData")


class ZenImuProperty:
    SamplingRate = _Enum("SamplingRate")
    StreamData = _Enum("StreamData")


class ZenSensorProperty:
    BatteryLevel = _Enum("BatteryLevel")


class ZenLogLevel:
    Warning = _Enum("Warning")


component_type_imu = "imu"


def set_log_level(level):
    pass


class _Handle:
    def __init__(self, h):
        self.handle = h

    def __eq__(self, other):
        return isinstance(other, _Handle) and other.handle == self.handle

    def __ne__(self, other):
        return not self.__eq__(other)


class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Desc:
    name = "LPMSB2-4B3141"
    io_type = "Bluetooth"
    identifier = "00:04:3E:4B:31:41"
    serial_number = "LPMSB2-4B3141"
    baud_rate = 115200


def _fault_done():
    p = os.environ.get("FAKE_OZ_STATE", "")
    return bool(p) and os.path.exists(p)


def _mark_fault():
    p = os.environ.get("FAKE_OZ_STATE", "")
    if p:
        with open(p, "w") as fh:
            fh.write(str(time.perf_counter()))


def _down_until():
    p = os.environ.get("FAKE_OZ_STATE", "")
    if not p or not os.path.exists(p):
        return 0.0
    with open(p) as fh:
        return float(fh.read() or 0) + _f("FAKE_OZ_DOWN_S", 0)


class _Component:
    def __init__(self, sensor):
        self.sensor = sensor.sensor
        self.component = _Handle(7)

    def set_int32_property(self, prop, value):
        return ZenError.NoError

    def set_bool_property(self, prop, value):
        return ZenError.NoError


class _Sensor:
    def __init__(self, client, h):
        self.client = client
        self.sensor = _Handle(h)
        self.imu = _Component(self)

    def get_any_component_of_type(self, t):
        return self.imu if t == component_type_imu else None

    def get_float_property(self, prop):
        return ZenError.NoError, 87.0

    def release(self):
        self.client._sensor = None


class _Client:
    def __init__(self):
        self._q = deque()
        self._sensor = None
        self._n = 0

    # -- listing ---------------------------------------------------------
    def list_sensors_async(self):
        if os.environ.get("FAKE_OZ_NONE") != "1":
            self._q.append(_Obj(event_type=ZenEventType.SensorFound,
                                sensor=None, component=None,
                                data=_Obj(sensor_found=_Desc())))
        self._q.append(_Obj(event_type=ZenEventType.SensorListingProgress,
                            sensor=None, component=None,
                            data=_Obj(sensor_listing_progress=_Obj(
                                progress=1.0, complete=1))))
        return ZenError.NoError

    # -- connecting ------------------------------------------------------
    def _connect(self):
        if os.environ.get("FAKE_OZ_NONE") == "1" or \
                time.perf_counter() < _down_until():
            return ZenSensorInitError.ConnectFailed, None
        self._n += 1
        s = _Sensor(self, self._n)
        self._sensor = s
        now = time.perf_counter()
        self._t0 = now                  # first sample due now
        self._k = 0                     # samples produced
        self._released = now
        self._pending = deque()
        self._fc0 = 1000 * self._n
        self._ts0 = 50.0 * self._n      # the sensor clock restarts per connect
        return ZenSensorInitError.NoError, s

    def obtain_sensor(self, desc):
        return self._connect()

    def obtain_sensor_by_name(self, io_type, name, baud=0):
        return self._connect()

    # -- events ----------------------------------------------------------
    def _produce(self):
        s = self._sensor
        if s is None:
            return
        now = time.perf_counter()
        up = now - self._t0
        drop_at, stall_at = _f("FAKE_OZ_DROP_AT", 0), _f("FAKE_OZ_STALL_AT", 0)
        if drop_at and not _fault_done() and up > drop_at:
            _mark_fault()
            self._sensor = None
            self._q.append(_Obj(event_type=ZenEventType.SensorDisconnected,
                                sensor=s.sensor, component=s.imu.component,
                                data=_Obj(sensor_disconnected=_Obj(error="Io_ReadFailed"))))
            return
        if stall_at and not _fault_done() and up > stall_at:
            _mark_fault()
            self._stalled = s
        if getattr(self, "_stalled", None) is s:
            return
        rate = _f("FAKE_OZ_RATE", 100)
        step = int(_f("FAKE_OZ_FC_STEP", 4))
        lose = int(_f("FAKE_OZ_LOSE_EVERY", 0))
        while self._t0 + self._k / rate <= now:
            k = self._k
            self._k += 1
            if lose and k and k % lose == 0:
                continue
            t = k / rate
            ang = 0.5235987755982988 * t          # 30 deg/s about z
            import math
            fc = self._fc0 + step * k
            if os.environ.get("FAKE_OZ_FC_JITTER") == "1" and k % 2:
                fc += 1                 # steps of 5, 3, 5, 3 ... nothing lost
            d = _Obj(timestamp=self._ts0 + t, frame_count=fc,
                     a=[0.0, 0.0, 1.0], g1=[0.0, 0.0, 30.0], g2=[0.0, 0.0, 0.0],
                     w=[0.0, 0.0, 30.0],
                     q=[math.cos(ang / 2), 0.0, 0.0, math.sin(ang / 2)])
            self._pending.append(_Obj(event_type=ZenEventType.ImuData,
                                      sensor=s.sensor, component=s.imu.component,
                                      data=_Obj(imu_data=d)))
        if now - self._released >= _f("FAKE_OZ_BURST_S", 0.04):
            self._released = now
            self._q.extend(self._pending)
            self._pending.clear()

    def poll_next_event(self):
        if not self._q:
            self._produce()
        return self._q.popleft() if self._q else None

    def wait_for_next_event(self):
        while True:
            ev = self.poll_next_event()
            if ev is not None:
                return ev
            time.sleep(0.001)

    def close(self):
        self._sensor = None


def make_client():
    return ZenError.NoError, _Client()
