"""
openzen_bridge.py -- an LPMS sensor read through LP-Research's own OpenZen
library, in a process of its own.

Why a separate process, run by a separate Python:

  * OpenZen ships prebuilt for ONE Python version at a time (openzen.pyd for
    Python 3.11 on Windows) and the console's agent runs on whatever Python the
    PC has. A small private Python 3.11 runs this file; the agent never imports
    OpenZen at all. `install_openzen.py` puts both in vendor/openzen/.
  * A Bluetooth driver that stalls holds up the process it runs in. Here that
    is this process, not the agent that is also reading the robot.
  * When the sensor drops off the air, this process notices (OpenZen reports
    the disconnect, or the samples simply stop), releases the sensor and
    connects again by itself, for as long as it is left running.

Output: one JSON object per line on stdout, flushed per line --
    {"ev": "sample", "ts": <sensor clock, s>, "fc": <frame>, "rx": <host
     perf_counter at receipt>, "a": [g], "g1": [deg/s], "g2": [deg/s],
     "w": [deg/s], "q": [w, x, y, z]}
    {"ev": "connected" | "connect_failed" | "disconnected" | "stalled" |
     "found" | "listed" | "fatal", ...}
Units are OpenZen's own (g and degrees per second); the agent converts.

The agent closes this process's stdin to stop it; it then releases the sensor
and exits, so a stopped agent never leaves the Bluetooth link held.

Runs on Python 3.8 or newer with only the standard library and OpenZen.
"""
import argparse
import json
import os
import sys
import threading
import time

_quit = threading.Event()
_out_lock = threading.Lock()


def emit(obj):
    line = json.dumps(obj, separators=(",", ":"))
    with _out_lock:
        try:
            sys.stdout.write(line + "\n")
            sys.stdout.flush()
        except (OSError, ValueError):
            _quit.set()


def _watch_stdin():
    """The agent holds our stdin open; its end means: stop."""
    try:
        while sys.stdin.read(1024):
            pass
    except Exception:       # noqa: BLE001
        pass
    _quit.set()


def _vec(v, n=3):
    try:
        return [float(x) for x in list(v)[:n]]
    except Exception:       # noqa: BLE001
        return None


def load_openzen(zen_dir):
    if zen_dir:
        zen_dir = os.path.abspath(zen_dir)
        sys.path.insert(0, zen_dir)
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(zen_dir)
            except OSError:
                pass
    import openzen          # noqa: E402  (path set just above)
    return openzen


def list_sensors(oz, client, seconds):
    """Every sensor OpenZen can see, as plain dicts."""
    found = []
    client.list_sensors_async()
    end = time.monotonic() + seconds
    while time.monotonic() < end and not _quit.is_set():
        ev = client.poll_next_event()
        if ev is None:
            time.sleep(0.01)
            continue
        if ev.event_type == oz.ZenEventType.SensorFound:
            d = ev.data.sensor_found
            found.append({"name": str(d.name), "io_type": str(d.io_type),
                          "identifier": str(d.identifier),
                          "serial": str(getattr(d, "serial_number", "")),
                          "desc": d})
            emit({"ev": "found", "name": str(d.name), "io_type": str(d.io_type),
                  "identifier": str(d.identifier)})
        elif ev.event_type == oz.ZenEventType.SensorListingProgress:
            if ev.data.sensor_listing_progress.complete > 0:
                break
    return found


def connect(oz, client, address, io_type, baud, list_s):
    """One attempt. Returns (sensor, imu, name) or raises RuntimeError."""
    if address:
        err, sensor = client.obtain_sensor_by_name(io_type, address, int(baud))
        name = address
    else:
        found = list_sensors(oz, client, list_s)
        pick = [f for f in found if "lpms" in f["name"].lower()] or found
        if not pick:
            raise RuntimeError("no LPMS sensor was found. Is it switched on, "
                               "charged, and paired with this PC in Windows "
                               "Bluetooth settings?")
        err, sensor = client.obtain_sensor(pick[0]["desc"])
        name = pick[0]["name"]
    if err != oz.ZenSensorInitError.NoError:
        raise RuntimeError("could not connect to %s (%s)" % (name, err))
    imu = sensor.get_any_component_of_type(oz.component_type_imu)
    if imu is None:
        try:
            sensor.release()
        except Exception:   # noqa: BLE001
            pass
        raise RuntimeError("%s has no IMU component" % name)
    return sensor, imu, name


def configure(oz, imu, rate):
    notes = []
    if rate:
        try:
            e = imu.set_int32_property(oz.ZenImuProperty.SamplingRate, int(rate))
            if e != oz.ZenError.NoError:
                notes.append("sampling rate not set (%s)" % e)
        except Exception as e:      # noqa: BLE001
            notes.append("sampling rate not set (%s)" % e)
    try:
        imu.set_bool_property(oz.ZenImuProperty.StreamData, True)
    except Exception:               # noqa: BLE001
        pass
    return notes


def battery(oz, sensor):
    try:
        err, lvl = sensor.get_float_property(oz.ZenSensorProperty.BatteryLevel)
        if err == oz.ZenError.NoError:
            return round(float(lvl), 1)
    except Exception:               # noqa: BLE001
        pass
    return None


def stream(oz, client, sensor, imu, stall_s):
    """Pass samples on until the sensor disconnects, stalls, or we are told
    to stop. Returns the reason it ended."""
    last = time.monotonic()
    last_batt = last
    while not _quit.is_set():
        ev = client.poll_next_event()
        now = time.monotonic()
        if ev is None:
            if now - last > stall_s:
                return "stalled", "no data for %.1f s" % (now - last)
            time.sleep(0.001)
            continue
        et = ev.event_type
        if et == oz.ZenEventType.ImuData:
            if ev.sensor != imu.sensor or \
                    ev.component.handle != imu.component.handle:
                continue
            d = ev.data.imu_data
            emit({"ev": "sample", "rx": time.perf_counter(),
                  "ts": float(d.timestamp), "fc": int(d.frame_count),
                  "a": _vec(d.a), "g1": _vec(d.g1), "g2": _vec(d.g2),
                  "w": _vec(d.w), "q": _vec(d.q, 4)})
            last = now
            if now - last_batt > 30.0:
                last_batt = now
                b = battery(oz, sensor)
                if b is not None:
                    emit({"ev": "battery", "level": b})
        elif et == oz.ZenEventType.SensorDisconnected:
            if ev.sensor == imu.sensor:
                return "disconnected", str(ev.data.sensor_disconnected.error)
    return "quit", ""


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--zen-dir", default="")
    ap.add_argument("--address", default="",
                    help="Bluetooth address or sensor name; blank finds one")
    ap.add_argument("--io", default="Bluetooth")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=int, default=100)
    ap.add_argument("--stall", type=float, default=2.0)
    ap.add_argument("--list", action="store_true",
                    help="report the sensors that can be seen, then exit")
    ap.add_argument("--list-seconds", type=float, default=12.0)
    args = ap.parse_args(argv)

    # Streaming runs until told to stop, so it watches stdin for the agent
    # going away. A search ends by itself, and is often started with no stdin
    # at all -- which would read as "stop" at once.
    if not args.list:
        threading.Thread(target=_watch_stdin, daemon=True).start()
    try:
        oz = load_openzen(args.zen_dir)
    except Exception as e:          # noqa: BLE001
        emit({"ev": "fatal", "error": "OpenZen could not be loaded: %s" % e,
              "python": sys.version.split()[0]})
        return 2
    try:
        oz.set_log_level(oz.ZenLogLevel.Warning)
    except Exception:               # noqa: BLE001
        pass
    err, client = oz.make_client()
    if err != oz.ZenError.NoError:
        emit({"ev": "fatal", "error": "OpenZen did not start (%s)" % err})
        return 2

    if args.list:
        found = list_sensors(oz, client, args.list_seconds)
        emit({"ev": "listed", "sensors": [
            {k: v for k, v in f.items() if k != "desc"} for f in found]})
        client.close()
        return 0

    backoff = 1.0
    failures = 0
    while not _quit.is_set():
        try:
            sensor, imu, name = connect(oz, client, args.address, args.io,
                                        args.baud, min(args.list_seconds, 10.0))
        except Exception as e:      # noqa: BLE001
            failures += 1
            emit({"ev": "connect_failed", "error": str(e), "attempt": failures,
                  "retry_s": backoff})
            # A client that has failed several times in a row is started
            # afresh: a Bluetooth stack that lost the device can leave the
            # old client unable to see it again.
            if failures % 3 == 0:
                try:
                    client.close()
                except Exception:   # noqa: BLE001
                    pass
                err, client = oz.make_client()
                if err != oz.ZenError.NoError:
                    emit({"ev": "fatal", "error": "OpenZen restart failed"})
                    return 2
            _quit.wait(backoff)
            backoff = min(backoff * 2.0, 15.0)
            continue
        notes = configure(oz, imu, args.rate)
        emit({"ev": "connected", "name": name, "battery": battery(oz, sensor),
              "notes": notes})
        backoff, failures = 1.0, 0
        why, detail = stream(oz, client, sensor, imu, args.stall)
        try:
            sensor.release()
        except Exception:           # noqa: BLE001
            pass
        if why == "quit":
            break
        emit({"ev": why, "error": detail})
        _quit.wait(0.5)
    try:
        client.close()
    except Exception:               # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
