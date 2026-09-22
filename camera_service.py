"""
camera_service.py — everything the console needs to know about the camera.

The old panel offered hardcoded resolution and frame-rate lists. A D435i does
not support every combination on every stream, so "Apply Config" could ask for
a mode the device silently refused, and nothing said so. This module asks the
DEVICE what it supports and hands that to the browser, which is the difference
between a panel that configures a camera and a panel that looks like it does.

What it provides:

    probe()          device identity, USB generation, every supported stream
                     profile, the live intrinsics, and the controllable options
                     with their real ranges
    stats()          depth quality on the current frame: fill rate, and the
                     depth distribution in a centre ROI
    point(x, y)      the metric depth at one pixel and its 3D position in the
                     camera frame — the only way to confirm from the browser
                     that depth is real metres and not a pretty colourmap
    set_option()     laser power, visual preset, depth units, exposure

Why fill rate matters enough to compute every time it is asked for: a stereo
camera returns nothing where it cannot match, and a machined metal face is
exactly the surface it fails on. A reconstruction built from 30%-filled depth
frames looks like a reconstruction right up until you measure it.

Import-tolerant: with no pyrealsense2 present every entry point returns a
structured "unavailable" answer rather than raising, because the bridge must
boot on a machine that has never seen a camera.
"""
from __future__ import annotations

import logging
import math
import time

log = logging.getLogger("camera")

try:
    import pyrealsense2 as rs
    _HAS_RS = True
    _RS_ERR = ""
except Exception as e:      # noqa: BLE001
    rs = None
    _HAS_RS = False
    _RS_ERR = str(e)

try:
    import numpy as np
    _HAS_NP = True
except Exception as e:      # noqa: BLE001
    np = None
    _HAS_NP = False


# Options worth exposing. Anything else on a D435i is either read-only or a
# footgun, and a panel with forty sliders is a panel nobody reads.
OPTION_SPEC = [
    ("laser_power",          "Laser power",        "depth"),
    ("emitter_enabled",      "Emitter",            "depth"),
    ("depth_units",          "Depth units",        "depth"),
    ("enable_auto_exposure", "Depth auto-exposure", "depth"),
    ("exposure",             "Depth exposure",     "depth"),
    ("gain",                 "Depth gain",         "depth"),
    ("visual_preset",        "Visual preset",      "depth"),
]

COLOR_OPTION_SPEC = [
    ("enable_auto_exposure",      "Colour auto-exposure", "color"),
    ("exposure",                  "Colour exposure",      "color"),
    ("gain",                      "Colour gain",          "color"),
    ("enable_auto_white_balance", "Auto white balance",   "color"),
    ("white_balance",             "White balance",        "color"),
]

# The D400 visual presets, in the order the SDK enumerates them.
PRESET_NAMES = ["Custom", "Default", "Hand", "High Accuracy",
                "High Density", "Medium Density"]


def unavailable(reason: str = "") -> dict:
    return {"available": False,
            "error": reason or _RS_ERR or "pyrealsense2 not installed"}


# ---------------------------------------------------------------------------
# probe
# ---------------------------------------------------------------------------

def _stream_key(vsp) -> str:
    st = vsp.stream_type()
    if st == rs.stream.depth:
        return "depth"
    if st == rs.stream.color:
        return "color"
    if st == rs.stream.infrared:
        return f"infrared{vsp.stream_index()}"
    return str(st).split(".")[-1]


def probe() -> dict:
    """
    Everything static about the attached camera.

    Enumerating profiles needs the device but NOT a running pipeline, so this
    is safe to call while streaming — which matters, because the panel needs
    the supported-mode list before it can offer a sensible choice, and asking
    the user to stop the stream first to find out what the stream can do is
    the wrong way round.
    """
    if not _HAS_RS:
        return unavailable()
    try:
        ctx = rs.context()
        devices = list(ctx.query_devices())
    except Exception as e:
        return unavailable(f"could not query devices: {e}")

    if not devices:
        return {"available": False, "error":
                "no RealSense device found. Use a USB 3 port (blue) and the "
                "cable that came with the camera — on USB 2 the device "
                "enumerates but drops to a reduced stream set."}

    d = devices[0]

    def info(key, dflt=""):
        try:
            return d.get_info(getattr(rs.camera_info, key))
        except Exception:
            return dflt

    sensors = []
    profiles: dict[str, dict] = {}
    options: dict[str, dict] = {}

    for s in d.query_sensors():
        sname = ""
        try:
            sname = s.get_info(rs.camera_info.name)
        except Exception:
            pass
        sensors.append(sname)

        # --- supported stream profiles, straight from the sensor ------------
        try:
            for p in s.get_stream_profiles():
                try:
                    vsp = p.as_video_stream_profile()
                    if not vsp:
                        continue
                    key = _stream_key(vsp)
                    fmt = str(vsp.format()).split(".")[-1]
                    # Only the formats the agent actually decodes.
                    if key == "depth" and fmt != "z16":
                        continue
                    if key == "color" and fmt not in ("bgr8", "rgb8"):
                        continue
                    if key.startswith("infrared") and fmt != "y8":
                        continue
                    res = f"{vsp.width()}x{vsp.height()}"
                    bucket = profiles.setdefault(key, {})
                    fps_list = bucket.setdefault(res, [])
                    if vsp.fps() not in fps_list:
                        fps_list.append(vsp.fps())
                except Exception:
                    continue
        except Exception as e:
            log.debug("profile enumeration failed on %s: %s", sname, e)

        # --- controllable options with their REAL ranges --------------------
        is_depth = "Stereo" in sname or "Depth" in sname
        spec = OPTION_SPEC if is_depth else COLOR_OPTION_SPEC
        target = options.setdefault("depth" if is_depth else "color", {})
        for opt_name, label, _scope in spec:
            try:
                opt = getattr(rs.option, opt_name)
                if not s.supports(opt):
                    continue
                r = s.get_option_range(opt)
                entry = {
                    "label": label,
                    "min": r.min, "max": r.max, "step": r.step,
                    "default": r.default, "value": s.get_option(opt),
                }
                if opt_name == "visual_preset":
                    entry["choices"] = PRESET_NAMES
                target[opt_name] = entry
            except Exception:
                continue

    for k in profiles:
        profiles[k] = {res: sorted(f, reverse=True)
                       for res, f in sorted(
                           profiles[k].items(),
                           key=lambda kv: -int(kv[0].split("x")[0]))}

    usb = info("usb_type_descriptor", "?")
    return {
        "available": True,
        "device": {
            "name": info("name", "RealSense"),
            "serial": info("serial_number"),
            "firmware": info("firmware_version"),
            "usb": usb,
            "usb_ok": not str(usb).startswith("2"),
            "has_imu": any("Motion" in s for s in sensors),
            "sensors": sensors,
        },
        "profiles": profiles,
        "options": options,
    }


def live_intrinsics(intr_obj, depth_scale: float | None = None) -> dict:
    """Serialise the intrinsics the agent read from the running pipeline."""
    if intr_obj is None:
        return {}
    out = {
        "fx": getattr(intr_obj, "fx", 0.0), "fy": getattr(intr_obj, "fy", 0.0),
        "cx": getattr(intr_obj, "cx", getattr(intr_obj, "ppx", 0.0)),
        "cy": getattr(intr_obj, "cy", getattr(intr_obj, "ppy", 0.0)),
        "width": getattr(intr_obj, "width", 0),
        "height": getattr(intr_obj, "height", 0),
    }
    if depth_scale is not None:
        out["depth_scale"] = depth_scale
    elif hasattr(intr_obj, "depth_scale"):
        out["depth_scale"] = intr_obj.depth_scale
    # Horizontal and vertical field of view, which is what tells you whether a
    # part fits in one view at a given standoff.
    if out["fx"] > 0 and out["width"]:
        out["hfov_deg"] = math.degrees(2 * math.atan(out["width"] / (2 * out["fx"])))
    if out["fy"] > 0 and out["height"]:
        out["vfov_deg"] = math.degrees(2 * math.atan(out["height"] / (2 * out["fy"])))
    return out


# ---------------------------------------------------------------------------
# depth quality
# ---------------------------------------------------------------------------

def stats(depth_raw, depth_scale: float = 0.001, roi_frac: float = 0.25) -> dict:
    """
    Depth quality on one raw frame.

    `fill` is the fraction of pixels that returned anything at all. On a
    machined metal face a stereo camera can fail on most of the frame, and a
    reconstruction built from a sparsely filled depth stream looks correct
    until you measure it. This number is the early warning.

    The ROI is a centred box, because that is where the part is during a
    survey sweep and whole-frame statistics are dominated by the background.
    """
    if not _HAS_NP:
        return {"available": False, "error": "numpy not installed"}
    if depth_raw is None:
        return {"available": False, "error": "no depth frame — is the depth stream on?"}

    d = np.asanyarray(depth_raw)
    if d.ndim != 2:
        return {"available": False, "error": "depth frame is not 2-D"}

    h, w = d.shape
    valid_all = int(np.count_nonzero(d))
    total = int(d.size)

    rh, rw = int(h * roi_frac), int(w * roi_frac)
    y0, x0 = (h - rh) // 2, (w - rw) // 2
    roi = d[y0:y0 + rh, x0:x0 + rw]
    rv = roi[roi > 0]

    out = {
        "available": True,
        "width": w, "height": h,
        "fill_all": valid_all / total if total else 0.0,
        "fill_roi": (rv.size / roi.size) if roi.size else 0.0,
        "roi_px": [int(x0), int(y0), int(rw), int(rh)],
        "depth_scale": depth_scale,
    }
    if rv.size:
        mm = rv.astype(np.float64) * depth_scale
        out.update({
            "roi_min_m": float(mm.min()),
            "roi_max_m": float(mm.max()),
            "roi_median_m": float(np.median(mm)),
            "roi_mean_m": float(mm.mean()),
            # Std over a flat surface is the depth noise you will actually get.
            # Quoting the datasheet figure instead is how people end up
            # surprised by their own error budget.
            "roi_std_mm": float(mm.std() * 1000.0),
        })
    return out


def point(depth_raw, x: int, y: int, intr, depth_scale: float = 0.001,
          window: int = 5) -> dict:
    """
    Metric depth at one pixel, plus its 3D position in the camera frame.

    Takes the MEDIAN of a small window rather than the single pixel: one pixel
    on a stereo camera is frequently a hole or an outlier, and a reading that
    flickers between 0 and a real value teaches the operator to distrust a
    panel that is working correctly.
    """
    if not _HAS_NP:
        return {"ok": False, "error": "numpy not installed"}
    if depth_raw is None:
        return {"ok": False, "error": "no depth frame"}
    d = np.asanyarray(depth_raw)
    h, w = d.shape[:2]
    x, y = int(x), int(y)
    if not (0 <= x < w and 0 <= y < h):
        return {"ok": False, "error": f"({x},{y}) outside {w}x{h}"}

    k = max(1, int(window)) // 2
    patch = d[max(0, y - k):y + k + 1, max(0, x - k):x + k + 1]
    vals = patch[patch > 0]
    if vals.size == 0:
        return {"ok": False, "x": x, "y": y,
                "error": "no depth return here — stereo could not match. "
                         "Common on specular metal, in shadow, or too close."}

    z = float(np.median(vals)) * depth_scale
    res = {"ok": True, "x": x, "y": y, "depth_m": z,
           "valid_in_window": int(vals.size), "window": int(window)}

    if intr is not None:
        fx = getattr(intr, "fx", 0.0)
        fy = getattr(intr, "fy", 0.0)
        cx = getattr(intr, "cx", getattr(intr, "ppx", 0.0))
        cy = getattr(intr, "cy", getattr(intr, "ppy", 0.0))
        if fx and fy:
            res["point_cam_m"] = [(x - cx) / fx * z, (y - cy) / fy * z, z]
    return res


# ---------------------------------------------------------------------------
# options
# ---------------------------------------------------------------------------

def set_option(sensor_kind: str, option_name: str, value) -> dict:
    """
    Set one option on the live device.

    Goes straight to the sensor rather than through a pipeline restart: laser
    power and exposure take effect immediately, and restarting the pipeline to
    change them would drop frames and reset the auto-exposure convergence you
    were waiting on.
    """
    if not _HAS_RS:
        return {"ok": False, "error": _RS_ERR or "pyrealsense2 not installed"}
    try:
        devices = list(rs.context().query_devices())
        if not devices:
            return {"ok": False, "error": "no device"}
        d = devices[0]
        want_depth = (sensor_kind == "depth")
        for s in d.query_sensors():
            name = ""
            try:
                name = s.get_info(rs.camera_info.name)
            except Exception:
                pass
            is_depth = "Stereo" in name or "Depth" in name
            if is_depth != want_depth:
                continue
            opt = getattr(rs.option, option_name, None)
            if opt is None or not s.supports(opt):
                return {"ok": False,
                        "error": f"{option_name} not supported on the {sensor_kind} sensor"}
            if option_name == "visual_preset" and isinstance(value, str):
                if value not in PRESET_NAMES:
                    return {"ok": False, "error": f"unknown preset {value!r}"}
                value = PRESET_NAMES.index(value)
            r = s.get_option_range(opt)
            v = max(r.min, min(r.max, float(value)))
            s.set_option(opt, v)
            return {"ok": True, "sensor": sensor_kind, "option": option_name,
                    "value": s.get_option(opt),
                    "clamped": abs(v - float(value)) > 1e-9}
        return {"ok": False, "error": f"no {sensor_kind} sensor on this device"}
    except Exception as e:
        return {"ok": False, "error": str(e)}
