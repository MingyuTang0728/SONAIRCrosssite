"""
rs_features.py — the rest of the D435i.

The console used two of this camera's capabilities: colourised depth and RGB.
A D435i is a stereo pair, an infrared projector, a colour camera and an IMU in
one housing, and the parts that were not being used are not garnish — they are
the parts that decide whether the depth is any good on a machined metal part:

  BOTH INFRARED CAMERAS, raw. Depth is computed FROM these. When depth has
  holes, the left/right images say why — no texture to match (turn the
  projector on), saturation from a specular highlight (drop the exposure), or
  one view occluded (move). Depth alone cannot distinguish these, and they
  have different fixes.

  THE PROJECTOR, as a control rather than a checkbox. Laser on for depth on
  featureless surfaces; laser off for a clean IR image; and ALTERNATING, so
  consecutive frames give you both. On a machined face this is the single
  biggest lever on fill rate.

  THE POST-PROCESSING CHAIN with its real parameters. Decimation, threshold,
  disparity transform, spatial, temporal and hole-filling, in the order Intel
  specifies. The order is not a preference: spatial and temporal filters are
  defined in DISPARITY space, and running them on depth instead quietly
  changes what they do.

  ON-CHIP SELF-CALIBRATION AND TARE. The D400 series can re-run its own
  extrinsic calibration in about ten seconds, and can be tared against a
  target at a known distance. A camera that has been bolted to a moving arm
  and knocked about deserves this before a campaign, and its health score is
  a number worth recording next to the data.

  FRAME METADATA. Per-frame hardware timestamps, the clock domain they are in,
  and the actual exposure used. The benchmark's whole claim rests on channels
  being on one clock; a frame whose timestamp is in the system domain rather
  than the hardware domain carries USB scheduling jitter, and you cannot tell
  by looking at the image.

  EXTRINSICS between every stream, read from the device, so a point found in
  the colour image can be placed in the depth camera's frame exactly rather
  than approximately.

Import-tolerant throughout: with no pyrealsense2, every entry point returns a
structured "unavailable" rather than raising.
"""
from __future__ import annotations

import json
import logging
import math
import time
from pathlib import Path

log = logging.getLogger("rs_features")

try:
    import pyrealsense2 as rs
    _HAS_RS = True
    _RS_ERR = ""
except Exception as e:                              # noqa: BLE001
    rs = None
    _HAS_RS = False
    _RS_ERR = str(e)

try:
    import numpy as np
    _HAS_NP = True
except Exception:                                   # pragma: no cover
    np = None
    _HAS_NP = False


def unavailable(reason: str = "") -> dict:
    return {"available": False,
            "error": reason or _RS_ERR or "pyrealsense2 not installed"}


# ---------------------------------------------------------------------------
# emitter modes
# ---------------------------------------------------------------------------

EMITTER_MODES = {
    "off":         {"emitter_enabled": 0.0,
                    "why": "clean infrared images; depth will be poor on "
                           "untextured surfaces"},
    "on":          {"emitter_enabled": 1.0,
                    "why": "best depth on machined and featureless surfaces"},
    "alternating": {"emitter_enabled": 2.0,
                    "why": "projector on and off on alternate frames — depth "
                           "and clean infrared from the same sequence"},
}


# ---------------------------------------------------------------------------
# post-processing
# ---------------------------------------------------------------------------

FILTER_SPEC = [
    # key,          label,                     option,                  lo,  hi, step
    ("decimation",  "Decimation",              "filter_magnitude",      1,   8,  1),
    ("threshold_min", "Nearest kept (m)",      "min_distance",          0.1, 4.0, 0.05),
    ("threshold_max", "Furthest kept (m)",     "max_distance",          0.1, 16.0, 0.1),
    ("spatial_magnitude", "Smoothing passes",  "filter_magnitude",      1,   5,  1),
    ("spatial_alpha", "Smoothing strength",    "filter_smooth_alpha",   0.25, 1.0, 0.05),
    ("spatial_delta", "Edge threshold",        "filter_smooth_delta",   1,   50, 1),
    ("spatial_holes", "Small hole fill",       "holes_fill",            0,   5,  1),
    ("temporal_alpha", "Time smoothing",       "filter_smooth_alpha",   0.0, 1.0, 0.05),
    ("temporal_delta", "Time threshold",       "filter_smooth_delta",   1,   100, 1),
    ("hole_filling", "Hole filling mode",      "holes_fill",            0,   2,  1),
]

DEFAULT_FILTERS = {
    "enabled": True,
    "decimation_on": False, "decimation": 2,
    "threshold_on": True, "threshold_min": 0.15, "threshold_max": 1.5,
    "disparity": True,
    "spatial_on": True, "spatial_magnitude": 2, "spatial_alpha": 0.5,
    "spatial_delta": 20, "spatial_holes": 0,
    "temporal_on": True, "temporal_alpha": 0.4, "temporal_delta": 20,
    "hole_filling_on": False, "hole_filling": 1,
}


class FilterChain:
    """
    Intel's recommended chain, in Intel's order, with the parameters exposed.

    Order: decimation -> threshold -> TO DISPARITY -> spatial -> temporal ->
    BACK TO DEPTH -> hole filling. The disparity round trip is not decoration:
    stereo error is uniform in disparity and very much not uniform in depth, so
    a spatial filter applied in depth space smooths the far field far too hard
    and the near field not at all.

    Every filter is also individually switchable, because the honest default
    for a measurement pipeline is fewer filters, not more: hole filling in
    particular INVENTS depth, which looks better and measures worse, so it
    ships off.
    """

    def __init__(self, config: dict | None = None):
        self.config = {**DEFAULT_FILTERS, **(config or {})}
        self._built = False
        self._f = {}
        self.error = ""

    def configure(self, config: dict) -> dict:
        self.config.update(config or {})
        self._built = False
        return dict(self.config)

    def _build(self):
        if not _HAS_RS:
            raise RuntimeError(_RS_ERR)
        c = self.config
        f = {}
        f["decimation"] = rs.decimation_filter()
        f["decimation"].set_option(rs.option.filter_magnitude,
                                   float(max(1, int(c["decimation"]))))
        f["threshold"] = rs.threshold_filter()
        f["threshold"].set_option(rs.option.min_distance, float(c["threshold_min"]))
        f["threshold"].set_option(rs.option.max_distance, float(c["threshold_max"]))
        f["to_disp"] = rs.disparity_transform(True)
        f["to_depth"] = rs.disparity_transform(False)
        sp = rs.spatial_filter()
        sp.set_option(rs.option.filter_magnitude, float(c["spatial_magnitude"]))
        sp.set_option(rs.option.filter_smooth_alpha, float(c["spatial_alpha"]))
        sp.set_option(rs.option.filter_smooth_delta, float(c["spatial_delta"]))
        sp.set_option(rs.option.holes_fill, float(c["spatial_holes"]))
        f["spatial"] = sp
        tp = rs.temporal_filter()
        tp.set_option(rs.option.filter_smooth_alpha, float(c["temporal_alpha"]))
        tp.set_option(rs.option.filter_smooth_delta, float(c["temporal_delta"]))
        f["temporal"] = tp
        hf = rs.hole_filling_filter()
        hf.set_option(rs.option.holes_fill, float(c["hole_filling"]))
        f["hole_filling"] = hf
        self._f = f
        self._built = True

    def process(self, depth_frame):
        """Run the chain. On any filter error the RAW frame is returned."""
        c = self.config
        if not c.get("enabled", True) or depth_frame is None:
            return depth_frame
        try:
            if not self._built:
                self._build()
            fr = depth_frame
            if c.get("decimation_on"):
                fr = self._f["decimation"].process(fr)
            if c.get("threshold_on"):
                fr = self._f["threshold"].process(fr)
            use_disp = c.get("disparity", True) and (c.get("spatial_on")
                                                     or c.get("temporal_on"))
            if use_disp:
                fr = self._f["to_disp"].process(fr)
            if c.get("spatial_on"):
                fr = self._f["spatial"].process(fr)
            if c.get("temporal_on"):
                fr = self._f["temporal"].process(fr)
            if use_disp:
                fr = self._f["to_depth"].process(fr)
            if c.get("hole_filling_on"):
                fr = self._f["hole_filling"].process(fr)
            return fr.as_depth_frame() if hasattr(fr, "as_depth_frame") else fr
        except Exception as e:                      # noqa: BLE001
            # A broken filter must never take the stream down with it. Report
            # once and pass the raw frame through — degraded depth beats none.
            if str(e) != self.error:
                self.error = str(e)
                log.warning("depth filter chain failed, passing raw depth: %s", e)
            return depth_frame

    def describe(self) -> dict:
        steps = []
        c = self.config
        if c.get("decimation_on"):
            steps.append(f"decimate x{int(c['decimation'])}")
        if c.get("threshold_on"):
            steps.append(f"keep {c['threshold_min']:.2f}-{c['threshold_max']:.2f} m")
        if c.get("spatial_on"):
            steps.append("smooth across the image")
        if c.get("temporal_on"):
            steps.append("smooth over time")
        if c.get("hole_filling_on"):
            steps.append("fill holes (INVENTS depth — not for measurement)")
        return {"config": dict(c), "steps": steps,
                "in_disparity_space": bool(c.get("disparity", True)),
                "error": self.error}


# ---------------------------------------------------------------------------
# device enumeration — every sensor, every option
# ---------------------------------------------------------------------------

def _device(index: int = 0):
    devs = list(rs.context().query_devices())
    if not devs:
        return None
    return devs[min(index, len(devs) - 1)]


def enumerate_device(index: int = 0) -> dict:
    """
    The whole camera: identity, firmware, every sensor with every writable
    option and its real range, and every supported stream profile.

    Ranges come from the DEVICE. A UI that offers a slider from a datasheet
    can ask for a value this unit refuses, and pyrealsense2 reports that by
    raising somewhere far from the slider.
    """
    if not _HAS_RS:
        return unavailable()
    try:
        dev = _device(index)
    except Exception as e:                          # noqa: BLE001
        return unavailable(f"could not query devices: {e}")
    if dev is None:
        return {"available": False, "error":
                "no RealSense camera found. Check the USB cable is in a USB 3 "
                "port (blue) and that no other program — RealSense Viewer "
                "included — is holding the camera."}

    def info(key, dflt=""):
        try:
            return dev.get_info(getattr(rs.camera_info, key))
        except Exception:
            return dflt

    out = {
        "available": True,
        "name": info("name"),
        "serial": info("serial_number"),
        "firmware": info("firmware_version"),
        "recommended_firmware": info("recommended_firmware_version"),
        "usb": info("usb_type_descriptor", "unknown"),
        "product_line": info("product_line"),
        "sensors": [],
    }
    out["usb3"] = str(out["usb"]).startswith("3")
    if not out["usb3"]:
        out["usb_warning"] = (
            f"Connected at USB {out['usb']}. On USB 2 the camera offers far "
            "fewer modes and cannot run colour and depth at full rate "
            "together. Use a USB 3 port and the cable that came with it.")

    for sensor in dev.query_sensors():
        try:
            sname = sensor.get_info(rs.camera_info.name)
        except Exception:
            sname = "sensor"
        entry = {"name": sname, "options": [], "profiles": {}}
        for opt in rs.option.__members__.values() if hasattr(rs.option, "__members__") \
                else []:
            try:
                if not sensor.supports(opt):
                    continue
                rng = sensor.get_option_range(opt)
                entry["options"].append({
                    "key": str(opt).split(".")[-1],
                    "value": float(sensor.get_option(opt)),
                    "min": float(rng.min), "max": float(rng.max),
                    "step": float(rng.step), "default": float(rng.default),
                    "readonly": bool(sensor.is_option_read_only(opt)),
                    "description": sensor.get_option_description(opt),
                })
            except Exception:
                continue
        for p in sensor.get_stream_profiles():
            try:
                vsp = p.as_video_stream_profile()
                if not vsp:
                    continue
                st = p.stream_type()
                key = ("infrared%d" % p.stream_index()) if st == rs.stream.infrared \
                    else str(st).split(".")[-1]
                res = f"{vsp.width()}x{vsp.height()}"
                entry["profiles"].setdefault(key, {}).setdefault(res, [])
                fps = p.fps()
                if fps not in entry["profiles"][key][res]:
                    entry["profiles"][key][res].append(fps)
            except Exception:
                continue
        for key in entry["profiles"]:
            for res in entry["profiles"][key]:
                entry["profiles"][key][res].sort(reverse=True)
        out["sensors"].append(entry)
    return out


def set_options(settings: list, index: int = 0) -> dict:
    """
    Apply a batch of options in one pass: [{sensor, option, value}, ...].

    Batched because the settings that matter come in pairs — auto-exposure off
    THEN exposure, white balance off THEN white balance — and a UI that sends
    them one round trip at a time races itself.
    """
    if not _HAS_RS:
        return unavailable()
    dev = _device(index)
    if dev is None:
        return {"ok": False, "error": "no camera"}
    sensors = {}
    for s in dev.query_sensors():
        try:
            sensors[s.get_info(rs.camera_info.name).lower()] = s
        except Exception:
            continue

    def pick(kind):
        kind = (kind or "depth").lower()
        for name, s in sensors.items():
            if kind in name:
                return s
        for name, s in sensors.items():
            if kind == "depth" and "stereo" in name:
                return s
            if kind == "color" and "rgb" in name:
                return s
        return None

    results = []
    # Auto-exposure and auto-white-balance first: setting a manual value while
    # the corresponding auto is still on is accepted and then overwritten.
    order = sorted(settings, key=lambda s: 0 if str(s.get("option", ""))
                   .startswith("enable_auto") else 1)
    for item in order:
        s = pick(item.get("sensor"))
        name = item.get("option", "")
        if s is None:
            results.append({**item, "ok": False, "error": "sensor not present"})
            continue
        try:
            opt = getattr(rs.option, name)
            if not s.supports(opt):
                results.append({**item, "ok": False,
                                "error": "this camera does not support it"})
                continue
            if s.is_option_read_only(opt):
                results.append({**item, "ok": False, "error": "read-only"})
                continue
            rng = s.get_option_range(opt)
            v = float(item.get("value", 0))
            clamped = min(max(v, float(rng.min)), float(rng.max))
            s.set_option(opt, clamped)
            results.append({**item, "ok": True, "applied": clamped,
                            "clamped": abs(clamped - v) > 1e-9})
        except Exception as e:                      # noqa: BLE001
            results.append({**item, "ok": False, "error": str(e)})
    return {"ok": all(r.get("ok") for r in results), "results": results}


def set_emitter(mode: str = "on", laser_power: float | None = None,
                index: int = 0) -> dict:
    if mode not in EMITTER_MODES:
        return {"ok": False, "error": f"mode must be one of {sorted(EMITTER_MODES)}"}
    batch = [{"sensor": "depth", "option": "emitter_enabled",
              "value": EMITTER_MODES[mode]["emitter_enabled"]}]
    if laser_power is not None:
        batch.append({"sensor": "depth", "option": "laser_power",
                      "value": float(laser_power)})
    res = set_options(batch, index)
    res["mode"] = mode
    res["why"] = EMITTER_MODES[mode]["why"]
    return res


# ---------------------------------------------------------------------------
# advanced mode (the full depth-engine preset)
# ---------------------------------------------------------------------------

def advanced_get(index: int = 0) -> dict:
    if not _HAS_RS:
        return unavailable()
    dev = _device(index)
    if dev is None:
        return {"ok": False, "error": "no camera"}
    try:
        adv = rs.rs400_advanced_mode(dev)
        if not adv.is_enabled():
            return {"ok": False, "enabled": False, "error":
                    "advanced mode is off on this camera. Turning it on "
                    "restarts the device, so it is a deliberate step: call "
                    "advanced_enable() first."}
        return {"ok": True, "enabled": True, "json": adv.serialize_json()}
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": str(e)}


def advanced_enable(on: bool = True, index: int = 0) -> dict:
    if not _HAS_RS:
        return unavailable()
    dev = _device(index)
    if dev is None:
        return {"ok": False, "error": "no camera"}
    try:
        adv = rs.rs400_advanced_mode(dev)
        adv.toggle_advanced_mode(bool(on))
        return {"ok": True, "enabled": bool(on), "note":
                "the camera reboots — give it about five seconds, then "
                "restart the stream"}
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": str(e)}


def advanced_set(preset_json: str, index: int = 0) -> dict:
    """Load a depth-engine preset — the .json files Intel and the Viewer emit."""
    if not _HAS_RS:
        return unavailable()
    dev = _device(index)
    if dev is None:
        return {"ok": False, "error": "no camera"}
    try:
        json.loads(preset_json)
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": f"that is not valid preset JSON: {e}"}
    try:
        adv = rs.rs400_advanced_mode(dev)
        if not adv.is_enabled():
            adv.toggle_advanced_mode(True)
            time.sleep(5.0)
            adv = rs.rs400_advanced_mode(_device(index))
        adv.load_json(preset_json)
        return {"ok": True}
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": str(e)}


# ---------------------------------------------------------------------------
# on-chip calibration
# ---------------------------------------------------------------------------

def self_calibrate(mode: str = "calibrate", target_distance_mm: float = 600.0,
                   speed: int = 2, index: int = 0, timeout_ms: int = 25000) -> dict:
    """
    Re-run the camera's own calibration, or tare it against a known distance.

    Worth doing before a campaign on a camera that lives on a moving arm, and
    worth RECORDING: the returned health score is the camera's own opinion of
    how far out it was, and it belongs in the run manifest next to the
    hand-eye residual.

    "calibrate" fixes the stereo extrinsics (point it at a textured, flat
    scene). "tare" fixes absolute scale and needs a flat target at a MEASURED
    distance — a tare against a guessed distance makes every depth reading
    wrong by the amount you guessed wrong, consistently, which is the hardest
    kind of error to notice.
    """
    if not _HAS_RS:
        return unavailable()
    dev = _device(index)
    if dev is None:
        return {"ok": False, "error": "no camera"}
    try:
        cal = rs.auto_calibrated_device(dev)
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": f"this device does not support on-chip "
                                      f"calibration: {e}"}
    progress = {"pct": 0}

    def cb(p):
        progress["pct"] = float(p)

    try:
        cfg = json.dumps({"speed": int(speed), "scan parameter": 0})
        if mode == "tare":
            table, health = cal.run_tare_calibration(
                float(target_distance_mm), cfg, cb, timeout_ms)
        else:
            table, health = cal.run_on_chip_calibration(cfg, cb, timeout_ms)
        cal.set_calibration_table(table)
        cal.write_calibration()
        h = float(health) if not hasattr(health, "__len__") else float(health[0])
        return {"ok": True, "mode": mode, "health": round(h, 4),
                "progress": progress["pct"],
                "verdict": _health_verdict(h),
                "note": "the new calibration is written to the camera's flash "
                        "and survives a power cycle"}
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "mode": mode, "error": str(e),
                "hint": "point the camera at a flat, textured surface filling "
                        "the frame at roughly the working distance, hold it "
                        "still, and try again"}


def _health_verdict(h: float) -> str:
    a = abs(h)
    if a < 0.25:
        return "good — the camera was already well calibrated"
    if a < 0.75:
        return "moderate — the calibration has been improved; re-run to confirm"
    return ("poor — the camera was significantly out. Re-run against a better "
            "target; if it stays poor, the optics may have been knocked.")


# ---------------------------------------------------------------------------
# frame metadata and extrinsics
# ---------------------------------------------------------------------------

METADATA_KEYS = ["frame_counter", "frame_timestamp", "sensor_timestamp",
                 "backend_timestamp", "actual_exposure", "gain_level",
                 "auto_exposure", "time_of_arrival", "white_balance",
                 "temperature"]


def frame_metadata(frame) -> dict:
    """
    Per-frame timing, and WHICH CLOCK it is in.

    `hardware_clock` is the one that matters. A frame stamped in the system
    domain carries USB scheduling jitter — tens of milliseconds, variable — and
    at 0.5 m/s tool speed 10 ms of timing error is 5 mm of position error
    attributed to something else entirely.
    """
    if not _HAS_RS or frame is None:
        return {"available": False}
    out = {"available": True}
    try:
        out["timestamp_ms"] = float(frame.get_timestamp())
        dom = frame.get_frame_timestamp_domain()
        out["domain"] = str(dom).split(".")[-1]
        out["hardware_clock"] = (dom == rs.timestamp_domain.hardware_clock)
        if not out["hardware_clock"]:
            out["warning"] = (
                "Frames are stamped on the host clock, not the camera's. That "
                "adds USB scheduling jitter to every frame time. Enable global "
                "time or accept the extra term in the timing budget — but "
                "record which one you did.")
    except Exception:
        pass
    for key in METADATA_KEYS:
        try:
            attr = getattr(rs.frame_metadata_value, key)
            if frame.supports_frame_metadata(attr):
                out[key] = int(frame.get_frame_metadata(attr))
        except Exception:
            continue
    return out


def stream_extrinsics(profile) -> dict:
    """Every stream's transform to the depth stream, read from the device."""
    if not _HAS_RS or profile is None:
        return unavailable()
    try:
        streams = {}
        for sp in profile.get_streams():
            st = sp.stream_type()
            key = ("infrared%d" % sp.stream_index()) if st == rs.stream.infrared \
                else str(st).split(".")[-1]
            streams[key] = sp
        if "depth" not in streams:
            return {"available": False, "error": "depth stream is not running"}
        base = streams["depth"]
        out = {"available": True, "reference": "depth", "to": {}}
        for key, sp in streams.items():
            if key == "depth":
                continue
            e = base.get_extrinsics_to(sp)
            out["to"][key] = {
                "translation_mm": [round(v * 1000.0, 3) for v in e.translation],
                "rotation": [round(v, 6) for v in e.rotation],
            }
        return out
    except Exception as e:                          # noqa: BLE001
        return {"available": False, "error": str(e)}


# ---------------------------------------------------------------------------
# point cloud export
# ---------------------------------------------------------------------------

def export_pointcloud(depth_raw, intr, path, color_bgr=None,
                      depth_scale: float | None = None,
                      stride: int = 2, z_min: float = 0.1,
                      z_max: float = 2.0) -> dict:
    """
    One frame to a coloured PLY, from the RAW depth and the device intrinsics.

    Deliberately not rs.pointcloud(): this works from the arrays already held
    in the bridge, so it needs no second pipeline and no live camera, and it is
    testable without hardware.
    """
    if not _HAS_NP:
        return {"ok": False, "error": "numpy not available"}
    if depth_raw is None or intr is None:
        return {"ok": False, "error": "no depth frame or no intrinsics"}
    scale = depth_scale if depth_scale is not None else getattr(intr, "depth_scale", 0.001)
    d = np.asarray(depth_raw)[::stride, ::stride].astype(np.float64) * scale
    h, w = d.shape
    ys, xs = np.mgrid[0:h, 0:w]
    xs = xs * stride
    ys = ys * stride
    m = (d > z_min) & (d < z_max)
    if not np.any(m):
        return {"ok": False, "error": "no depth in range to export"}
    z = d[m]
    x = (xs[m] - intr.cx) / intr.fx * z
    y = (ys[m] - intr.cy) / intr.fy * z
    rgb = None
    if color_bgr is not None:
        c = np.asarray(color_bgr)
        if c.ndim == 3 and c.shape[0] >= ys.max() + 1 and c.shape[1] >= xs.max() + 1:
            rgb = c[ys[m], xs[m]][:, ::-1]          # BGR -> RGB

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="ascii") as fh:
        fh.write("ply\nformat ascii 1.0\n")
        fh.write(f"element vertex {int(m.sum())}\n")
        fh.write("property float x\nproperty float y\nproperty float z\n")
        if rgb is not None:
            fh.write("property uchar red\nproperty uchar green\n"
                     "property uchar blue\n")
        fh.write("end_header\n")
        if rgb is None:
            for i in range(len(z)):
                fh.write(f"{x[i]:.5f} {y[i]:.5f} {z[i]:.5f}\n")
        else:
            for i in range(len(z)):
                fh.write(f"{x[i]:.5f} {y[i]:.5f} {z[i]:.5f} "
                         f"{int(rgb[i][0])} {int(rgb[i][1])} {int(rgb[i][2])}\n")
    return {"ok": True, "path": str(p.resolve()), "n_points": int(m.sum()),
            "coloured": rgb is not None}


# ---------------------------------------------------------------------------
# infrared diagnosis — why is there no depth here?
# ---------------------------------------------------------------------------

def diagnose_ir(ir_left, ir_right=None, depth_raw=None, roi_frac: float = 0.4) -> dict:
    """
    Read the stereo pair and say why depth is missing, rather than that it is.

    Three distinguishable causes, three different fixes, and depth alone
    cannot tell them apart:
      saturation  bright pixels clipped at 255 — a specular highlight off a
                  machined face. Fix: drop exposure, or move the light.
      no texture  low local contrast — nothing for the stereo to match.
                  Fix: turn the projector on, or turn it up.
      too dark    underexposed. Fix: raise exposure or gain.
    """
    if not _HAS_NP:
        return {"ok": False, "error": "numpy not available"}
    if ir_left is None:
        return {"ok": False, "error":
                "the left infrared stream is not running. Turn it on — it is "
                "the image depth is computed from, and it is the only way to "
                "see why depth is missing."}
    a = np.asarray(ir_left)
    if a.ndim == 3:
        a = a[..., 0]
    h, w = a.shape
    fy, fx = int(h * (1 - roi_frac) / 2), int(w * (1 - roi_frac) / 2)
    roi = a[fy:h - fy, fx:w - fx].astype(np.float32)

    sat = float((roi >= 250).mean())
    dark = float((roi <= 8).mean())
    # Local contrast as the mean absolute gradient: cheap, and it is exactly
    # the quantity a block matcher needs in order to find a match at all.
    gx = np.abs(np.diff(roi, axis=1)).mean()
    gy = np.abs(np.diff(roi, axis=0)).mean()
    texture = float((gx + gy) / 2.0)
    mean = float(roi.mean())

    fill = None
    if depth_raw is not None:
        dd = np.asarray(depth_raw)
        dr = dd[fy:dd.shape[0] - fy, fx:dd.shape[1] - fx] if dd.shape[0] > 2 * fy else dd
        fill = float((dr > 0).mean())

    causes = []
    if sat > 0.05:
        causes.append({"cause": "glare",
                       "detail": f"{sat * 100:.0f}% of the centre is fully "
                                 "bright — a specular highlight",
                       "fix": "turn colour/IR auto-exposure off and lower the "
                              "exposure, or angle the part away from the light"})
    if dark > 0.35 and mean < 25:
        causes.append({"cause": "too dark",
                       "detail": f"mean infrared level {mean:.0f} of 255",
                       "fix": "raise the infrared exposure or gain, or turn "
                              "the projector on"})
    if texture < 3.0 and sat <= 0.05:
        causes.append({"cause": "no texture",
                       "detail": f"local contrast {texture:.1f} — the stereo "
                                 "has nothing to match",
                       "fix": "turn the laser projector on and raise its "
                              "power; this is the usual answer on a machined "
                              "or painted face"})
    if not causes:
        causes.append({"cause": "none found",
                       "detail": "the infrared image looks matchable",
                       "fix": "if depth is still missing, the surface may be "
                              "outside the depth range, or beyond the stereo "
                              "baseline's minimum distance (about 105 mm)"})
    return {"ok": True, "saturated_frac": round(sat, 4),
            "dark_frac": round(dark, 4), "texture": round(texture, 2),
            "mean_level": round(mean, 1),
            "depth_fill": round(fill, 3) if fill is not None else None,
            "stereo_pair": ir_right is not None,
            "causes": causes}
