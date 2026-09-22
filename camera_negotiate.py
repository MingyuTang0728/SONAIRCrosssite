"""
camera_negotiate.py — pick a stream configuration the device will actually accept.

`Couldn't resolve requests` is librealsense saying "no device mode matches what
you asked for". It is the single most common way a RealSense bring-up fails,
and the message names neither the offending stream nor a working alternative.

Two causes account for nearly all of it:

  1. The requested resolution / frame-rate / format triple does not exist on
     this device. USB 2.1 is the usual reason — the same D435i that offers
     dozens of modes on USB 3 offers a handful on USB 2, so a config that
     worked on one port fails on another with no visible difference.

  2. The motion module (accel + gyro) is requested by two pipelines at once.
     The IMU lives behind a single sensor; a second pipeline asking for it
     fails the whole config, taking depth and colour down with it even though
     nothing is wrong with them.

`negotiate` fixes the first by asking the device what it supports and choosing
the nearest match. The second is avoided by construction: motion streams belong
to exactly one owner (bench_agent), and the video pipeline never asks for them.
"""
from __future__ import annotations

import logging

log = logging.getLogger("camera.negotiate")

try:
    import pyrealsense2 as rs
    _HAS_RS = True
except Exception:       # noqa: BLE001
    rs = None
    _HAS_RS = False


def supported_video_modes(device) -> dict[str, list[tuple[int, int, int]]]:
    """
    {"depth": [(w, h, fps), ...], "color": [...], "infrared": [...]}

    Only the formats the agent decodes. A mode the agent cannot read is not a
    mode it can fall back to, so offering it would just move the failure.
    """
    out: dict[str, list[tuple[int, int, int]]] = {}
    if not _HAS_RS or device is None:
        return out
    for sensor in device.query_sensors():
        try:
            profiles = sensor.get_stream_profiles()
        except Exception:
            continue
        for p in profiles:
            try:
                v = p.as_video_stream_profile()
                if not v:
                    continue
                st = v.stream_type()
                fmt = v.format()
                if st == rs.stream.depth and fmt == rs.format.z16:
                    key = "depth"
                elif st == rs.stream.color and fmt in (rs.format.bgr8, rs.format.rgb8):
                    key = "color"
                elif st == rs.stream.infrared and fmt == rs.format.y8:
                    key = "infrared"
                else:
                    continue
                out.setdefault(key, []).append((v.width(), v.height(), v.fps()))
            except Exception:
                continue
    for k in out:
        out[k] = sorted(set(out[k]))
    return out


def nearest_mode(modes: list[tuple[int, int, int]], w: int, h: int, fps: int):
    """
    The supported mode closest to what was asked for.

    Pixel count is weighted far above frame rate: halving the resolution
    changes what you can measure, halving the frame rate only changes how
    often you measure it. A reconstruction survives 15 fps; it does not
    survive being handed 424x240 when it expected 848x480 without being told.
    """
    if not modes:
        return None
    want_px = w * h

    def cost(m):
        mw, mh, mf = m
        px_err = abs(mw * mh - want_px) / max(want_px, 1)
        fps_err = abs(mf - fps) / max(fps, 1)
        # Prefer an exact aspect-ratio match; a letterboxed image quietly
        # changes the intrinsics' relationship to the scene.
        aspect_err = abs((mw / mh) - (w / h)) if h and mh else 0.0
        return px_err * 10.0 + aspect_err * 5.0 + fps_err
    return min(modes, key=cost)


def negotiate(device, want: dict) -> tuple[dict, list[str]]:
    """
    Turn a requested config into one the device supports.

    Returns (resolved, notes). `notes` is non-empty whenever something was
    changed, and the caller is expected to surface it — silently substituting a
    mode is how a panel ends up lying about what the data was captured at.
    """
    notes: list[str] = []
    resolved = dict(want)
    if not _HAS_RS or device is None:
        return resolved, notes

    modes = supported_video_modes(device)
    if not modes:
        notes.append("could not enumerate device modes; sending the request unchanged")
        return resolved, notes

    def fix(stream_key, res_field, fps_field, label):
        if stream_key not in modes:
            return
        try:
            w, h = (int(x) for x in str(resolved.get(res_field, "")).split("x"))
            fps = int(resolved.get(fps_field, 30))
        except Exception:
            return
        if (w, h, fps) in modes[stream_key]:
            return
        best = nearest_mode(modes[stream_key], w, h, fps)
        if not best:
            return
        bw, bh, bf = best
        resolved[res_field] = f"{bw}x{bh}"
        resolved[fps_field] = bf
        notes.append(f"{label} {w}x{h}@{fps} is not supported on this device "
                     f"and connection; using {bw}x{bh}@{bf}")

    fix("depth", "stereo_res", "stereo_fps", "depth")
    fix("color", "rgb_res", "rgb_fps", "colour")

    # IR shares the stereo sensor, so it must use the depth mode exactly.
    if (resolved.get("ir1_en") or resolved.get("ir2_en")) and "infrared" in modes:
        try:
            w, h = (int(x) for x in str(resolved["stereo_res"]).split("x"))
            fps = int(resolved["stereo_fps"])
            if (w, h, fps) not in modes["infrared"]:
                resolved["ir1_en"] = False
                resolved["ir2_en"] = False
                notes.append(f"infrared does not support {w}x{h}@{fps}; IR disabled "
                             f"so depth and colour can still start")
        except Exception:
            pass

    return resolved, notes


def usb_generation(device) -> str:
    if not _HAS_RS or device is None:
        return "?"
    try:
        return str(device.get_info(rs.camera_info.usb_type_descriptor))
    except Exception:
        return "?"


def describe_failure(device, want: dict, exc: Exception) -> str:
    """
    A message that names the likely cause instead of repeating the SDK's.

    "Couldn't resolve requests" tells an operator nothing actionable. This
    says which stream is impossible and what the device does offer.
    """
    base = str(exc)
    if "resolve" not in base.lower():
        return base

    usb = usb_generation(device)
    modes = supported_video_modes(device)
    lines = [f"the camera rejected this stream combination ({base})"]

    if usb.startswith("2"):
        lines.append(f"this camera is connected over USB {usb}. On USB 2 a D435i "
                     f"offers only a handful of modes — use a USB 3 port (blue) "
                     f"and the cable that came with the camera.")
    for key, field, fps_field in (("depth", "stereo_res", "stereo_fps"),
                                  ("color", "rgb_res", "rgb_fps")):
        if key in modes:
            top = sorted(modes[key], key=lambda m: -(m[0] * m[1]))[:4]
            offered = ", ".join(f"{w}x{h}@{f}" for w, h, f in top)
            lines.append(f"{key}: asked for {want.get(field)}@{want.get(fps_field)}; "
                         f"device offers {offered}")
    return "  |  ".join(lines)
