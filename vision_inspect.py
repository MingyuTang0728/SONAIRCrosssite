"""
vision_inspect.py — locate a component, plan a scan over it, flag candidates.

The operator-facing pipeline, in the order it runs:

    locate()    find the part sitting on the fixture: segment it off the
                table plane, take its outline, measure it, and put its
                centroid in the robot base frame
    plan()      lay a scan path over the located outline at a fixed standoff
    detect()    flag places on the part that do not look like the rest of it

On what detect() is and is not. It finds places where the surface departs from
its own local neighbourhood — a dent, a step, a dark mark. It does NOT identify
defects. It cannot tell a scratch from a wipe mark, a pit from a shadow, or a
crack from a machining line, and it is blind to anything below the surface.
Everything it returns is a CANDIDATE for a human or a real NDT sensor to judge,
and the output is named that way throughout so the distinction survives being
pasted into a report.

That framing is not modesty. A depth camera at 300 mm resolves roughly 1-2 mm;
the defects this project ultimately cares about are smaller than its noise
floor. The value here is telling the arm where to look closely, not deciding
what is there.
"""
from __future__ import annotations

import math

try:
    import numpy as np
    _HAS_NP = True
except Exception:                       # noqa: BLE001
    np = None
    _HAS_NP = False

try:
    import cv2
    _HAS_CV = True
except Exception:                       # noqa: BLE001
    cv2 = None
    _HAS_CV = False


def _need(what: str) -> dict:
    return {"ok": False, "error": f"{what} is required for this step and is not installed"}


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def deproject(x, y, z, intr) -> list:
    fx = getattr(intr, "fx", 0.0); fy = getattr(intr, "fy", 0.0)
    cx = getattr(intr, "cx", getattr(intr, "ppx", 0.0))
    cy = getattr(intr, "cy", getattr(intr, "ppy", 0.0))
    if not fx or not fy:
        return [0.0, 0.0, float(z)]
    return [(x - cx) / fx * z, (y - cy) / fy * z, float(z)]


def _fit_plane(pts):
    """Least-squares plane through an Nx3 array. Returns (normal, d), |n| = 1."""
    c = pts.mean(axis=0)
    u, s, vt = np.linalg.svd(pts - c)
    n = vt[2]
    # Point the normal TOWARD the camera (-Z). A part sitting on the fixture is
    # CLOSER to the lens than the fixture is, so with this convention its
    # signed distance from the plane comes out positive and "height above the
    # table" means what it says. The opposite convention silently makes every
    # part negative, and the segmentation then finds nothing.
    if n[2] > 0:
        n = -n
    return n, -float(n @ c)


# ---------------------------------------------------------------------------
# 1. locate
# ---------------------------------------------------------------------------

def locate(depth_raw, intr, depth_scale: float = 0.001,
           T_base_cam=None, min_height_mm: float = 3.0,
           max_range_m: float = 1.2, min_area_px: int = 800) -> dict:
    """
    Find the component standing on the fixture plane.

    Works on DEPTH, not colour. A part and its fixture are frequently the same
    metal under the same light, so a colour segmentation separates them only
    by luck; the thing that reliably distinguishes them is that one is 20 mm
    higher than the other.

    `min_height_mm` is what counts as "standing proud". Set it below your
    part's thickness and above the plane-fit residual — at 3 mm the default
    clears typical depth noise while still catching a thin coupon.
    """
    if not _HAS_NP:
        return _need("numpy")
    if not _HAS_CV:
        return _need("opencv-python")
    if depth_raw is None:
        return {"ok": False, "error": "no depth frame — is the depth stream on?"}

    d = np.asanyarray(depth_raw).astype(np.float32) * depth_scale
    h, w = d.shape[:2]
    valid = (d > 0.05) & (d < max_range_m)
    if valid.sum() < 500:
        return {"ok": False, "error":
                f"only {int(valid.sum())} valid depth pixels. The part may be out "
                f"of range, or the surface may be too specular for stereo — try "
                f"the laser emitter on, or a matte reference surface."}

    # --- fit the dominant plane (the table / fixture) -----------------------
    ys, xs = np.nonzero(valid)
    step = max(1, len(xs) // 8000)
    sx, sy = xs[::step], ys[::step]
    pts = np.array([deproject(x, y, d[y, x], intr) for x, y in zip(sx, sy)])

    best_n, best_d, best_inl = None, 0.0, 0
    rng = np.random.default_rng(0)
    for _ in range(120):
        idx = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        dd = -float(n @ p0)
        inl = int(np.sum(np.abs(pts @ n + dd) < 0.004))
        if inl > best_inl:
            best_n, best_d, best_inl = n, dd, inl
    if best_n is None or best_inl < len(pts) * 0.15:
        return {"ok": False, "error":
                "no dominant plane found. This step expects the part to be "
                "sitting on a flat fixture that fills much of the view."}
    if best_n[2] > 0:                      # normal toward the camera, see _fit_plane
        best_n, best_d = -best_n, -best_d

    # refine on the inliers, so the plane is not defined by three lucky points
    resid = pts @ best_n + best_d
    inliers = pts[np.abs(resid) < 0.004]
    if len(inliers) > 50:
        best_n, best_d = _fit_plane(inliers)

    # --- height above that plane, per pixel ---------------------------------
    height = np.zeros((h, w), np.float32)
    yy, xx = np.nonzero(valid)
    zz = d[yy, xx]
    fx = getattr(intr, "fx", 1.0); fy = getattr(intr, "fy", 1.0)
    cx = getattr(intr, "cx", getattr(intr, "ppx", w / 2))
    cy = getattr(intr, "cy", getattr(intr, "ppy", h / 2))
    px = (xx - cx) / fx * zz
    py = (yy - cy) / fy * zz
    height[yy, xx] = px * best_n[0] + py * best_n[1] + zz * best_n[2] + best_d

    mask = ((height > min_height_mm / 1000.0) & valid).astype(np.uint8) * 255
    # Close pinholes from missing stereo returns, then drop speckle. Doing it
    # in this order matters: opening first would erase a thin part entirely.
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

    # CHAIN_APPROX_NONE, not SIMPLE: SIMPLE collapses a rectangle to four
    # corners, and a path planner given four points plans over a quadrilateral
    # instead of the part. The outline is resampled to a fixed count below.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area_px]
    if not contours:
        return {"ok": False, "error":
                f"nothing found standing more than {min_height_mm:.0f} mm above "
                f"the fixture. Lower the height threshold, or check the part is "
                f"in view and within {max_range_m:.1f} m."}

    c = max(contours, key=cv2.contourArea)
    part = np.zeros_like(mask); cv2.drawContours(part, [c], -1, 255, -1)

    M = cv2.moments(c)
    ux = M["m10"] / M["m00"]; uy = M["m01"] / M["m00"]
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)

    pm = part > 0
    hv = height[pm & valid]
    top_h = float(np.percentile(hv, 95)) if hv.size else 0.0
    zc = float(np.median(d[pm & valid])) if (pm & valid).sum() else 0.0
    centroid_cam = deproject(ux, uy, zc, intr)

    # Pixel extents to millimetres at the part's own distance. Using the
    # centroid depth rather than the plane's keeps a tall part from reading
    # larger than it is.
    mm_per_px_x = zc / fx * 1000.0
    mm_per_px_y = zc / fy * 1000.0
    (rw_px, rh_px) = rect[1]

    out = {
        "ok": True,
        "contour_px": [[int(p[0][0]), int(p[0][1])] for p in c],
        "oriented_box_px": [[float(p[0]), float(p[1])] for p in box],
        "centroid_px": [float(ux), float(uy)],
        "bbox_px": [int(v) for v in cv2.boundingRect(c)],
        "angle_deg": float(rect[2]),
        "size_mm": [float(rw_px * mm_per_px_x), float(rh_px * mm_per_px_y)],
        "height_mm": top_h * 1000.0,
        "area_mm2": float(cv2.contourArea(c) * mm_per_px_x * mm_per_px_y),
        "centroid_cam_m": centroid_cam,
        "plane": {"normal": [float(v) for v in best_n], "offset": float(best_d),
                  "tilt_deg": math.degrees(math.acos(max(-1, min(1, abs(float(best_n[2]))))))},
        "image_size": [int(w), int(h)],
        "fill": float((pm & valid).sum() / max(pm.sum(), 1)),
        "mm_per_px": [mm_per_px_x, mm_per_px_y],
    }

    if T_base_cam is not None:
        T = np.asarray(T_base_cam, dtype=float).reshape(4, 4)
        p = T[:3, :3] @ np.array(centroid_cam) + T[:3, 3]
        out["centroid_base_m"] = [float(v) for v in p]
        out["contour_base_m"] = _contour_to_base(c, d, intr, T, valid)

    out["_mask"] = part          # kept in-process for detect(); never serialised
    out["_height"] = height
    return out


def _contour_to_base(contour, depth_m, intr, T, valid, n_points: int = 160) -> list:
    """
    The outline, in robot base coordinates — what the path planner needs.

    Resampled to a fixed number of points rather than strided: a strided walk
    over a contour returns a handful of points on a simple shape and hundreds
    on a ragged one, so the planner's input density would depend on how noisy
    the segmentation happened to be.
    """
    pts = []
    h, w = depth_m.shape[:2]
    total = len(contour)
    if total == 0:
        return pts
    stride = max(1, total // max(1, n_points))
    for i, p in enumerate(contour):
        if i % stride:
            continue
        x, y = int(p[0][0]), int(p[0][1])
        # Walk inward a little: the outline sits on the depth edge, where
        # stereo is least reliable, and an edge pixel often has no return.
        found = None
        for r in range(0, 6):
            for dx, dy in ((0, 0), (r, 0), (-r, 0), (0, r), (0, -r)):
                xx, yy = min(max(x + dx, 0), w - 1), min(max(y + dy, 0), h - 1)
                if valid[yy, xx]:
                    found = (xx, yy, float(depth_m[yy, xx]))
                    break
            if found:
                break
        if not found:
            continue
        c = deproject(found[0], found[1], found[2], intr)
        b = T[:3, :3] @ np.array(c) + T[:3, 3]
        pts.append([float(v) for v in b])
    return pts


# ---------------------------------------------------------------------------
# 2. plan
# ---------------------------------------------------------------------------

def plan(located: dict, spacing_mm: float = 5.0, step_mm: float = 5.0,
         standoff_mm: float = 100.0, margin_mm: float = 5.0,
         mode: str = "raster", tool_rotvec=(0.0, math.pi, 0.0)) -> dict:
    """
    Lay a path over the located component, in robot base coordinates.

    `raster`  serpentine fill of the outline — full coverage, for a first pass
    `contour` follow the outline inward — edges first, where defects cluster

    Waypoints stay inside the measured outline rather than its bounding box:
    for anything other than a rectangle those differ by a lot, and the
    difference is entirely time spent scanning the fixture.
    """
    if not _HAS_NP:
        return _need("numpy")
    if not located.get("ok"):
        return {"ok": False, "error": "locate the component first"}
    pts = located.get("contour_base_m")
    if not pts or len(pts) < 3:
        return {"ok": False, "error":
                "the outline has no robot-frame coordinates. Set the hand-eye "
                "calibration so camera points can be placed in the base frame."}

    P = np.array(pts, dtype=float)
    z_top = float(np.percentile(P[:, 2], 90))
    poly = P[:, :2]

    # Shrink the polygon by the margin, about its own centroid. Crude next to a
    # true offset, but it never self-intersects, which a naive offset does on
    # any concave outline.
    cen = poly.mean(axis=0)
    r = np.linalg.norm(poly - cen, axis=1)
    scale = max(0.0, 1.0 - (margin_mm / 1000.0) / max(r.mean(), 1e-6))
    poly_in = cen + (poly - cen) * scale

    rx, ry, rz = tool_rotvec
    z = z_top + standoff_mm / 1000.0
    waypoints = []

    if mode == "contour":
        # Resample the closed outline at a constant arc length. Striding by
        # index instead spaces points by however finely the segmentation
        # happened to trace that stretch of edge, which is not a distance.
        step_m = max(step_mm / 1000.0, 1e-4)
        closed = np.vstack([poly_in, poly_in[:1]])
        seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(cum[-1])
        n = max(1, int(total / step_m))
        for k in range(n):
            dist = k * step_m
            i = int(np.searchsorted(cum, dist, side="right")) - 1
            i = min(max(i, 0), len(seg) - 1)
            t = (dist - cum[i]) / seg[i] if seg[i] > 1e-12 else 0.0
            x, y = closed[i] + (closed[i + 1] - closed[i]) * t
            waypoints.append({"type": f"CONTOUR_{k}",
                              "coords": [float(x), float(y), z, rx, ry, rz]})

    else:
        x0, y0 = poly_in.min(axis=0)
        x1, y1 = poly_in.max(axis=0)
        n_lines = max(1, int((y1 - y0) / (spacing_mm / 1000.0)))
        for i in range(n_lines + 1):
            yy = y0 + i * spacing_mm / 1000.0
            if yy > y1:
                break
            xs = _row_span(poly_in, yy)
            if not xs:
                continue
            a, b = xs
            n_steps = max(1, int((b - a) / (step_mm / 1000.0)))
            row = [a + j * (b - a) / n_steps for j in range(n_steps + 1)]
            if i % 2:
                row.reverse()
            for j, xx in enumerate(row):
                waypoints.append({
                    "type": f"SCAN_L{i}_{'END' if j == len(row) - 1 else 'PT'}",
                    "coords": [float(xx), float(yy), z, rx, ry, rz]})

    if not waypoints:
        return {"ok": False, "error":
                "the margin is larger than the part. Reduce it, or check the "
                "outline is the component and not a reflection."}

    return {
        "ok": True, "mode": mode, "waypoints": waypoints,
        "n_waypoints": len(waypoints),
        "standoff_mm": standoff_mm, "spacing_mm": spacing_mm,
        "surface_z_m": z_top, "path_z_m": z,
        "note": ("Path follows the measured outline at a fixed height above the "
                 "part's top surface. It assumes the top is roughly flat; a "
                 "stepped part needs the height-field planner in scan3d."),
    }


def _row_span(poly, y):
    """Where a horizontal line at `y` enters and leaves the polygon."""
    xs = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1) if y2 != y1 else 0.0
            xs.append(x1 + t * (x2 - x1))
    if len(xs) < 2:
        return None
    return min(xs), max(xs)


# ---------------------------------------------------------------------------
# 3. detect
# ---------------------------------------------------------------------------

def detect(color_bgr, located: dict, intr, depth_scale: float = 0.001,
           depth_thresh_mm: float = 1.5, visual_thresh: int = 22,
           min_area_mm2: float = 0.5, T_base_cam=None) -> dict:
    """
    Flag places on the part that differ from their own surroundings.

    Two independent channels, reported separately because they fail differently:

      surface  height residual against a plane fitted to the part's own top.
               Catches dents, steps and burrs. Blind to anything shallower
               than the depth noise, which at 300 mm is around 1-2 mm — so a
               genuine scratch will not appear here, and its absence means
               nothing.

      visual   local darkness against a blurred copy of the image. Catches
               marks, scratches and staining. Also catches shadows, coolant,
               fingerprints and machining lines, which is why every hit is a
               candidate rather than a finding.

    A candidate confirmed by both channels is worth looking at first; that is
    all `both` means.
    """
    if not _HAS_NP:
        return _need("numpy")
    if not _HAS_CV:
        return _need("opencv-python")
    if not located.get("ok"):
        return {"ok": False, "error": "locate the component first"}

    mask = located.get("_mask")
    height = located.get("_height")
    if mask is None or height is None:
        return {"ok": False, "error": "locate() result is missing its working "
                                      "arrays; re-run the locate step"}

    pm = mask > 0
    mmx, mmy = located.get("mm_per_px", [1.0, 1.0])
    px_area_mm2 = mmx * mmy
    cands = []

    # --- surface channel ----------------------------------------------------
    ys_p, xs_p = np.nonzero(pm)
    if ys_p.size > 100:
        # Baseline = a quadratic surface fitted to the part's OWN top, solved
        # robustly. Two reasons not to use a blur: cv2's median filter will not
        # take float32 at a useful kernel size, and more importantly a blurred
        # baseline is dragged toward whatever defect sits under the kernel, so
        # a large dent partly hides itself. A fit over the whole face is not.
        zs = height[ys_p, xs_p].astype(np.float64)
        x0, y0 = xs_p.mean(), ys_p.mean()
        sx = (xs_p - x0) / max(xs_p.std(), 1.0)
        sy = (ys_p - y0) / max(ys_p.std(), 1.0)
        A = np.column_stack([np.ones_like(sx), sx, sy, sx * sx, sx * sy, sy * sy])
        wgt = np.ones_like(zs)
        coef = np.zeros(6)
        for _ in range(3):        # IRLS: three passes is plenty to shed outliers
            Aw = A * wgt[:, None]
            coef, *_ = np.linalg.lstsq(Aw, zs * wgt, rcond=None)
            r = zs - A @ coef
            scale = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-9
            wgt = 1.0 / (1.0 + (r / (3.0 * scale)) ** 2)
        resid_flat = (zs - A @ coef) * 1000.0         # mm
        resid = np.zeros_like(height, dtype=np.float32)
        resid[ys_p, xs_p] = resid_flat.astype(np.float32)
        hit = (np.abs(resid) > depth_thresh_mm) & pm
        hit = cv2.morphologyEx(hit.astype(np.uint8) * 255, cv2.MORPH_OPEN,
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        cnts, _ = cv2.findContours(hit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a_mm2 = cv2.contourArea(c) * px_area_mm2
            if a_mm2 < min_area_mm2:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            x, y = M["m10"] / M["m00"], M["m01"] / M["m00"]
            dev = float(resid[int(y), int(x)])
            cands.append({"channel": "surface", "px": [float(x), float(y)],
                          "area_mm2": float(a_mm2), "deviation_mm": dev,
                          "kind": "protrusion" if dev > 0 else "depression",
                          "score": float(min(1.0, abs(dev) / (depth_thresh_mm * 4)))})

    # --- visual channel -----------------------------------------------------
    if color_bgr is not None:
        img = np.asanyarray(color_bgr)
        if img.ndim == 3:
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            g = img
        if g.shape[:2] != mask.shape[:2]:
            g = cv2.resize(g, (mask.shape[1], mask.shape[0]))
        bg = cv2.medianBlur(g, 31)
        dark = cv2.subtract(bg, g)
        hit = ((dark > visual_thresh) & pm).astype(np.uint8) * 255
        hit = cv2.morphologyEx(hit, cv2.MORPH_OPEN,
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        cnts, _ = cv2.findContours(hit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a_mm2 = cv2.contourArea(c) * px_area_mm2
            if a_mm2 < min_area_mm2:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            x, y = M["m10"] / M["m00"], M["m01"] / M["m00"]
            contrast = float(dark[int(y), int(x)])
            cands.append({"channel": "visual", "px": [float(x), float(y)],
                          "area_mm2": float(a_mm2), "contrast": contrast,
                          "kind": "mark",
                          "score": float(min(1.0, contrast / (visual_thresh * 4)))})

    # --- agreement between channels ----------------------------------------
    for a in cands:
        a["both"] = any(
            b["channel"] != a["channel"]
            and math.dist(a["px"], b["px"]) < 15
            for b in cands)
        if a["both"]:
            a["score"] = min(1.0, a["score"] * 1.5)

    # --- positions ----------------------------------------------------------
    for a in cands:
        x, y = int(a["px"][0]), int(a["px"][1])
        z = float(np.median(height[max(0, y - 2):y + 3, max(0, x - 2):x + 3]))
        a["point_cam_m"] = deproject(x, y, z, intr) if z else None
        if T_base_cam is not None and a["point_cam_m"]:
            T = np.asarray(T_base_cam, dtype=float).reshape(4, 4)
            p = T[:3, :3] @ np.array(a["point_cam_m"]) + T[:3, 3]
            a["point_base_m"] = [float(v) for v in p]

    cands.sort(key=lambda c: -c["score"])
    return {
        "ok": True,
        "candidates": cands[:200],
        "n_total": len(cands),
        "n_surface": sum(1 for c in cands if c["channel"] == "surface"),
        "n_visual": sum(1 for c in cands if c["channel"] == "visual"),
        "n_both": sum(1 for c in cands if c["both"]),
        "settings": {"depth_thresh_mm": depth_thresh_mm,
                     "visual_thresh": visual_thresh,
                     "min_area_mm2": min_area_mm2},
        "note": ("Candidates, not findings. The surface channel is blind below "
                 "the depth noise floor (1-2 mm at 300 mm), and the visual "
                 "channel cannot tell a scratch from a wipe mark or a shadow. "
                 "Use these to decide where to look closely."),
    }
