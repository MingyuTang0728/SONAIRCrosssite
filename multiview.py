"""
multiview.py — from a box drawn on the live image to a scan path on a 3D model.

The single-shot pipeline in vision_inspect works from one depth frame, and
that is its limit: one viewpoint sees one side. A stereo camera looking down
at a machined part measures the top face and nothing else, and it measures the
top face badly wherever the surface is steep enough to break the stereo match.
Every vertical wall, every undercut, and every glancing face is simply absent —
not noisy, absent — so a path planned on it plans over holes.

This module is the answer to that, and it follows the operator's own sequence:

  1. REGION      The operator draws a box round the part in the camera image.
                 That box plus the depth frame plus the hand-eye transform
                 gives the part's position and extent IN THE ROBOT'S FRAME,
                 which is the only frame in which viewpoints mean anything.
  2. VIEWPOINTS  Given the extent, decide how many photographs to take and
                 from where. This is arithmetic, not taste: the standoff comes
                 from the part's size and the camera's focal length so the part
                 fills the frame; the view count comes from the required
                 overlap between adjacent views; the tilt comes from how tall
                 the part is relative to its footprint. Then every pose is
                 checked against the robot's reach and the cell envelope, and
                 the survivors are ordered to make the arm's trip short.
  3. FUSION      Each captured view is deprojected, placed in the base frame,
                 cropped to the region (so the fixture and the far wall never
                 enter the model), and merged into one voxel grid.
  4. MODEL       The fused cloud is cleaned, localised, and handed to the
                 existing surface path planner — so a path planned on the
                 fused model comes out in the same shape the console already
                 previews and executes.

The region crop in step 3 is what makes this work at all. Without it, eight
views from eight angles each contribute their own view of the bench, the
fixture and the far wall, and the "component" that falls out of clustering is
whichever of those happened to be biggest.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

try:
    import numpy as np
    _HAS_NP = True
except Exception as e:                              # noqa: BLE001
    np = None
    _HAS_NP = False
    _NP_ERR = str(e)

try:
    import scan3d
    _HAS_SCAN = True
except Exception as e:                              # noqa: BLE001
    scan3d = None
    _HAS_SCAN = False
    _SCAN_ERR = str(e)


# UR5e geometry. Reach is the published 850 mm; the inner radius keeps planned
# poses out of the column the arm cannot fold into.
DEFAULT_REACH = {"max_radius_m": 0.82, "min_radius_m": 0.20, "min_z_m": 0.02}


def available() -> tuple[bool, str]:
    if not _HAS_NP:
        return False, f"numpy not importable ({_NP_ERR})"
    if not _HAS_SCAN:
        return False, f"scan3d not importable ({_SCAN_ERR})"
    return True, ""


# ---------------------------------------------------------------------------
# 1. region from an image ROI
# ---------------------------------------------------------------------------

def region_from_roi(depth_raw, intr, roi, T_base_cam=None,
                    depth_scale: float | None = None,
                    z_min: float = 0.08, z_max: float = 1.5,
                    trim_percentile: float = 2.0) -> dict:
    """
    The box the operator drew -> the part's extent in the robot's base frame.

    `roi` is (x, y, w, h) in DEPTH-IMAGE pixels. The depth and colour streams
    are aligned upstream, so a box drawn on the colour image indexes the depth
    image directly; if that alignment is ever turned off this is the first
    thing that breaks, which is why the depth fill of the box is reported.

    Extent is taken from trimmed percentiles rather than min/max. A single
    stray pixel on the fixture edge — and there is always one — would otherwise
    set the part's size, and every viewpoint computed from that size would be
    wrong in the same direction.
    """
    ok, why = available()
    if not ok:
        return {"ok": False, "error": why}
    if depth_raw is None:
        return {"ok": False, "error": "no depth frame — start the depth stream"}
    if intr is None:
        return {"ok": False, "error":
                "camera intrinsics unavailable. They must be read from the "
                "camera, never assumed."}

    d = np.asarray(depth_raw)
    if d.ndim != 2:
        return {"ok": False, "error": "depth frame is not a 2-D raw array"}
    H, W = d.shape
    x, y, w, h = (int(round(float(v))) for v in roi)
    x = max(0, min(W - 2, x))
    y = max(0, min(H - 2, y))
    w = max(2, min(W - x, w))
    h = max(2, min(H - y, h))

    scale = depth_scale if depth_scale is not None else getattr(intr, "depth_scale", 0.001)
    sub = d[y:y + h, x:x + w].astype(np.float64) * scale
    valid = (sub > z_min) & (sub < z_max)
    n_valid = int(valid.sum())
    fill = n_valid / float(sub.size)
    if n_valid < 50:
        return {"ok": False, "fill": round(fill, 3), "error":
                "almost no depth inside that box. The camera cannot measure "
                "this surface from here — move closer, turn the laser "
                "projector on, or cut the glare. A shiny machined face at a "
                "glancing angle returns nothing at all."}

    ys, xs = np.nonzero(valid)
    z = sub[valid]
    px = xs + x
    py = ys + y
    X = (px - intr.cx) / intr.fx * z
    Y = (py - intr.cy) / intr.fy * z
    pts_cam = np.stack([X, Y, z], axis=1)

    frame = "camera"
    pts = pts_cam
    if T_base_cam is not None:
        T = np.asarray(T_base_cam, dtype=float).reshape(4, 4)
        pts = (T[:3, :3] @ pts_cam.T).T + T[:3, 3]
        frame = "base"

    lo = np.percentile(pts, trim_percentile, axis=0)
    hi = np.percentile(pts, 100.0 - trim_percentile, axis=0)
    centre = (lo + hi) / 2.0
    size = np.maximum(hi - lo, 1e-4)

    out = {
        "ok": True,
        "frame": frame,
        "roi": [x, y, w, h],
        "n_points": n_valid,
        "fill": round(fill, 3),
        "centre_m": [round(float(v), 5) for v in centre],
        "size_mm": [round(float(v) * 1000.0, 1) for v in size],
        "aabb_min_m": [round(float(v), 5) for v in lo],
        "aabb_max_m": [round(float(v), 5) for v in hi],
        "distance_mm": round(float(np.median(z)) * 1000.0, 1),
        "_centre": centre.tolist(),
        "_size": size.tolist(),
    }
    if frame == "camera":
        out["warning"] = ("No hand-eye calibration, so this region is in the "
                          "CAMERA's frame. Viewpoints cannot be planned until "
                          "the calibration is done — the robot has no way to "
                          "know where the camera was looking.")
    if fill < 0.35:
        out["note"] = (f"Only {fill * 100:.0f}% of the box returned depth. The "
                       "model will have holes where it did not.")
    return out


# ---------------------------------------------------------------------------
# 2. viewpoint planning
# ---------------------------------------------------------------------------

def standoff_for(size_m, intr, fill_fraction: float = 0.65) -> float:
    """
    How far back to put the camera so the part fills the frame.

    From the pinhole relation, an object of width S imaged across a fraction f
    of a sensor of width W pixels at focal length fx sits at

        z = S . fx / (f . W)

    Doing it this way rather than with a fixed 300 mm means a 40 mm bracket and
    a 400 mm casting both arrive at a sensible resolution, and it is the same
    arithmetic for both axes — the binding one wins.
    """
    sx, sy = float(size_m[0]), float(size_m[1])
    diag = math.sqrt(sx * sx + sy * sy)
    zx = diag * intr.fx / max(1.0, fill_fraction * intr.width)
    zy = diag * intr.fy / max(1.0, fill_fraction * intr.height)
    return max(zx, zy)


# How much of a part's surface one view usefully measures, in azimuth. The
# geometric answer for a convex body is 180 deg, but the outer 30 deg on each
# side arrives at such a glancing angle that the stereo match fails there —
# so the honest working figure is nearer 120, and planning against 180 is how
# a scan ends up with seams exactly where the views were supposed to meet.
USEFUL_VIEW_AZIMUTH_DEG = 120.0


def views_for_overlap(overlap: float = 0.55) -> int:
    """
    How many azimuths are needed so adjacent views share `overlap` of what
    each of them measures.

    Fusion needs genuine overlap, not adjacency: two views that merely touch
    give the voxel grid nothing to average, so every voxel is single-hit,
    `min_hits=2` discards the lot, and the model comes out empty for a reason
    that looks nothing like its cause.
    """
    overlap = min(0.9, max(0.0, overlap))
    step = max(5.0, (1.0 - overlap) * USEFUL_VIEW_AZIMUTH_DEG)
    return int(max(6, min(24, math.ceil(360.0 / step))))


def plan_views(region: dict, intr=None, T_tcp_cam=None, *,
               n_views: int | None = None,
               standoff_mm: float | None = None,
               tilt_deg: float | None = None,
               rings: int | None = None,
               overlap: float = 0.55,
               fill_fraction: float = 0.65,
               reach: dict | None = None,
               envelope: dict | None = None,
               start_pose=None) -> dict:
    """
    Where to photograph the part from, and in what order.

    Every number here is derived from the part unless the caller pins it:

      standoff  from the part's footprint and the camera's focal length, so
                the part fills the frame at every view (`standoff_for`).
      tilt      from the part's height against its footprint. A flat plate
                needs a shallow tilt — steep views of a flat top see nothing
                new and lose the stereo match. A tall part needs a steep one,
                because its walls are invisible from above.
      count     from the overlap adjacent views must share (`views_for_overlap`),
                with a second ring added when the part is tall enough that one
                elevation cannot cover it.

    Poses that the arm cannot reach, or that leave the cell envelope, are
    dropped WITH A REASON rather than silently, so "only 5 of 12 views" comes
    with an explanation instead of looking like a bug.
    """
    ok, why = available()
    if not ok:
        return {"ok": False, "error": why}
    if not region.get("ok"):
        return {"ok": False, "error": "locate the part first"}
    if region.get("frame") != "base":
        return {"ok": False, "error":
                "the region is in the camera's frame — hand-eye calibration "
                "must be done before viewpoints can be planned"}
    if T_tcp_cam is None:
        return {"ok": False, "error": "hand-eye calibration (T_tcp_cam) not set"}

    centre = np.asarray(region["_centre"], dtype=float)
    size = np.asarray(region["_size"], dtype=float)
    footprint = float(max(size[0], size[1]))
    height = float(size[2])

    if standoff_mm is None:
        if intr is None:
            return {"ok": False, "error": "camera intrinsics needed to choose "
                                          "a standoff, or pass standoff_mm"}
        standoff = standoff_for(size, intr, fill_fraction)
        standoff = float(min(max(standoff, 0.18), 0.80))
        standoff_source = "computed from the part size and the lens"
    else:
        standoff = float(standoff_mm) / 1000.0
        standoff_source = "set by the operator"

    if tilt_deg is None:
        # Aspect drives it: a plate barely taller than it is wide wants a
        # shallow tilt, a block wants a steep one.
        aspect = height / max(1e-3, footprint)
        tilt = math.degrees(math.atan(min(1.6, max(0.18, aspect * 1.7))))
        tilt = float(min(62.0, max(18.0, tilt)))
        tilt_source = "from the part's height against its footprint"
    else:
        tilt = float(tilt_deg)
        tilt_source = "set by the operator"

    if n_views is None:
        n_views = views_for_overlap(overlap)
        count_source = f"for {overlap * 100:.0f}% overlap between views"
    else:
        n_views = int(n_views)
        count_source = "set by the operator"

    if rings is None:
        # A second elevation only earns its time on a part tall enough that one
        # elevation genuinely cannot see both the top and the walls.
        rings = 2 if height > 0.35 * footprint and height > 0.015 else 1
    rings = max(1, min(3, int(rings)))

    reach = {**DEFAULT_REACH, **(reach or {})}
    poses, rejected = [], []
    idx = 0
    for r in range(rings):
        # Rings are offset in azimuth so the second ring's views fall between
        # the first's, not on top of them.
        ring_tilt = tilt if rings == 1 else tilt * (0.55 + 0.75 * r / max(1, rings - 1))
        ring_tilt = min(72.0, ring_tilt)
        phase = math.pi / n_views * r
        for i in range(n_views):
            az = 2.0 * math.pi * i / n_views + phase
            el = math.radians(90.0 - ring_tilt)     # from horizontal
            pos = centre + standoff * np.array([
                math.cos(el) * math.cos(az),
                math.cos(el) * math.sin(az),
                math.sin(el)])
            T_cam = _look_at(pos, centre)
            T_tcp = T_cam @ np.linalg.inv(np.asarray(T_tcp_cam, dtype=float))
            tcp_pose = scan3d.matrix_to_pose(T_tcp)
            why_not = _reject_reason(T_tcp[:3, 3], reach, envelope)
            entry = {
                "index": idx,
                "ring": r,
                "azimuth_deg": round(math.degrees(az) % 360.0, 1),
                "tilt_deg": round(ring_tilt, 1),
                "camera_pose": [round(float(v), 5) for v in scan3d.matrix_to_pose(T_cam)],
                "tcp_pose": [round(float(v), 5) for v in tcp_pose],
                "camera_xyz_mm": [round(float(v) * 1000.0, 1) for v in pos],
            }
            idx += 1
            if why_not:
                entry["rejected"] = why_not
                rejected.append(entry)
            else:
                poses.append(entry)

    if not poses:
        return {"ok": False, "n_rejected": len(rejected), "rejected": rejected,
                "error": ("every planned viewpoint is out of reach. The part "
                          "is too far from the robot, or the standoff is too "
                          "large for the space — move the fixture closer or "
                          "reduce the standoff.")}

    order = _order_poses(poses, start_pose)
    for k, p in enumerate(order):
        p["order"] = k

    cov = _coverage(order)
    return {
        "ok": True,
        "views": order,
        "n": len(order),
        "n_rejected": len(rejected),
        "rejected": rejected[:8],
        "standoff_mm": round(standoff * 1000.0, 1),
        "standoff_source": standoff_source,
        "tilt_deg": round(tilt, 1),
        "tilt_source": tilt_source,
        "rings": rings,
        "count_source": count_source,
        "azimuth_coverage_deg": cov["azimuth_deg"],
        "travel_mm": round(_travel(order) * 1000.0, 1),
        "centre_m": region["centre_m"],
        "part_size_mm": region["size_mm"],
        "explain": (
            f"{len(order)} views at {standoff * 1000.0:.0f} mm standoff, tilted "
            f"{tilt:.0f} deg from vertical in {rings} ring"
            f"{'s' if rings > 1 else ''}. Standoff {standoff_source}; tilt "
            f"{tilt_source}; count {count_source}."),
    }


def _look_at(pos, target, world_up=(0.0, 0.0, 1.0)):
    """
    A camera at `pos` looking at `target`, as a 4x4 in the base frame.

    Camera convention is X right, Y down, Z forward. The frame is built and
    then ASSERTED right-handed: a left-handed frame here is not an error any
    single view shows — it silently mirrors the cloud about the optical axis,
    and the mirroring only becomes visible once two views have to agree.
    """
    pos = np.asarray(pos, dtype=float)
    target = np.asarray(target, dtype=float)
    fwd = target - pos
    n = np.linalg.norm(fwd)
    if n < 1e-9:
        raise ValueError("camera position coincides with the target")
    fwd = fwd / n
    up = np.asarray(world_up, dtype=float)
    right = np.cross(fwd, up)
    if np.linalg.norm(right) < 1e-6:                # looking straight down
        right = np.array([1.0, 0.0, 0.0])
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd], axis=1)
    assert np.linalg.det(R) > 0, "camera frame must be right-handed"
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def _reject_reason(p, reach, envelope) -> str:
    r = float(math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2))
    if r > reach["max_radius_m"]:
        return f"out of reach ({r * 1000:.0f} mm from the base)"
    if math.sqrt(p[0] ** 2 + p[1] ** 2) < reach["min_radius_m"]:
        return "too close to the robot's own column"
    if p[2] < reach["min_z_m"]:
        return f"below the table ({p[2] * 1000:.0f} mm)"
    if envelope:
        for i, ax in enumerate("xyz"):
            lim = envelope.get(ax)
            if lim and not (float(lim[0]) <= p[i] <= float(lim[1])):
                return f"outside the cell envelope on {ax.upper()}"
    return ""


def _order_poses(poses, start_pose=None):
    """
    Nearest-neighbour ordering from the current tool position.

    Not optimal and not trying to be — on 8 to 16 viewpoints the greedy tour
    is within a few percent of optimal, and the arm spends far more time
    settling at each view than travelling between them.
    """
    remaining = list(poses)
    here = (np.asarray(start_pose[:3], dtype=float)
            if start_pose is not None and len(start_pose) >= 3
            else np.asarray(remaining[0]["tcp_pose"][:3], dtype=float))
    out = []
    while remaining:
        d = [float(np.linalg.norm(np.asarray(p["tcp_pose"][:3]) - here))
             for p in remaining]
        k = int(np.argmin(d))
        out.append(remaining.pop(k))
        here = np.asarray(out[-1]["tcp_pose"][:3], dtype=float)
    return out


def _travel(poses) -> float:
    if len(poses) < 2:
        return 0.0
    pts = np.asarray([p["tcp_pose"][:3] for p in poses], dtype=float)
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def _coverage(poses) -> dict:
    az = sorted(p["azimuth_deg"] % 360.0 for p in poses)
    if len(az) < 2:
        return {"azimuth_deg": 0.0}
    gaps = [b - a for a, b in zip(az, az[1:])] + [360.0 - az[-1] + az[0]]
    return {"azimuth_deg": round(360.0 - max(gaps), 1)}


# ---------------------------------------------------------------------------
# 3-4. fusion and the model
# ---------------------------------------------------------------------------

class MultiViewSession:
    """
    Captures, fuses, cleans, and plans — one object so the intermediate arrays
    stay in the process and never cross a websocket.

    Each capture is GATED on depth fill inside the region before it is merged.
    A view taken mid-motion, or of a face the stereo could not match, adds
    noise at full weight to the voxel grid, and a voxel grid cannot tell a bad
    view from a good one afterwards. Rejecting it at capture is the only point
    where the information to reject it still exists.
    """

    def __init__(self, intr, T_tcp_cam, region: dict, voxel_mm: float = 1.5,
                 margin_mm: float = 25.0):
        ok, why = available()
        if not ok:
            raise RuntimeError(why)
        self.intr = intr
        self.T_tcp_cam = np.asarray(T_tcp_cam, dtype=float).reshape(4, 4)
        self.region = region
        self.voxel_m = voxel_mm / 1000.0
        self.margin = margin_mm / 1000.0
        self.cloud = scan3d.VoxelCloud(self.voxel_m)
        self.views: list[dict] = []
        self.surface = None
        self.component = None
        self.last_plan: dict | None = None
        self.started = time.time()

        lo = np.asarray(region["aabb_min_m"], dtype=float) - self.margin
        hi = np.asarray(region["aabb_max_m"], dtype=float) + self.margin
        self.crop = (lo, hi)

    # -- capture -----------------------------------------------------------
    def add_view(self, depth_raw, tcp_pose, *, stride: int = 2,
                 min_points: int = 300, label: str = "") -> dict:
        if depth_raw is None:
            return {"ok": False, "error": "no depth frame available right now"}
        if not tcp_pose or len(tcp_pose) < 6:
            return {"ok": False, "error":
                    "no TCP pose — a view without its pose cannot be placed "
                    "in the base frame, so it cannot be fused"}
        cam = scan3d.deproject(depth_raw, self.intr, stride=stride)
        if cam.shape[0] == 0:
            return {"ok": False, "error": "this view returned no depth at all"}
        base = scan3d.transform_to_base(cam, tcp_pose, self.T_tcp_cam)
        lo, hi = self.crop
        keep = np.all((base >= lo) & (base <= hi), axis=1)
        inside = base[keep]
        n_in = int(inside.shape[0])
        if n_in < min_points:
            return {"ok": False, "n_points": n_in, "n_raw": int(base.shape[0]),
                    "error": (f"only {n_in} of {base.shape[0]} points landed on "
                              "the part. Either the arm is not pointing at it, "
                              "or the surface returned no depth from this "
                              "angle. Not merged — a view like this adds noise "
                              "and nothing else.")}
        added = self.cloud.add(inside)
        rec = {"view": len(self.views), "label": label,
               "points_used": n_in, "points_raw": int(base.shape[0]),
               "new_voxels": int(added),
               "tcp_pose": [round(float(v), 5) for v in tcp_pose[:6]],
               "t": round(time.time() - self.started, 2)}
        self.views.append(rec)
        return {"ok": True, **rec, **self.cloud.stats()}

    # -- model -------------------------------------------------------------
    def build(self, min_hits: int = 2, plane_tol_mm: float = 4.0,
              link_mm: float = 8.0, keep_plane: bool = False) -> dict:
        """
        Fuse -> drop the fixture plane -> keep the biggest connected body.

        `min_hits` is the quality dial that matters: a voxel seen from only one
        viewpoint is a voxel nothing has confirmed. Requiring two is what turns
        a pile of overlapping scans into a measurement, and it is also why the
        view planning above insists on real overlap.
        """
        if len(self.views) < 1:
            return {"ok": False, "error": "no views captured yet"}
        raw = self.cloud.points(min_hits=min_hits)
        if raw.shape[0] < 50:
            return {"ok": False, "stats": self.cloud.stats(),
                    "error": (f"only {raw.shape[0]} voxels were seen at least "
                              f"{min_hits} times. Capture more views, or set "
                              "the confirmation count to 1 — but a model built "
                              "from single-view voxels is not a measurement.")}
        if keep_plane:
            body, plane = raw, None
        else:
            body, plane = scan3d.remove_dominant_plane(raw, tol=plane_tol_mm / 1000.0)
        blob, clus = scan3d.largest_cluster(body, link_m=link_mm / 1000.0)
        if blob.shape[0] < 30:
            return {"ok": False, "error":
                    "nothing survived cleaning. If the part is thin, lower the "
                    "fixture-plane tolerance; it is being removed with the "
                    "fixture."}
        self.surface = blob
        self.component = scan3d.localise(blob)
        comp = self.component.as_dict()
        return {"ok": True,
                "stats": self.cloud.stats(),
                "views": len(self.views),
                "plane": plane,
                "clustering": clus,
                "component": comp,
                "points": int(blob.shape[0]),
                "size_mm": [round(float(v), 1) for v in comp.get("extents_mm", [])] or None,
                "top_z_mm": round(float(comp.get("top_z", 0.0)) * 1000.0, 1),
                "yaw_deg": round(float(comp.get("yaw_deg", 0.0)), 1)}

    # -- for the browser ---------------------------------------------------
    def preview(self, max_points: int = 12000, source: str = "surface") -> dict:
        """
        A decimated cloud the console can draw.

        Decimated deliberately and by a REGULAR stride rather than a random
        sample: a random sample of a voxel grid looks like noise on screen,
        while a stride keeps the surface's structure legible at a tenth of the
        points. The full cloud stays here; the browser never needs it.
        """
        pts = self.surface if (source == "surface" and self.surface is not None) \
            else self.cloud.points(min_hits=1)
        if pts is None or len(pts) == 0:
            return {"ok": False, "error": "no points yet"}
        pts = np.asarray(pts, dtype=float)
        step = max(1, int(math.ceil(len(pts) / float(max_points))))
        sub = pts[::step]
        centre = sub.mean(axis=0)
        z = sub[:, 2]
        zlo, zhi = float(z.min()), float(z.max())
        return {"ok": True,
                "n": int(len(sub)),
                "n_full": int(len(pts)),
                "stride": step,
                "centre_m": [round(float(v), 5) for v in centre],
                "z_range_m": [round(zlo, 5), round(zhi, 5)],
                "points": [[round(float(p[0]), 4), round(float(p[1]), 4),
                            round(float(p[2]), 4)] for p in sub]}

    def plan_path(self, **kw) -> dict:
        if self.surface is None:
            return {"ok": False, "error": "build the model first"}
        self.last_plan = scan3d.plan_surface_path(self.surface, **kw)
        return self.last_plan

    def export_ply(self, path: str | Path) -> dict:
        """
        Write the fused cloud as an ASCII PLY — the format every 3D tool and
        every Isaac importer reads without a plugin.
        """
        pts = self.surface if self.surface is not None else self.cloud.points(1)
        if pts is None or len(pts) == 0:
            return {"ok": False, "error": "nothing to export"}
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="ascii") as fh:
            fh.write("ply\nformat ascii 1.0\n")
            fh.write(f"comment SONAIR multi-view fusion, {len(self.views)} views\n")
            fh.write(f"element vertex {len(pts)}\n")
            fh.write("property float x\nproperty float y\nproperty float z\n")
            fh.write("end_header\n")
            for q in np.asarray(pts, dtype=float):
                fh.write(f"{q[0]:.5f} {q[1]:.5f} {q[2]:.5f}\n")
        return {"ok": True, "path": str(p.resolve()), "n_points": int(len(pts))}

    def status(self) -> dict:
        return {"views": len(self.views),
                "captures": self.views[-8:],
                "cloud": self.cloud.stats(),
                "built": self.surface is not None,
                "planned": bool(self.last_plan and self.last_plan.get("ok")),
                "voxel_mm": round(self.voxel_m * 1000.0, 2),
                "region": {k: self.region.get(k)
                           for k in ("centre_m", "size_mm", "distance_mm")}}
