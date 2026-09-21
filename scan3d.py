"""
scan3d.py — eye-in-hand 3D reconstruction and automatic scan-path planning.

This is the pipeline asked for: carry the camera on the arm, sweep it over the
work area, reconstruct the component, locate it, then plan an inspection path
over its actual surface.

    plan_survey_poses      where to look from
    (robot executes, host captures depth + TCP pose at each stop)
    deproject              depth image -> camera-frame points
    transform_to_base      camera frame -> robot base frame, via TCP and hand-eye
    VoxelCloud             accumulate and downsample the fused cloud
    remove_dominant_plane  drop the table, keep what sits on it
    largest_cluster        isolate the component
    ComponentPose          oriented bounding box and principal axes
    plan_surface_path      raster over the real surface, at a standoff

What this is NOT: it is not SLAM, and it is not a CAD-model registration. It
builds a height field of what the camera can see from above and plans over
that. For a part with undercuts or a full 360° inspection you need multi-side
capture and a genuine surface reconstruction, and the honest answer is that
this pipeline would mislead you there rather than fail visibly.

Accuracy is bounded by the hand-eye calibration, not by the depth sensor. A
D435i at 300 mm has roughly 1-2 mm depth noise, which averages down over many
views; a 2° hand-eye rotation error does not average down at all and puts the
whole cloud 10 mm out at 300 mm standoff. Calibrate before trusting any of it.

numpy only. No Open3D, no PCL — this has to run on the same machine as the
bridge without a second environment.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:      # pragma: no cover
    _HAS_NUMPY = False
    np = None


# =============================================================================
# rigid transforms
# =============================================================================

def rotvec_to_matrix(rv) -> "np.ndarray":
    """UR reports orientation as a rotation vector (axis * angle)."""
    rv = np.asarray(rv, dtype=float)[:3]
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.eye(3)
    k = rv / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def matrix_to_rotvec(R) -> "np.ndarray":
    R = np.asarray(R, dtype=float)
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if abs(theta - math.pi) < 1e-6:
        # Near 180 deg the usual formula loses all precision; take the axis
        # from the largest diagonal of (R + I) instead.
        A = (R + np.eye(3)) / 2.0
        k = np.sqrt(np.maximum(np.diag(A), 0.0))
        i = int(np.argmax(k))
        if k[i] > 1e-9:
            k = A[:, i] / k[i]
        return k / np.linalg.norm(k) * theta
    s = math.sin(theta)
    return theta / (2 * s) * np.array([R[2, 1] - R[1, 2],
                                       R[0, 2] - R[2, 0],
                                       R[1, 0] - R[0, 1]])


def pose_to_matrix(pose) -> "np.ndarray":
    """UR pose [x,y,z,rx,ry,rz] -> 4x4 homogeneous."""
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(pose[3:6])
    T[:3, 3] = np.asarray(pose[:3], dtype=float)
    return T


def matrix_to_pose(T) -> list:
    return list(np.asarray(T)[:3, 3]) + list(matrix_to_rotvec(np.asarray(T)[:3, :3]))


# =============================================================================
# camera model
# =============================================================================

@dataclass
class CameraIntrinsics:
    """
    Read these from the camera, never from a datasheet.

    pyrealsense2:
        p = pipeline.get_active_profile()
        s = p.get_stream(rs.stream.depth).as_video_stream_profile()
        i = s.get_intrinsics()   ->  i.fx, i.fy, i.ppx, i.ppy, i.width, i.height
    """
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float = 0.001   # depth units -> metres

    @classmethod
    def from_realsense(cls, intr, depth_scale: float = 0.001) -> "CameraIntrinsics":
        return cls(fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy,
                   width=intr.width, height=intr.height, depth_scale=depth_scale)


def deproject(depth_image, intr: CameraIntrinsics,
              z_min: float = 0.10, z_max: float = 1.2,
              stride: int = 2) -> "np.ndarray":
    """
    Depth image -> Nx3 points in the CAMERA frame (metres, +Z forward).

    `stride` subsamples. A 640x480 frame is 307k points; at stride 2 it is 77k,
    which is plenty for a 1 mm voxel grid and four times faster to transform.
    Points outside [z_min, z_max] are dropped: zero means "no return" on a
    stereo camera, and far returns are almost always the far wall.
    """
    d = np.asarray(depth_image)
    if d.ndim != 2:
        raise ValueError("depth_image must be a 2-D array of raw depth units")
    d = d[::stride, ::stride]
    h, w = d.shape
    ys, xs = np.mgrid[0:h, 0:w]
    xs = xs * stride
    ys = ys * stride

    z = d.astype(np.float64) * intr.depth_scale
    valid = (z > z_min) & (z < z_max)
    if not np.any(valid):
        return np.zeros((0, 3))
    z = z[valid]
    x = (xs[valid] - intr.cx) / intr.fx * z
    y = (ys[valid] - intr.cy) / intr.fy * z
    return np.stack([x, y, z], axis=1)


def transform_to_base(points_cam, tcp_pose, T_tcp_cam) -> "np.ndarray":
    """
    Camera-frame points -> robot base frame.

        T_base_cam = T_base_tcp . T_tcp_cam

    `T_tcp_cam` is the hand-eye calibration: where the camera sits relative to
    the tool flange. It is the single largest error source in this pipeline —
    everything downstream inherits it, and unlike depth noise it does not
    average out over views.
    """
    if points_cam.shape[0] == 0:
        return points_cam
    T = pose_to_matrix(tcp_pose) @ np.asarray(T_tcp_cam, dtype=float)
    return (T[:3, :3] @ points_cam.T).T + T[:3, 3]


# =============================================================================
# fusion
# =============================================================================

class VoxelCloud:
    """
    Accumulating voxel grid.

    Points are binned to a grid and only the running mean per occupied voxel is
    kept, so memory is bounded by the working volume rather than by how many
    views you take. Averaging inside a voxel is also what makes the depth noise
    average down across views — the reason to take more views at all.
    """

    def __init__(self, voxel_m: float = 0.002):
        self.voxel = float(voxel_m)
        self._sum: dict[tuple, "np.ndarray"] = {}
        self._n: dict[tuple, int] = {}
        self.views = 0

    def add(self, points_base) -> int:
        pts = np.asarray(points_base, dtype=float)
        if pts.shape[0] == 0:
            return 0
        keys = np.floor(pts / self.voxel).astype(np.int64)
        for k, p in zip(map(tuple, keys), pts):
            if k in self._sum:
                self._sum[k] += p
                self._n[k] += 1
            else:
                self._sum[k] = p.copy()
                self._n[k] = 1
        self.views += 1
        return pts.shape[0]

    def points(self, min_hits: int = 1) -> "np.ndarray":
        """
        Occupied voxel centroids.

        `min_hits` is the noise filter that matters: a voxel seen once may be a
        stereo mismatch, a voxel seen from three different poses is real
        geometry. Raise it as you add views.
        """
        out = [self._sum[k] / self._n[k] for k in self._sum if self._n[k] >= min_hits]
        return np.array(out) if out else np.zeros((0, 3))

    def stats(self) -> dict:
        hits = list(self._n.values())
        return {"voxels": len(self._sum), "views": self.views,
                "voxel_mm": self.voxel * 1000.0,
                "mean_hits": (sum(hits) / len(hits)) if hits else 0.0,
                "max_hits": max(hits) if hits else 0}


# =============================================================================
# segmentation
# =============================================================================

def remove_dominant_plane(points, tol: float = 0.004, iters: int = 200,
                          seed: int = 0) -> tuple["np.ndarray", dict]:
    """
    RANSAC out the largest plane — in this cell, the table or fixture plate.

    Returns (points_above_plane, plane_info). Only points on the POSITIVE side
    of the plane are kept, not simply the off-plane points: anything below the
    table is a reflection or a stereo artefact, and keeping it would give the
    clustering step a second blob to choose from.
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 50:
        return pts, {"found": False, "reason": "too few points to fit a plane"}

    rng = np.random.default_rng(seed)
    best_n, best_d, best_count = None, 0.0, 0
    for _ in range(iters):
        idx = rng.choice(pts.shape[0], 3, replace=False)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        d = -float(n @ p0)
        inliers = int(np.sum(np.abs(pts @ n + d) < tol))
        if inliers > best_count:
            best_n, best_d, best_count = n, d, inliers

    if best_n is None or best_count < pts.shape[0] * 0.15:
        return pts, {"found": False, "reason": "no dominant plane", "inliers": best_count}

    if best_n[2] < 0:            # point the normal up, so "above" is unambiguous
        best_n, best_d = -best_n, -best_d
    signed = pts @ best_n + best_d
    above = pts[signed > tol]
    return above, {
        "found": True,
        "normal": best_n.tolist(),
        "offset": best_d,
        "inliers": best_count,
        "removed": int(pts.shape[0] - above.shape[0]),
        "tilt_deg": math.degrees(math.acos(max(-1, min(1, float(best_n[2]))))),
    }


def largest_cluster(points, link_m: float = 0.008) -> tuple["np.ndarray", dict]:
    """
    Keep the biggest connected blob, on a voxel-grid flood fill.

    A grid flood fill rather than a KD-tree region grow: at these point counts
    it is comparable in speed, it needs no extra dependency, and its linkage
    distance is exactly the grid pitch, which is easier to reason about when a
    part comes out split in two.
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] == 0:
        return pts, {"clusters": 0}

    keys = np.floor(pts / link_m).astype(np.int64)
    occupied: dict[tuple, list[int]] = {}
    for i, k in enumerate(map(tuple, keys)):
        occupied.setdefault(k, []).append(i)

    seen: set[tuple] = set()
    clusters: list[list[int]] = []
    neighbours = [(dx, dy, dz)
                  for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                  if (dx, dy, dz) != (0, 0, 0)]
    for start in occupied:
        if start in seen:
            continue
        stack, members = [start], []
        seen.add(start)
        while stack:
            cur = stack.pop()
            members.extend(occupied[cur])
            for d in neighbours:
                nb = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
                if nb in occupied and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        clusters.append(members)

    if not clusters:
        return pts, {"clusters": 0}
    clusters.sort(key=len, reverse=True)
    biggest = pts[clusters[0]]
    return biggest, {
        "clusters": len(clusters),
        "kept": int(biggest.shape[0]),
        "discarded": int(pts.shape[0] - biggest.shape[0]),
        "sizes": [len(c) for c in clusters[:5]],
    }


# =============================================================================
# localisation
# =============================================================================

@dataclass
class ComponentPose:
    """Where the component is, and how it is oriented, in the robot base frame."""
    centre: list = field(default_factory=list)
    axes: list = field(default_factory=list)      # 3x3, columns are the axes
    extents: list = field(default_factory=list)   # full size along each axis, m
    n_points: int = 0
    aabb_min: list = field(default_factory=list)
    aabb_max: list = field(default_factory=list)
    top_z: float = 0.0

    def as_dict(self) -> dict:
        d = asdict(self)
        d["extents_mm"] = [e * 1000.0 for e in self.extents]
        d["yaw_deg"] = (math.degrees(math.atan2(self.axes[1][0], self.axes[0][0]))
                        if self.axes else 0.0)
        return d


def localise(points) -> ComponentPose:
    """
    Oriented bounding box by PCA.

    The third axis is forced to Z-up rather than taken from PCA: for a flat-ish
    part the two in-plane eigenvalues are close, PCA's third axis flips sign
    between runs, and a scan path built on a flipped axis runs backwards. Only
    the in-plane rotation is estimated, which is the part that is actually
    well conditioned.
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 3:
        return ComponentPose()

    centre = pts.mean(axis=0)
    xy = pts[:, :2] - centre[:2]
    cov = np.cov(xy.T)
    evals, evecs = np.linalg.eigh(cov)
    order = np.argsort(evals)[::-1]
    e0 = np.array([evecs[0, order[0]], evecs[1, order[0]], 0.0])
    e0 /= np.linalg.norm(e0)
    e1 = np.array([-e0[1], e0[0], 0.0])
    e2 = np.array([0.0, 0.0, 1.0])
    axes = np.stack([e0, e1, e2], axis=1)

    local = (pts - centre) @ axes
    lo, hi = local.min(axis=0), local.max(axis=0)
    return ComponentPose(
        centre=(centre + axes @ ((lo + hi) / 2.0)).tolist(),
        axes=axes.tolist(),
        extents=(hi - lo).tolist(),
        n_points=int(pts.shape[0]),
        aabb_min=pts.min(axis=0).tolist(),
        aabb_max=pts.max(axis=0).tolist(),
        top_z=float(pts[:, 2].max()),
    )


# =============================================================================
# path planning
# =============================================================================

def plan_survey_poses(centre, radius: float = 0.35, height: float = 0.45,
                      n_views: int = 8) -> list[dict]:
    """
    Where to put the camera for the reconstruction sweep.

    A ring of poses around the work centre, each looking inward and down.
    Multiple viewing angles are the point: a single top-down sweep cannot see
    vertical faces at all, and the voxel averaging only reduces noise across
    genuinely different viewpoints.

    These are CAMERA poses. The arm has to be commanded in TCP poses, so the
    caller converts with T_tcp_cam before sending anything to the robot —
    `survey_to_tcp` below does that.
    """
    centre = np.asarray(centre, dtype=float)
    out = []
    for i in range(n_views):
        a = 2 * math.pi * i / n_views
        pos = centre + np.array([radius * math.cos(a), radius * math.sin(a), height])
        forward = centre - pos
        forward /= np.linalg.norm(forward)
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:        # looking straight down
            right = np.array([1.0, 0.0, 0.0])
        right /= np.linalg.norm(right)
        # Camera convention: X right, Y down, Z forward, RIGHT-handed, so
        # down = forward x right. Writing it the other way round gives a
        # reflection (det = -1), which survives as a valid-looking matrix and
        # only shows up later as a cloud mirrored about the optical axis.
        down = np.cross(forward, right)
        R = np.stack([right, down, forward], axis=1)
        assert np.linalg.det(R) > 0, "survey pose frame must be right-handed"
        out.append({
            "index": i,
            "camera_pose": matrix_to_pose(
                np.block([[R, pos.reshape(3, 1)], [np.zeros((1, 3)), 1.0]])),
            "azimuth_deg": math.degrees(a),
        })
    return out


def survey_to_tcp(camera_pose, T_tcp_cam) -> list:
    """T_base_tcp = T_base_cam . inverse(T_tcp_cam)."""
    T_cam = pose_to_matrix(camera_pose)
    T_tc = np.asarray(T_tcp_cam, dtype=float)
    return matrix_to_pose(T_cam @ np.linalg.inv(T_tc))


def height_field(points, pitch: float = 0.002) -> tuple[dict, dict]:
    """
    Top-down height map: for each XY cell, the highest Z seen.

    This is what the scan path follows. It is an explicit admission of the
    pipeline's limit — a height field cannot represent an undercut, so a path
    planned on it is only valid for a part inspected from above.
    """
    pts = np.asarray(points, dtype=float)
    grid: dict[tuple, float] = {}
    for p in pts:
        k = (int(math.floor(p[0] / pitch)), int(math.floor(p[1] / pitch)))
        if k not in grid or p[2] > grid[k]:
            grid[k] = float(p[2])
    return grid, {"cells": len(grid), "pitch_mm": pitch * 1000.0}


def plan_surface_path(points, standoff: float = 0.10, line_spacing: float = 0.005,
                      step_along: float = 0.005, margin: float = 0.005,
                      pitch: float = 0.002, tool_rotvec=(0.0, math.pi, 0.0),
                      serpentine: bool = True) -> dict:
    """
    Raster over the reconstructed surface at a constant standoff.

    The path follows the measured height field, so a stepped or curved top
    surface keeps the sensor at the same distance instead of the fixed Z a
    four-corner planar path would give you. Cells with no measurement are
    skipped rather than interpolated: moving a probe to a standoff computed
    from a guessed height is how you crash into the part.

    Returns waypoints in the same shape the console's Zig-Zag planner uses, so
    the existing preview, 3D overlay and execute path all work unchanged.
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 10:
        return {"ok": False, "error": "not enough surface points to plan a path",
                "waypoints": []}

    grid, ginfo = height_field(pts, pitch)
    xs = [k[0] for k in grid]
    ys = [k[1] for k in grid]
    x0, x1 = min(xs) * pitch, (max(xs) + 1) * pitch
    y0, y1 = min(ys) * pitch, (max(ys) + 1) * pitch
    x0 += margin; x1 -= margin
    y0 += margin; y1 -= margin
    if x1 <= x0 or y1 <= y0:
        return {"ok": False, "error": "margin larger than the component footprint",
                "waypoints": []}

    def height_at(x, y):
        """Highest Z within one cell, widening once before giving up."""
        k = (int(math.floor(x / pitch)), int(math.floor(y / pitch)))
        if k in grid:
            return grid[k]
        best = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                z = grid.get((k[0] + dx, k[1] + dy))
                if z is not None and (best is None or z > best):
                    best = z
        return best

    rx, ry, rz = tool_rotvec
    waypoints = []
    n_lines = max(1, int((y1 - y0) / line_spacing))
    skipped = 0
    for i in range(n_lines + 1):
        y = y0 + i * line_spacing
        if y > y1:
            break
        n_steps = max(1, int((x1 - x0) / step_along))
        xs_line = [x0 + j * step_along for j in range(n_steps + 1)]
        if serpentine and i % 2 == 1:
            xs_line.reverse()
        for j, x in enumerate(xs_line):
            z = height_at(x, y)
            if z is None:
                skipped += 1
                continue
            waypoints.append({
                "type": f"SCAN_L{i}_{'END' if j == len(xs_line) - 1 else 'PT'}",
                "coords": [float(x), float(y), float(z + standoff),
                           float(rx), float(ry), float(rz)],
                "surface_z": float(z),
            })

    if not waypoints:
        return {"ok": False, "error": "every raster point fell on unmeasured surface",
                "waypoints": []}

    zs = [w["coords"][2] for w in waypoints]
    return {
        "ok": True,
        "waypoints": waypoints,
        "n_waypoints": len(waypoints),
        "n_lines": n_lines + 1,
        "skipped_unmeasured": skipped,
        "coverage": len(waypoints) / max(1, len(waypoints) + skipped),
        "z_range_mm": [min(zs) * 1000.0, max(zs) * 1000.0],
        "bounds_m": {"x": [x0, x1], "y": [y0, y1]},
        "standoff_mm": standoff * 1000.0,
        "height_field": ginfo,
        "note": ("Path follows the measured height field at a constant standoff. "
                 "Valid for inspection from above only; a height field cannot "
                 "represent undercuts. Unmeasured cells are skipped, never "
                 "interpolated."),
    }


# =============================================================================
# the session object the bridge drives
# =============================================================================

class ReconstructionSession:
    """
    Holds one reconstruction from start to planned path.

    Deliberately stateful and step-by-step rather than one `scan()` call: each
    stage has a failure mode that needs looking at before the next one runs,
    and a single call would hide which stage went wrong.
    """

    def __init__(self, intr: CameraIntrinsics, T_tcp_cam, voxel_m: float = 0.002):
        if not _HAS_NUMPY:
            raise RuntimeError("scan3d needs numpy")
        self.intr = intr
        self.T_tcp_cam = np.asarray(T_tcp_cam, dtype=float)
        self.cloud = VoxelCloud(voxel_m)
        self.captures: list[dict] = []
        self.component: ComponentPose | None = None
        self.surface = None
        self.last_plan: dict | None = None

    def add_view(self, depth_image, tcp_pose, stride: int = 2) -> dict:
        cam = deproject(depth_image, self.intr, stride=stride)
        base = transform_to_base(cam, tcp_pose, self.T_tcp_cam)
        n = self.cloud.add(base)
        rec = {"view": len(self.captures), "points": int(n),
               "tcp_pose": [float(v) for v in tcp_pose]}
        self.captures.append(rec)
        return {**rec, **self.cloud.stats()}

    def reconstruct(self, min_hits: int = 2, plane_tol: float = 0.004,
                    link_m: float = 0.008) -> dict:
        raw = self.cloud.points(min_hits=min_hits)
        if raw.shape[0] < 50:
            return {"ok": False,
                    "error": (f"only {raw.shape[0]} voxels seen at least {min_hits} "
                              f"times — take more views, or lower min_hits"),
                    "stats": self.cloud.stats()}
        above, plane = remove_dominant_plane(raw, tol=plane_tol)
        blob, clus = largest_cluster(above, link_m=link_m)
        self.surface = blob
        self.component = localise(blob)
        return {
            "ok": True,
            "stats": self.cloud.stats(),
            "plane": plane,
            "clustering": clus,
            "component": self.component.as_dict(),
        }

    def plan(self, **kw) -> dict:
        if self.surface is None:
            return {"ok": False, "error": "run reconstruct() first"}
        self.last_plan = plan_surface_path(self.surface, **kw)
        return self.last_plan

    def export(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps({
            "intrinsics": asdict(self.intr),
            "T_tcp_cam": self.T_tcp_cam.tolist(),
            "captures": self.captures,
            "cloud": self.cloud.stats(),
            "component": self.component.as_dict() if self.component else None,
            "plan": self.last_plan,
        }, indent=2), encoding="utf-8")
        return p
