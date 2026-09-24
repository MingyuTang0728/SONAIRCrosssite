"""
handeye.py — where the camera actually sits on the tool.

Everything downstream inherits this one transform. A point deprojected from a
depth pixel is placed in the robot's base frame by

    p_base = T_base_tcp @ T_tcp_cam @ p_cam

so an error in T_tcp_cam is not a small offset on the result — it is a rigid
error that grows with standoff and rotates with the tool. At 300 mm standoff a
2 degree rotation error puts the point 10 mm out, and it puts it out in a
DIFFERENT DIRECTION at every viewpoint, which is exactly what stops a
multi-view reconstruction from fusing: the same physical face arrives as three
faces 10 mm apart and the voxel grid faithfully records all three.

This module does not guess it. It measures it, and then it says how well.

The measurement is the classic AX = XB problem: hold a fixed target, move the
arm to a set of poses, and solve for the constant tool-to-camera transform
that makes the target's observed motion consistent with the arm's known
motion. OpenCV solves it five different ways; all five are run and compared,
because agreement between independent solvers is evidence and a single number
from a single solver is not.

THE RESIDUAL THAT MATTERS is not the solver's own. It is this: the target has
not moved, so for every sample

    T_base_target = T_base_tcp @ T_tcp_cam @ T_cam_target

must come out the SAME. The spread of those reconstructed target poses across
all samples is reported in millimetres and degrees, and it is the number to
quote — it is measured in the frame the work happens in, and it cannot be made
small by a solver that has fitted its own noise.

Import-tolerant: with no OpenCV, detection and solving report themselves
unavailable and the rest of the console keeps running.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    import numpy as np
    _HAS_NP = True
except Exception as e:                              # noqa: BLE001
    np = None
    _HAS_NP = False
    _NP_ERR = str(e)

try:
    import cv2
    _HAS_CV = True
    _CV_ERR = ""
except Exception as e:                              # noqa: BLE001
    cv2 = None
    _HAS_CV = False
    _CV_ERR = str(e)


# ---------------------------------------------------------------------------
# small rigid-transform helpers (kept local so this module stands alone)
# ---------------------------------------------------------------------------

def rotvec_to_matrix(rv):
    rv = np.asarray(rv, dtype=float).reshape(3)
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.eye(3)
    k = rv / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def matrix_to_rotvec(R):
    R = np.asarray(R, dtype=float)
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if abs(math.pi - theta) < 1e-6:
        # Near 180 deg the antisymmetric part vanishes; recover the axis from
        # the largest diagonal term of R + I instead, which stays conditioned.
        A = (R + np.eye(3)) / 2.0
        k = np.sqrt(np.clip(np.diag(A), 0.0, None))
        i = int(np.argmax(k))
        if k[i] > 1e-9:
            k = A[:, i] / k[i]
        return k / (np.linalg.norm(k) or 1.0) * theta
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(theta)) * theta


def pose_to_matrix(pose):
    """UR pose [x,y,z,rx,ry,rz] (metres, axis-angle) -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(pose[3:6])
    T[:3, 3] = np.asarray(pose[:3], dtype=float)
    return T


def matrix_to_pose(T):
    T = np.asarray(T, dtype=float)
    return [*[float(v) for v in T[:3, 3]],
            *[float(v) for v in matrix_to_rotvec(T[:3, :3])]]


def invert(T):
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


# ---------------------------------------------------------------------------
# targets
# ---------------------------------------------------------------------------

@dataclass
class TargetSpec:
    """
    The calibration board. `cols`/`rows` are INNER CORNER counts for a
    chessboard — 9x6 for the usual 10x7-square board — because that is what
    OpenCV wants and because counting squares is the single most common way to
    get a calibration silently wrong by one square pitch.
    """
    kind: str = "chessboard"          # "chessboard" | "charuco" | "circles"
    cols: int = 9
    rows: int = 6
    square_mm: float = 25.0
    marker_mm: float = 18.0           # charuco only
    dictionary: str = "DICT_4X4_50"   # charuco only

    def object_points(self):
        """Board corners in the board's own frame, metres, Z=0."""
        s = self.square_mm / 1000.0
        pts = np.zeros((self.rows * self.cols, 3), dtype=np.float32)
        grid = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        pts[:, :2] = grid * s
        return pts

    def as_dict(self):
        return dict(self.__dict__)


def available() -> tuple[bool, str]:
    if not _HAS_NP:
        return False, f"numpy not importable ({_NP_ERR})"
    if not _HAS_CV:
        return False, (f"OpenCV not importable ({_CV_ERR}) — "
                       "run: pip install opencv-python")
    return True, ""


def detect_target(image, spec: TargetSpec, intrinsics=None) -> dict:
    """
    Find the board in one image and, if intrinsics are given, its pose.

    Returns `ok`, the 2D corners for drawing, and `T_cam_target`. Sub-pixel
    refinement is not optional here: at 25 mm squares and 600 mm range, one
    pixel of corner error is roughly 0.1 deg of board rotation, and the
    hand-eye solve amplifies board rotation error directly into tool rotation
    error.
    """
    ok, why = available()
    if not ok:
        return {"ok": False, "error": why}
    if image is None:
        return {"ok": False, "error": "no camera image — start the colour stream"}

    img = np.asarray(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

    if spec.kind == "charuco":
        return _detect_charuco(gray, spec, intrinsics)

    if spec.kind == "circles":
        found, corners = cv2.findCirclesGrid(
            gray, (spec.cols, spec.rows), cv2.CALIB_CB_ASYMMETRIC_GRID)
    else:
        found, corners = _find_chessboard(gray, spec.cols, spec.rows)

    if not found:
        return {"ok": False, **_why_not(gray, spec)}

    corners = _as_corner_array(corners)
    out = {"ok": True, "n_corners": int(len(corners)),
           "corners": _corner_list(corners)}
    if intrinsics is not None:
        K, dist = _K_from(intrinsics)
        obj = spec.object_points()
        ok_pnp, rvec, tvec = cv2.solvePnP(obj, corners, K, dist,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok_pnp:
            return {"ok": False, "error": "board found but its pose could not "
                                          "be solved — check the square size"}
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.reshape(3)
        out["T_cam_target"] = T.tolist()
        out["distance_mm"] = round(float(np.linalg.norm(tvec)) * 1000.0, 1)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
        out["reprojection_px"] = round(float(err.mean()), 3)
        out["reprojection_max_px"] = round(float(err.max()), 3)
    return out


def _detect_charuco(gray, spec: TargetSpec, intrinsics):
    try:
        adict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, spec.dictionary))
        board = cv2.aruco.CharucoBoard((spec.cols, spec.rows),
                                       spec.square_mm / 1000.0,
                                       spec.marker_mm / 1000.0, adict)
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": f"ChArUco unavailable in this OpenCV "
                                      f"build ({e}) — use a chessboard"}
    corners, ids, _ = cv2.aruco.detectMarkers(gray, adict)
    if ids is None or len(ids) < 4:
        return {"ok": False, "error": "fewer than 4 ArUco markers visible"}
    n, ch_c, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if n is None or n < 6:
        return {"ok": False, "error": f"only {n or 0} ChArUco corners — "
                                      "move closer or improve the lighting"}
    ch_c = _as_corner_array(ch_c)
    out = {"ok": True, "n_corners": int(n), "corners": _corner_list(ch_c)}
    if intrinsics is not None:
        K, dist = _K_from(intrinsics)
        ok_pose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_c, ch_ids, board, K, dist, None, None)
        if ok_pose:
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(rvec)[0]
            T[:3, 3] = tvec.reshape(3)
            out["T_cam_target"] = T.tolist()
            out["distance_mm"] = round(float(np.linalg.norm(tvec)) * 1000.0, 1)
    return out


def _find_chessboard(gray, cols, rows):
    """
    Find the corners, preferring the detector built for dense boards.

    findChessboardCornersSB handles what the classic one struggles with: many
    small squares, motion blur, uneven lighting and steep viewing angles — all
    of which describe a fine-pitch board held at arm's length by a robot. It
    also returns sub-pixel positions directly, so no separate refinement step
    can undo them. The classic detector stays as the fallback for builds that
    lack it.

    CALIB_CB_FAST_CHECK is deliberately absent: it is a cheap early reject
    that gives up on exactly the marginal images worth trying harder on, and
    here a detection costs milliseconds while a missed one costs a pose.
    """
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = 0
        for name in ("CALIB_CB_EXHAUSTIVE", "CALIB_CB_ACCURACY",
                     "CALIB_CB_NORMALIZE_IMAGE"):
            flags |= getattr(cv2, name, 0)
        try:
            ok, c = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
            if ok:
                return True, c
        except Exception:
            pass
    ok, c = cv2.findChessboardCorners(
        gray, (cols, rows),
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
        c = cv2.cornerSubPix(gray, _as_corner_array(c), (11, 11), (-1, -1), crit)
    return ok, c


def probe_board_size(gray, cols, rows, span=2):
    """
    Nothing found at the declared size — what IS on the board?

    Tries the sizes around it and the swapped orientation. Miscounting the
    inner corners is the commonest calibration mistake there is, and "no
    25x18 board found" gives the operator nothing to act on, while "this is a
    24x17 board" ends the problem. Bounded deliberately: this runs on a
    failure, not on every frame.
    """
    tried = []
    for dc in range(-span, span + 1):
        for dr in range(-span, span + 1):
            c, r = cols + dc, rows + dr
            if c < 3 or r < 3 or (c == cols and r == rows):
                continue
            tried.append((c, r))
    tried.append((rows, cols))          # the axes the other way round
    # nearest first: a miscount is usually off by one
    tried.sort(key=lambda t: abs(t[0] - cols) + abs(t[1] - rows))
    for c, r in tried[:14]:
        try:
            ok, _ = _find_chessboard(gray, c, r)
        except Exception:
            continue
        if ok:
            return (c, r)
    return None


def _sharpness(gray):
    """Variance of the Laplacian: the standard cheap focus measure."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _why_not(gray, spec) -> dict:
    """
    Say what is wrong with THIS image, not what is usually wrong.

    A detector that only ever reports "not found" makes the operator guess
    between the count, the focus, the lighting and the framing. Each of those
    is measurable, so each is measured.
    """
    out = {"probed": True}
    found = probe_board_size(gray, spec.cols, spec.rows)
    if found:
        out["suggested_size"] = list(found)
        out["error"] = (f"No {spec.cols}x{spec.rows} board here, but a "
                        f"{found[0]}x{found[1]} one WAS found. Change the "
                        f"inner-corner counts to {found[0]} across and "
                        f"{found[1]} down. Count inner corners, not squares.")
        return out

    sharp = _sharpness(gray)
    out["sharpness"] = round(sharp, 1)
    mean = float(gray.mean())
    out["brightness"] = round(mean, 1)
    reasons = []
    if sharp < 60:
        reasons.append("the image is blurred — hold the arm still, and give "
                       "the camera a moment to settle after it moves")
    if mean < 45:
        reasons.append("the picture is very dark — more light on the board, "
                       "or raise the colour exposure on the Camera page")
    if mean > 215:
        reasons.append("the picture is washed out — less light, or lower the "
                       "colour exposure")
    if not reasons:
        reasons.append(f"no {spec.cols}x{spec.rows} grid was found and no "
                       "nearby size matched either. Check the whole board is "
                       "in frame with a clear white margin all round, that it "
                       "is flat, and that the count is of INNER corners")
    out["error"] = ("Board not detected: " + "; ".join(reasons)
                    + f". (sharpness {sharp:.0f}, brightness {mean:.0f}/255)")
    return out


def _as_corner_array(corners):
    """
    Normalise a detector's corner output to (N, 1, 2) float32.

    OpenCV 4 returns (N, 1, 2) from findChessboardCorners; OpenCV 5 returns
    (N, 2). Both are correct and the difference is invisible until something
    indexes the middle axis, so it is flattened once here rather than guarded
    at every use.
    """
    a = np.asarray(corners, dtype=np.float32)
    return a.reshape(-1, 1, 2)


def _corner_list(corners):
    return [[float(p[0]), float(p[1])] for p in
            np.asarray(corners, dtype=float).reshape(-1, 2)]


def _K_from(intr):
    """Accept a scan3d.CameraIntrinsics, a dict, or a 3x3 matrix."""
    if isinstance(intr, dict):
        fx, fy = intr["fx"], intr["fy"]
        cx, cy = intr.get("cx", intr.get("ppx")), intr.get("cy", intr.get("ppy"))
        dist = np.asarray(intr.get("coeffs", [0, 0, 0, 0, 0]), dtype=float)
    elif hasattr(intr, "fx"):
        fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
        dist = np.asarray(getattr(intr, "coeffs", [0, 0, 0, 0, 0]), dtype=float)
    else:
        K = np.asarray(intr, dtype=float).reshape(3, 3)
        return K, np.zeros(5)
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float)
    if dist.size < 5:
        dist = np.zeros(5)
    return K, dist


# ---------------------------------------------------------------------------
# the session
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# AX = XB solvers
#
# Implemented here in numpy rather than called out to cv2.calibrateHandEye,
# for two reasons. It is not present in every OpenCV build — OpenCV 5 does not
# expose it — and a calibration that silently cannot run on the machine it is
# needed on is worse than no calibration. And three independent formulations
# that agree is evidence; one opaque call that returns a number is not.
#
# Where cv2.calibrateHandEye IS available its methods are run as well and
# folded into the same comparison, so nothing is lost on OpenCV 4.
# ---------------------------------------------------------------------------

def _skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _project_so3(M):
    """Nearest rotation matrix in the Frobenius sense."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def _motion_pairs(T_bg, T_ct, min_angle_deg: float = 5.0):
    """
    Turn absolute poses into the (A, B) motion pairs the solvers consume.

    A = T_bg_j^-1 T_bg_i  (how the tool moved)
    B = C_j C_i^-1        (how the board appeared to move in the camera)

    Pairs whose rotation is tiny are DROPPED. They carry almost no information
    about X and they dominate the least-squares by sheer count, which is how a
    set of poses that looks large ends up determining nothing.
    """
    pairs = []
    n = len(T_bg)
    for i in range(n):
        for j in range(i + 1, n):
            A = invert(T_bg[j]) @ T_bg[i]
            B = T_ct[j] @ invert(T_ct[i])
            ang = float(np.linalg.norm(matrix_to_rotvec(A[:3, :3])))
            if math.degrees(ang) < min_angle_deg:
                continue
            pairs.append((A, B, ang))
    return pairs


def _solve_translation(pairs, Rx):
    """(R_A - I) t_x = R_x t_B - t_A, stacked over every motion pair."""
    C, d = [], []
    for A, B, _ in pairs:
        C.append(A[:3, :3] - np.eye(3))
        d.append(Rx @ B[:3, 3] - A[:3, 3])
    C = np.vstack(C)
    d = np.concatenate(d)
    t, *_ = np.linalg.lstsq(C, d, rcond=None)
    return t


def solve_tsai(pairs):
    """Tsai & Lenz 1989 — rotation from modified Rodrigues vectors."""
    S, b = [], []
    for A, B, _ in pairs:
        ra = matrix_to_rotvec(A[:3, :3])
        rb = matrix_to_rotvec(B[:3, :3])
        ta, tb = np.linalg.norm(ra), np.linalg.norm(rb)
        if ta < 1e-9 or tb < 1e-9:
            continue
        Pa = 2.0 * math.sin(ta / 2.0) * (ra / ta)
        Pb = 2.0 * math.sin(tb / 2.0) * (rb / tb)
        S.append(_skew(Pa + Pb))
        b.append(Pb - Pa)
    if not S:
        raise ValueError("no usable motion pairs")
    x, *_ = np.linalg.lstsq(np.vstack(S), np.concatenate(b), rcond=None)
    n2 = float(x @ x)
    Px = 2.0 * x / math.sqrt(1.0 + n2)
    p2 = float(Px @ Px)
    Rx = ((1.0 - p2 / 2.0) * np.eye(3) +
          0.5 * (np.outer(Px, Px) + math.sqrt(max(0.0, 4.0 - p2)) * _skew(Px)))
    Rx = _project_so3(Rx)
    return Rx, _solve_translation(pairs, Rx)


def solve_park(pairs):
    """Park & Martin 1994 — rotation in the Lie algebra, closed form."""
    M = np.zeros((3, 3))
    for A, B, _ in pairs:
        a = matrix_to_rotvec(A[:3, :3])
        b = matrix_to_rotvec(B[:3, :3])
        M += np.outer(b, a)
    w, V = np.linalg.eigh(M.T @ M)
    w = np.clip(w, 1e-12, None)
    Rx = V @ np.diag(1.0 / np.sqrt(w)) @ V.T @ M.T
    Rx = _project_so3(Rx)
    return Rx, _solve_translation(pairs, Rx)


def solve_andreff(pairs):
    """
    Andreff 1999 — one linear system for rotation and translation together.

    Structurally different from the other two: it never forms a rotation
    vector, so it fails differently, which is what makes its agreement with
    them worth having.
    """
    rows, rhs = [], []
    for A, B, _ in pairs:
        Ra, Rb = A[:3, :3], B[:3, :3]
        ta, tb = A[:3, 3], B[:3, 3]
        top = np.hstack([np.eye(9) - np.kron(Ra, Rb), np.zeros((9, 3))])
        bot = np.hstack([np.kron(np.eye(3), tb.reshape(1, 3)), np.eye(3) - Ra])
        rows.append(top)
        rhs.append(np.zeros(9))
        rows.append(bot)
        rhs.append(ta)
    x, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)
    Rx = x[:9].reshape(3, 3)
    scale = abs(np.linalg.det(Rx)) ** (1.0 / 3.0)
    if scale > 1e-9:
        Rx = Rx / scale
    Rx = _project_so3(Rx)
    return Rx, _solve_translation(pairs, Rx)


NATIVE_SOLVERS = {"tsai": solve_tsai, "park": solve_park, "andreff": solve_andreff}

CV_SOLVERS = {
    "cv-tsai": "CALIB_HAND_EYE_TSAI",
    "cv-park": "CALIB_HAND_EYE_PARK",
    "cv-horaud": "CALIB_HAND_EYE_HORAUD",
    "cv-daniilidis": "CALIB_HAND_EYE_DANIILIDIS",
}

SOLVERS = {**{k: k for k in NATIVE_SOLVERS}, **CV_SOLVERS}


@dataclass
class Sample:
    tcp_pose: list                 # UR pose at capture
    T_cam_target: list             # 4x4, board in camera frame
    reprojection_px: float = 0.0
    distance_mm: float = 0.0
    n_corners: int = 0
    t: float = field(default_factory=time.time)


class HandEyeSession:
    """
    Collect samples, judge whether they are good enough to solve, solve, and
    report how far off the answer is.

    The "good enough" check is the part that is usually missing and the part
    that decides whether the result is worth anything. AX = XB is degenerate
    if every rotation is about the same axis — mathematically the translation
    is then unrecoverable along that axis — so a set of ten poses that all
    differ by a wrist twist looks like ten samples and carries the information
    of one. `readiness()` says so BEFORE the solve, in plain words, because
    afterwards it is indistinguishable from a bad camera.
    """

    MIN_SAMPLES = 5
    RECOMMENDED = 12

    def __init__(self, spec: TargetSpec | None = None):
        self.spec = spec or TargetSpec()
        self.samples: list[Sample] = []
        self.result: dict | None = None

    # -- collection --------------------------------------------------------
    def add(self, image, tcp_pose, intrinsics) -> dict:
        ok, why = available()
        if not ok:
            return {"ok": False, "error": why}
        if not tcp_pose or len(tcp_pose) < 6 or not any(tcp_pose):
            return {"ok": False, "error":
                    "no TCP pose — the robot link must be running, since a "
                    "view without its pose carries no information about where "
                    "the camera was"}
        det = detect_target(image, self.spec, intrinsics)
        if not det.get("ok"):
            return det
        if "T_cam_target" not in det:
            return {"ok": False, "error": "camera intrinsics unavailable — "
                                          "start the colour stream first"}
        self.samples.append(Sample(
            tcp_pose=[float(v) for v in tcp_pose[:6]],
            T_cam_target=det["T_cam_target"],
            reprojection_px=det.get("reprojection_px", 0.0),
            distance_mm=det.get("distance_mm", 0.0),
            n_corners=det.get("n_corners", 0)))
        return {"ok": True, "n": len(self.samples), **self.readiness(),
                "detection": {k: det[k] for k in
                              ("n_corners", "distance_mm", "reprojection_px")
                              if k in det}}

    def remove_last(self) -> dict:
        if not self.samples:
            return {"ok": False, "error": "nothing to remove"}
        self.samples.pop()
        return {"ok": True, "n": len(self.samples), **self.readiness()}

    def clear(self) -> dict:
        self.samples.clear()
        self.result = None
        return {"ok": True, "n": 0}

    # -- diversity ---------------------------------------------------------
    def readiness(self) -> dict:
        """
        Is this set of poses capable of determining the transform?

        Two independent requirements, both reported:
          rotation spread  — the largest pairwise rotation between samples.
                             Under about 30 deg the translation part is poorly
                             conditioned and the answer will look precise and
                             be wrong.
          axis spread      — rotations must be about genuinely different axes.
                             Measured as the smallest singular value of the
                             stacked unit axes; near zero means one axis.
        """
        n = len(self.samples)
        out = {"n": n, "ready": False, "advice": []}
        if n < 2:
            out["advice"].append(
                f"Capture at least {self.MIN_SAMPLES} poses "
                f"({self.RECOMMENDED} is comfortable).")
            return out
        Rs = [pose_to_matrix(s.tcp_pose)[:3, :3] for s in self.samples]
        angles, axes = [], []
        for i in range(n):
            for j in range(i + 1, n):
                rv = matrix_to_rotvec(Rs[i].T @ Rs[j])
                a = float(np.linalg.norm(rv))
                angles.append(math.degrees(a))
                if a > 1e-3:
                    axes.append(rv / a)
        max_angle = max(angles) if angles else 0.0
        axis_rank = 0.0
        if len(axes) >= 3:
            sv = np.linalg.svd(np.asarray(axes), compute_uv=False)
            axis_rank = float(sv[2] / (sv[0] or 1.0))
        reproj = [s.reprojection_px for s in self.samples if s.reprojection_px]
        out.update({
            "max_rotation_deg": round(max_angle, 1),
            "axis_spread": round(axis_rank, 3),
            "mean_reprojection_px": round(sum(reproj) / len(reproj), 3)
            if reproj else None,
            "distance_span_mm": round(
                max(s.distance_mm for s in self.samples) -
                min(s.distance_mm for s in self.samples), 1),
        })
        if n < self.MIN_SAMPLES:
            out["advice"].append(f"Need at least {self.MIN_SAMPLES} poses; "
                                 f"you have {n}.")
        if max_angle < 30.0:
            out["advice"].append(
                f"Largest rotation between poses is only {max_angle:.0f} deg. "
                "Tilt the tool by 30-60 deg between captures — small rotations "
                "cannot determine where the camera sits.")
        if len(axes) >= 3 and axis_rank < 0.15:
            out["advice"].append(
                "Every rotation is about nearly the same axis. Rotate about "
                "all three tool axes, not just the wrist.")
        if out.get("distance_span_mm", 0) < 50 and n >= self.MIN_SAMPLES:
            out["advice"].append(
                "All poses are at a similar distance from the board. Vary the "
                "standoff by 100 mm or more so scale is observable.")
        if reproj and max(reproj) > 1.0:
            out["advice"].append(
                "Some boards were detected with over 1 px reprojection error. "
                "Re-take those: blur and glare there become tool error here.")
        out["ready"] = (n >= self.MIN_SAMPLES and max_angle >= 30.0
                        and not (len(axes) >= 3 and axis_rank < 0.15))
        if out["ready"] and not out["advice"]:
            out["advice"].append("Pose set looks good. Solve.")
        return out

    # -- solve -------------------------------------------------------------
    def solve(self, method: str = "all") -> dict:
        ok, why = available()
        if not ok:
            return {"ok": False, "error": why}
        if len(self.samples) < self.MIN_SAMPLES:
            return {"ok": False, "error":
                    f"need at least {self.MIN_SAMPLES} poses, have "
                    f"{len(self.samples)}"}

        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for s in self.samples:
            T = pose_to_matrix(s.tcp_pose)
            R_g2b.append(T[:3, :3])
            t_g2b.append(T[:3, 3])
            C = np.asarray(s.T_cam_target, dtype=float)
            R_t2c.append(C[:3, :3])
            t_t2c.append(C[:3, 3])

        T_bg = [pose_to_matrix(s.tcp_pose) for s in self.samples]
        T_ct = [np.asarray(s.T_cam_target, dtype=float) for s in self.samples]
        pairs = _motion_pairs(T_bg, T_ct)
        if len(pairs) < 3:
            return {"ok": False, "error":
                    "the poses barely rotate relative to one another, so there "
                    "is nothing to solve. Tilt the tool by 30-60 deg between "
                    "captures and try again."}

        names = list(SOLVERS) if method == "all" else [method]
        per: dict[str, dict] = {}
        for name in names:
            try:
                if name in NATIVE_SOLVERS:
                    R, t = NATIVE_SOLVERS[name](pairs)
                else:
                    attr = CV_SOLVERS.get(name)
                    if attr is None or not hasattr(cv2, "calibrateHandEye") \
                            or not hasattr(cv2, attr):
                        continue
                    R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c,
                                                method=getattr(cv2, attr))
            except Exception as e:                  # noqa: BLE001
                per[name] = {"ok": False, "error": str(e)}
                continue
            X = np.eye(4)
            X[:3, :3] = np.asarray(R, dtype=float)
            X[:3, 3] = np.asarray(t, dtype=float).reshape(3)
            per[name] = {"ok": True, "T_tcp_cam": X.tolist(),
                         **self.residual(X)}

        good = {k: v for k, v in per.items() if v.get("ok")}
        if not good:
            return {"ok": False, "error": "every solver failed", "per_method": per}

        # Pick on the residual that is measured in the work frame, not on the
        # solver's own cost — they are not the same quantity and only one of
        # them is what the reconstruction will suffer.
        best_name = min(good, key=lambda k: good[k]["target_spread_mm"])
        best = good[best_name]
        X = np.asarray(best["T_tcp_cam"], dtype=float)

        spread = [(k, v["target_spread_mm"]) for k, v in good.items()]
        agree_mm = max(
            float(np.linalg.norm(np.asarray(v["T_tcp_cam"])[:3, 3] - X[:3, 3]))
            * 1000.0 for v in good.values())
        agree_deg = max(
            math.degrees(float(np.linalg.norm(matrix_to_rotvec(
                X[:3, :3].T @ np.asarray(v["T_tcp_cam"])[:3, :3]))))
            for v in good.values())

        res = {
            "ok": True,
            "method": best_name,
            "T_tcp_cam": X.tolist(),
            "translation_mm": [round(float(v) * 1000.0, 2) for v in X[:3, 3]],
            "rotation_deg": [round(math.degrees(float(v)), 3)
                             for v in matrix_to_rotvec(X[:3, :3])],
            "n_samples": len(self.samples),
            "solver_agreement_mm": round(agree_mm, 2),
            "solver_agreement_deg": round(agree_deg, 3),
            "per_method": {k: {"target_spread_mm": v["target_spread_mm"],
                               "target_spread_deg": v["target_spread_deg"],
                               "translation_mm":
                                   [round(float(c) * 1000.0, 2)
                                    for c in np.asarray(v["T_tcp_cam"])[:3, 3]]}
                           for k, v in good.items()},
            **{k: v for k, v in best.items()
               if k.startswith("target_") or k == "per_sample_mm"},
            "calib_version": f"handeye-{time.strftime('%Y%m%dT%H%M%S')}",
            "solved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "target": self.spec.as_dict(),
        }
        res["n_motion_pairs"] = len(pairs)
        res["readiness"] = self.readiness()
        res["verdict"] = _verdict(res)
        if agree_mm > 5.0 or agree_deg > 1.0:
            res["warning"] = (
                f"The solvers disagree by up to {agree_mm:.1f} mm / "
                f"{agree_deg:.2f} deg. That is a property of the pose set, not "
                "of the camera — add poses with larger and more varied "
                "rotations and solve again.")
        self.result = res
        return res

    def residual(self, X) -> dict:
        """
        The target has not moved. How much does the calibration say it has?

        Reconstructs T_base_target from every sample and reports the spread
        about the mean, in millimetres and degrees. This is the number that
        predicts what the reconstruction will do.
        """
        X = np.asarray(X, dtype=float)
        poses = []
        for s in self.samples:
            poses.append(pose_to_matrix(s.tcp_pose) @ X @
                         np.asarray(s.T_cam_target, dtype=float))
        pts = np.asarray([T[:3, 3] for T in poses])
        centre = pts.mean(axis=0)
        d_mm = np.linalg.norm(pts - centre, axis=1) * 1000.0

        # Rotation spread about the "average" orientation, taken as the sample
        # whose total angle to the others is smallest — a proper Karcher mean
        # buys nothing here and a reference sample cannot diverge.
        Rs = [T[:3, :3] for T in poses]
        tot = [sum(float(np.linalg.norm(matrix_to_rotvec(Ri.T @ Rj)))
                   for Rj in Rs) for Ri in Rs]
        ref = Rs[int(np.argmin(tot))]
        d_deg = [math.degrees(float(np.linalg.norm(matrix_to_rotvec(ref.T @ R))))
                 for R in Rs]
        return {
            "target_spread_mm": round(float(d_mm.max()), 3),
            "target_rms_mm": round(float(np.sqrt((d_mm ** 2).mean())), 3),
            "target_spread_deg": round(float(max(d_deg)), 3),
            "per_sample_mm": [round(float(v), 2) for v in d_mm],
        }

    def status(self) -> dict:
        ok, why = available()
        return {"available": ok, "error": why, "n": len(self.samples),
                "target": self.spec.as_dict(),
                "samples": [{"pose_mm": [round(v * 1000.0, 1) for v in s.tcp_pose[:3]],
                             "distance_mm": s.distance_mm,
                             "reprojection_px": s.reprojection_px,
                             "corners": s.n_corners} for s in self.samples],
                "readiness": self.readiness() if self.samples else {"n": 0},
                "result": _summary(self.result) if self.result else None}


def _verdict(res: dict) -> str:
    """
    What to tell the operator, in one paragraph.

    The residual ALONE is not enough to judge a calibration, and the failure
    it cannot see is the one that matters most. With every pose rotated about
    a single axis the solve is degenerate: the translation along that axis is
    unobservable, so any value fits, the board reconstructs perfectly, and the
    residual reads 0.0 mm while the answer is centimetres out. That case is
    caught by pose diversity and by solver disagreement, never by the residual,
    so both are checked here BEFORE the residual gets to say anything.
    """
    rd = res.get("readiness") or {}
    agree_mm = res.get("solver_agreement_mm", 0.0)
    agree_deg = res.get("solver_agreement_deg", 0.0)
    if rd and not rd.get("ready", True):
        return ("Do not use this. The poses cannot determine the transform — "
                + (rd.get("advice") or ["add more varied poses"])[0] +
                " A calibration from poses like these can show a perfect "
                "residual and still be centimetres wrong, because the residual "
                "cannot see the direction the poses left unmeasured.")
    if agree_mm > 10.0 or agree_deg > 2.0:
        return (f"Do not use this. Independent solvers disagree by "
                f"{agree_mm:.0f} mm / {agree_deg:.1f} deg on the same data, "
                "which means the pose set does not pin the answer down. Add "
                "poses with larger rotations about all three tool axes.")
    s = res.get("target_spread_mm", 999.0)
    if s <= 2.0:
        return ("Good. The board reconstructs to within "
                f"{s:.1f} mm across every pose — fine for 3D reconstruction "
                "and path planning.")
    if s <= 5.0:
        return (f"Usable. {s:.1f} mm of spread will show up as blur in the "
                "fused cloud. Add a few more poses if you need better.")
    return (f"Not good enough: {s:.1f} mm of spread. Something is wrong — "
            "check the square size in millimetres, that the board is rigid "
            "and did not move, and that the TCP is set correctly on the "
            "pendant. Do not build a reconstruction on this.")


def _summary(res: dict) -> dict:
    keep = ("method", "translation_mm", "rotation_deg", "n_samples",
            "target_spread_mm", "target_rms_mm", "target_spread_deg",
            "solver_agreement_mm", "solver_agreement_deg", "calib_version",
            "verdict", "warning", "solved_utc", "per_method")
    return {k: res[k] for k in keep if k in res}


# ---------------------------------------------------------------------------
# persistence
# ---------------------------------------------------------------------------

DEFAULT_PATH = Path("calibration/handeye.json")


def save(result: dict, path: str | Path = DEFAULT_PATH) -> dict:
    if not result or not result.get("ok"):
        return {"ok": False, "error": "no solved calibration to save"}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {k: v for k, v in result.items() if k != "per_sample_mm"}
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return {"ok": True, "path": str(path.resolve()),
            "calib_version": result.get("calib_version", "")}


def load(path: str | Path = DEFAULT_PATH) -> dict:
    path = Path(path)
    if not path.exists():
        return {"ok": False, "error": f"no saved calibration at {path}"}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": f"{path} is not readable: {e}"}
    T = data.get("T_tcp_cam")
    if not T or len(T) != 4:
        return {"ok": False, "error": f"{path} has no 4x4 T_tcp_cam"}
    data["ok"] = True
    data["path"] = str(path.resolve())
    return data
