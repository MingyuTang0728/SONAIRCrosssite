"""
UR5e kinematics: where the flange is, which way it faces, and how fast it turns.

Three things in this project need the robot's geometry and nothing else:

  * checking that a planned joint excursion keeps the tool inside the safe
    envelope BEFORE the arm is asked to make it -- a batch of 270 runs cannot be
    supervised move by move;
  * the angular velocity of the flange, which is what the IMU bolted to it
    measures, independently of how the IMU is mounted -- the basis of both the
    time alignment and the mounting-rotation estimate;
  * the same quantities for the simulator's side, so both are computed one way.

Nominal Denavit-Hartenberg parameters, as published by Universal Robots for
the UR5e. Each individual arm is factory-calibrated to within a millimetre or
two of these; the difference was measured on this cell at 1.2-1.5 mm and about
0.2 deg, which is a floor on any position comparison made against nominal
kinematics and belongs in the error budget, not in the gap.

Pure numpy, no robot connection.
"""
from __future__ import annotations

import numpy as np

# UR5e nominal DH (standard convention): a, d, alpha per joint.
UR5E_DH = {
    "a": (0.0, -0.425, -0.3922, 0.0, 0.0, 0.0),
    "d": (0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996),
    "alpha": (np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0),
}


def _dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def frames(q, dh=UR5E_DH):
    """The base-frame transform of every joint frame, T_0^0 .. T_0^6."""
    out = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        T = T @ _dh(float(q[i]), dh["d"][i], dh["a"][i], dh["alpha"][i])
        out.append(T)
    return out


def fk(q, dh=UR5E_DH):
    """Flange pose in the base frame, as a 4x4 transform."""
    return frames(q, dh)[-1]


def fk_pose(q, dh=UR5E_DH):
    """Flange pose as [x, y, z, rx, ry, rz], the UR pose convention."""
    T = fk(q, dh)
    return list(T[:3, 3]) + list(rotvec(T[:3, :3]))


def body_angular_velocity(q, qd, dh=UR5E_DH):
    """
    Angular velocity of the flange, expressed IN THE FLANGE FRAME, rad/s.

    From the joint velocities through the rotational Jacobian rather than by
    differencing orientations: the controller reports joint velocities
    directly, so there is no differentiation noise to fight.

    An IMU rigidly attached to the flange measures exactly this vector, rotated
    by however the IMU happens to be mounted. Its MAGNITUDE is therefore the
    same in both, which is what lets the time offset be found before the
    mounting is known.
    """
    Ts = frames(q, dh)
    w_base = np.zeros(3)
    for i in range(6):
        w_base += Ts[i][:3, 2] * float(qd[i])       # z axis of frame i-1
    return Ts[-1][:3, :3].T @ w_base


def rotvec(R):
    """Rotation matrix -> axis-angle vector."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = float(np.arccos(c))
    if ang < 1e-9:
        return np.zeros(3)
    if abs(np.pi - ang) < 1e-6:
        d = np.sqrt(np.maximum(0.0, (np.diag(R) + 1.0) / 2.0))
        i = int(np.argmax(d))
        axis = np.zeros(3)
        axis[i] = d[i]
        for j in range(3):
            if j != i:
                axis[j] = (R[i, j] + R[j, i]) / (4.0 * d[i])
        return axis / np.linalg.norm(axis) * ang
    k = ang / (2.0 * np.sin(ang))
    return k * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rotmat(rv):
    """Axis-angle vector -> rotation matrix."""
    rv = np.asarray(rv, dtype=float)
    ang = float(np.linalg.norm(rv))
    if ang < 1e-12:
        return np.eye(3)
    k = rv / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
