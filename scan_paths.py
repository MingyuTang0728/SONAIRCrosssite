"""
Inspection paths that are worth recording.

The cell already had a tool-space box called `scan_shaped`, and it was measured
on the real robot: over 46 seconds the carrier turned **0.9 degrees** in total
and linear acceleration sat flat on the sensor's own noise floor, 27 samples out
of 8749 above 0.5 m/s^2. It looks exactly like an inspection scan and, to an
inertial unit, is indistinguishable from standing still.

The reason is the whole point of this module. A raster traced over a FLAT
surface with the tool held at a FIXED orientation moves the tool without ever
rotating it, and orientation and angular rate are two of the three channels the
benchmark scores. There is nothing wrong with the path; there is nothing in it.

Real parts are not flat and real inspection does not hold a fixed orientation.
A probe, a camera at fixed standoff, an ultrasonic wheel -- all of them are held
NORMAL TO THE SURFACE, so sweeping across a curved part rotates the tool
continuously, by exactly the curvature of the part. That rotation is free: it is
not an artificial wiggle added to give the sensors something to look at, it is
what the job actually requires. Follow the surface of a cylinder and the tool
sweeps through the arc angle every pass.

So the paths here are generated from a PART, not from a box: give it a radius,
an arc to cover and a length to cover, and it returns the poses that keep the
tool at a fixed standoff, normal to the surface, tracing a zig-zag over it.

Poses are UR tool poses -- x, y, z in metres, then an axis-angle rotation vector
rx, ry, rz in radians -- in the robot's base frame.
"""
from __future__ import annotations

import math

# The tool's own +Z points out of the flange, along the tool. To hold a tool
# against a surface it must point INTO the surface, i.e. along -n where n is the
# outward surface normal.
_TOOL_AXIS = (0.0, 0.0, 1.0)


def _norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v] if n > 1e-12 else [0.0, 0.0, 1.0]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _matrix_to_rotvec(R):
    """3x3 rotation matrix -> axis-angle vector, which is what a UR pose uses."""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return [0.0, 0.0, 0.0]
    if abs(math.pi - ang) < 1e-6:
        # Near 180 degrees the off-diagonal differences vanish; take the axis
        # from the diagonal instead, which stays well conditioned there.
        d = [math.sqrt(max(0.0, (R[i][i] + 1.0) / 2.0)) for i in range(3)]
        i = d.index(max(d))
        axis = [0.0, 0.0, 0.0]
        axis[i] = d[i]
        for j in range(3):
            if j != i:
                axis[j] = (R[i][j] + R[j][i]) / (4.0 * d[i]) if d[i] > 1e-9 else 0.0
        return [a * ang for a in _norm(axis)]
    k = ang / (2.0 * math.sin(ang))
    return [k * (R[2][1] - R[1][2]),
            k * (R[0][2] - R[2][0]),
            k * (R[1][0] - R[0][1])]


def _pose_from_frame(pos, z_axis, x_hint):
    """
    A tool pose whose Z points along `z_axis`, with X as close to `x_hint` as
    that allows. Returns [x, y, z, rx, ry, rz].
    """
    z = _norm(z_axis)
    x = [c - _dot(x_hint, z) * zc for c, zc in zip(x_hint, z)]
    if math.sqrt(sum(c * c for c in x)) < 1e-6:
        # The hint was parallel to Z; any perpendicular will do.
        alt = [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0]
        x = [c - _dot(alt, z) * zc for c, zc in zip(alt, z)]
    x = _norm(x)
    y = _cross(z, x)
    R = [[x[0], y[0], z[0]],
         [x[1], y[1], z[1]],
         [x[2], y[2], z[2]]]
    return list(pos) + _matrix_to_rotvec(R)


def arc_zigzag(*, centre, axis=(0.0, 0.0, 1.0), radius=0.12, standoff=0.05,
               arc_deg=70.0, length=0.18, passes=8, points_per_pass=16,
               start_offset=0.0):
    """
    A zig-zag over the outside of a cylinder, tool normal to the surface.

    `centre`      a point on the cylinder's axis, in the robot's base frame
    `axis`        the cylinder's axis direction
    `radius`      the part's radius, metres
    `standoff`    how far off the surface the tool rides, metres
    `arc_deg`     how much of the circumference one pass covers
    `length`      how far along the axis the whole scan covers
    `passes`      how many sweeps across the arc (alternating direction)
    `points_per_pass`  waypoints per sweep -- more is smoother, not faster

    Returns a list of tool poses. Consecutive poses differ by a small rotation
    as well as a small translation, which is the property the flat box lacked.
    """
    axis = _norm(axis)
    # Two directions perpendicular to the axis, to sweep the arc in.
    seed = [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0]
    u = _norm([c - _dot(seed, axis) * ac for c, ac in zip(seed, axis)])
    w = _cross(axis, u)

    half = math.radians(arc_deg) / 2.0
    poses = []
    passes = max(1, int(passes))
    n = max(2, int(points_per_pass))
    for p in range(passes):
        # Where along the axis this pass sits, and which way it sweeps. The
        # alternation is what makes it a zig-zag rather than a comb: the tool
        # never lifts and returns, it turns round and comes back.
        s = (start_offset + (length * p / max(1, passes - 1))) if passes > 1 \
            else start_offset
        along = [ac * s for ac in axis]
        for i in range(n):
            f = i / (n - 1)
            if p % 2:
                f = 1.0 - f
            ang = -half + 2.0 * half * f
            # Outward normal at this angle, and the point at the standoff.
            nx = [u[k] * math.cos(ang) + w[k] * math.sin(ang) for k in range(3)]
            pos = [centre[k] + along[k] + nx[k] * (radius + standoff)
                   for k in range(3)]
            # Tool points INTO the surface, and its X runs along the part, so
            # the tool's own frame stays consistent from pass to pass rather
            # than flipping when the sweep reverses.
            poses.append(_pose_from_frame(pos, [-c for c in nx], axis))
    return poses


def describe(poses, speed_m_s) -> dict:
    """
    What this path will actually do: how far the tool travels, how long that
    takes, and -- the number that decides whether it is worth recording -- how
    fast the tool ROTATES while doing it.
    """
    if len(poses) < 2:
        return {"points": len(poses)}
    dist = 0.0
    turn = 0.0
    for a, b in zip(poses, poses[1:]):
        dist += math.dist(a[:3], b[:3])
        turn += _angle_between(a[3:6], b[3:6])
    secs = dist / max(speed_m_s, 1e-6)
    return {
        "points": len(poses),
        "path_length_m": round(dist, 4),
        "total_rotation_deg": round(math.degrees(turn), 1),
        "duration_s": round(secs, 1),
        "mean_linear_speed_m_s": round(speed_m_s, 4),
        "mean_angular_rate_deg_s": round(math.degrees(turn) / secs, 2)
        if secs > 0 else 0.0,
    }


def _angle_between(rv_a, rv_b) -> float:
    """Angle between two orientations given as rotation vectors, radians."""
    qa, qb = _rotvec_to_quat(rv_a), _rotvec_to_quat(rv_b)
    d = abs(sum(x * y for x, y in zip(qa, qb)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def _rotvec_to_quat(rv):
    ang = math.sqrt(sum(c * c for c in rv))
    if ang < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    a = [c / ang for c in rv]
    s = math.sin(ang / 2.0)
    return [math.cos(ang / 2.0), a[0] * s, a[1] * s, a[2] * s]
