"""
Orientation from raw inertial data — pure stdlib, no numpy.

Why this module exists: the benchmark's headline modality is ORIENTATION, but
only one of the three inertial tiers actually reports it. FusionHub fuses
on-board and streams a quaternion; the D435i's BMI055 and any bare consumer
module report accelerometer and gyroscope only. Without this module the two
tiers are not comparable, and "compare orientation across units" — which is
the whole point of having more than one unit — is not a thing you can do.

Two estimators, and the choice between them is not a preference:

  Madgwick   gradient-descent fusion of gyro + accel. Tracks fast motion
             through the gyro and pulls the estimate back to gravity slowly,
             which is what you want on a moving arm.
  Complementary  a first-order blend. Cheaper, easier to reason about, and
             used here as the cross-check: if the two disagree by more than a
             degree or so on the same data, something is wrong with the data,
             not with the filter.

Neither can observe heading. With no magnetometer, rotation about gravity is
unobservable from accel+gyro alone, so yaw drifts at the gyro's bias rate and
IS REPORTED AS SUCH (`yaw_observable: False`). Quoting a yaw number from a
6-axis unit without saying that is how a 3 deg/hour bias becomes a silent 10 mm
position error four hours into a campaign.

Gyro bias is ESTIMATED FROM A STATIONARY WINDOW rather than assumed zero. A
BMI055 sitting on a bench routinely shows 1-2 deg/s of bias; integrated over a
60 s run that is 60-120 deg of pure fiction.
"""
from __future__ import annotations

import math
import time
from collections import deque

GRAVITY = 9.80665

# The largest integration step that is taken at face value. At 100 Hz a real
# gap is 10 ms; 200 ms is twenty missed packets, which is a dropout worth
# knowing about and still small enough that integrating across it is sane.
# Anything beyond it is a clock fault, not a gap.
MAX_STEP_S = 0.20


# ---------------------------------------------------------------------------
# quaternion helpers  (w, x, y, z — Hamilton convention, same as FusionHub)
# ---------------------------------------------------------------------------

def q_normalise(q):
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [w / n, x / n, y / n, z / n]


def q_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def q_conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]


def q_to_euler_deg(q):
    """Roll, pitch, yaw in degrees (ZYX / aerospace order)."""
    w, x, y, z = q_normalise(q)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    # Clamp rather than let a slightly-over-unit value raise: at exactly
    # +/-90 deg pitch the expression tips over 1.0 on rounding alone.
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def q_to_matrix(q):
    """3x3 rotation matrix as a list of lists."""
    w, x, y, z = q_normalise(q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ]


def q_angle_deg(a, b) -> float:
    """Geodesic angle between two orientations, in degrees."""
    a = q_normalise(a)
    b = q_normalise(b)
    d = abs(sum(ai * bi for ai, bi in zip(a, b)))
    d = max(-1.0, min(1.0, d))
    return math.degrees(2.0 * math.acos(d))


def gravity_from_quat(q):
    """The gravity direction expressed in the sensor frame."""
    w, x, y, z = q_normalise(q)
    return [2.0 * (x * z - w * y),
            2.0 * (w * x + y * z),
            w * w - x * x - y * y + z * z]


def tilt_from_accel(accel):
    """
    Roll and pitch straight out of one accelerometer reading, in degrees.

    This is the zero-state estimate and the sanity check on the filters: while
    the unit is still it is exact, and it needs no history at all.
    """
    ax, ay, az = (float(v) for v in accel)
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-6:
        return [0.0, 0.0]
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return [math.degrees(roll), math.degrees(pitch)]


# ---------------------------------------------------------------------------
# gyro bias
# ---------------------------------------------------------------------------

class GyroBias:
    """
    Running bias estimate, updated only while the unit is demonstrably still.

    Stillness is judged on BOTH channels: the accelerometer norm must sit near
    1 g and the gyro magnitude must be small. Using the gyro alone would let a
    unit in steady free-fall or on a smoothly rotating fixture count as still,
    and a bias learned during motion is worse than no bias correction at all.
    """

    def __init__(self, accel_tol: float = 0.35, gyro_tol: float = 0.06,
                 tau: float = 4.0):
        self.accel_tol = accel_tol     # m/s^2 away from 1 g
        self.gyro_tol = gyro_tol       # rad/s
        self.tau = tau                 # s, learning time constant
        self.bias = [0.0, 0.0, 0.0]
        self.still = False
        self.still_s = 0.0
        self.n_updates = 0

    def update(self, accel, gyro, dt: float) -> list:
        if dt <= 0.0 or dt > 1.0:
            return list(self.bias)
        an = math.sqrt(sum(v * v for v in accel)) if accel else 0.0
        gn = math.sqrt(sum(v * v for v in gyro)) if gyro else 0.0
        self.still = (abs(an - GRAVITY) < self.accel_tol) and (gn < self.gyro_tol)
        if not self.still:
            self.still_s = 0.0
            return list(self.bias)
        self.still_s += dt
        # Only start learning after half a second of stillness, so the tail of
        # a movement — which is still, briefly, at every turnaround — does not
        # get averaged into the bias.
        if self.still_s < 0.5:
            return list(self.bias)
        k = min(1.0, dt / self.tau)
        self.bias = [b + k * (g - b) for b, g in zip(self.bias, gyro)]
        self.n_updates += 1
        return list(self.bias)

    def apply(self, gyro):
        return [g - b for g, b in zip(gyro, self.bias)]

    def status(self) -> dict:
        return {"bias_rad_s": [round(b, 6) for b in self.bias],
                "bias_deg_s": [round(math.degrees(b), 4) for b in self.bias],
                "still": self.still,
                "still_s": round(self.still_s, 2),
                "updates": self.n_updates}


# ---------------------------------------------------------------------------
# estimators
# ---------------------------------------------------------------------------

class MadgwickAHRS:
    """
    6-axis Madgwick. `beta` trades gyro trust against accelerometer trust:
    higher converges faster to gravity and lets vibration through, lower is
    smoother and drifts longer. 0.04 suits an arm-mounted unit.
    """

    def __init__(self, beta: float = 0.04):
        self.beta = beta
        self.q = [1.0, 0.0, 0.0, 0.0]

    def reset_from_accel(self, accel) -> list:
        """Seed the filter from gravity so it does not start 90 deg out."""
        roll, pitch = (math.radians(v) for v in tilt_from_accel(accel))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        self.q = q_normalise([cr * cp, sr * cp, cr * sp, -sr * sp])
        return list(self.q)

    def update(self, gyro, accel, dt: float) -> list:
        if dt <= 0.0 or dt > 1.0:
            return list(self.q)
        gx, gy, gz = (float(v) for v in gyro)
        q1, q2, q3, q4 = self.q

        # gyro-only rate of change
        qd = [0.5 * (-q2 * gx - q3 * gy - q4 * gz),
              0.5 * (q1 * gx + q3 * gz - q4 * gy),
              0.5 * (q1 * gy - q2 * gz + q4 * gx),
              0.5 * (q1 * gz + q2 * gy - q3 * gx)]

        n = math.sqrt(sum(float(v) * float(v) for v in accel)) if accel else 0.0
        if n > 1e-6:
            ax, ay, az = (float(v) / n for v in accel)
            f1 = 2 * (q2 * q4 - q1 * q3) - ax
            f2 = 2 * (q1 * q2 + q3 * q4) - ay
            f3 = 2 * (0.5 - q2 * q2 - q3 * q3) - az
            j11, j12, j13, j14 = -2 * q3, 2 * q4, -2 * q1, 2 * q2
            j21, j22, j23, j24 = 2 * q2, 2 * q1, 2 * q4, 2 * q3
            j32, j33 = -4 * q2, -4 * q3
            s = [j11 * f1 + j21 * f2,
                 j12 * f1 + j22 * f2 + j32 * f3,
                 j13 * f1 + j23 * f2 + j33 * f3,
                 j14 * f1 + j24 * f2]
            sn = math.sqrt(sum(v * v for v in s))
            if sn > 1e-12:
                s = [v / sn for v in s]
                qd = [d - self.beta * v for d, v in zip(qd, s)]

        self.q = q_normalise([q + d * dt for q, d in zip(self.q, qd)])
        return list(self.q)


class ComplementaryAHRS:
    """
    The cross-check, in Mahony form: the accelerometer's disagreement with the
    estimated gravity direction is fed back as an extra angular rate before
    integration, rather than as a post-hoc nudge to the quaternion.

    Doing the correction in the rate domain is what keeps it a genuinely
    independent estimator of the same quantity — it shares no code path with
    the Madgwick gradient step, so agreement between the two says something.
    """

    def __init__(self, kp: float = 1.2):
        self.kp = kp
        self.q = [1.0, 0.0, 0.0, 0.0]

    def reset_from_accel(self, accel) -> list:
        m = MadgwickAHRS()
        self.q = m.reset_from_accel(accel)
        return list(self.q)

    def update(self, gyro, accel, dt: float) -> list:
        if dt <= 0.0 or dt > 1.0:
            return list(self.q)
        w = [float(v) for v in gyro]
        n = math.sqrt(sum(float(v) * float(v) for v in accel)) if accel else 0.0
        if n > 1e-6 and abs(n - GRAVITY) < 1.5:
            # Trust the accelerometer as a gravity reference only while its
            # magnitude is plausibly gravity. Under a hard acceleration it is
            # not, and correcting toward it then tips the estimate the wrong way.
            g_est = gravity_from_quat(self.q)
            a = [float(v) / n for v in accel]
            err = [a[1] * g_est[2] - a[2] * g_est[1],
                   a[2] * g_est[0] - a[0] * g_est[2],
                   a[0] * g_est[1] - a[1] * g_est[0]]
            w = [wi + self.kp * e for wi, e in zip(w, err)]
        dq = q_multiply(self.q, [0.0, w[0] * dt, w[1] * dt, w[2] * dt])
        self.q = q_normalise([qi + 0.5 * d for qi, d in zip(self.q, dq)])
        return list(self.q)


class AttitudeTracker:
    """
    One per inertial unit. Feed it whatever the unit reports; it fills in what
    the unit does not.

    A unit that already streams a quaternion (FusionHub) keeps it — this class
    never overwrites measured orientation with an estimate — but still runs the
    estimator alongside so the two can be differenced. That difference is a
    free, continuous check on the fusion running inside the other box, and it
    has caught a mis-set output frame more than once.
    """

    def __init__(self, unit: str = "imu", beta: float = 0.04):
        self.unit = unit
        self.madgwick = MadgwickAHRS(beta=beta)
        self.comp = ComplementaryAHRS()
        self.bias = GyroBias()
        self.seeded = False
        self.t_last: float | None = None
        self.n = 0
        self.quat_source = "none"       # "device" | "estimated"
        self.quat = [1.0, 0.0, 0.0, 0.0]
        self.est_quat = [1.0, 0.0, 0.0, 0.0]
        self.disagreement_deg = 0.0
        self.device_vs_est_deg = 0.0
        self.device_vs_est_tilt_deg = 0.0
        self.rate_hz = 0.0
        # The rate is measured on the HOST clock, not on the sensor's own
        # timestamp. A sensor clock only has to hiccup once -- one packet
        # whose time field reads far in the future -- for a window anchored to
        # it to stop closing, and the symptom is an update rate frozen at 0 Hz
        # while data is plainly arriving. The host clock cannot do that, and
        # "how fast is it arriving" is the question the number answers anyway.
        self._arrivals: deque = deque(maxlen=256)
        self.sensor_clock_ok = True
        self.n_clock_jumps = 0

    def update(self, t: float, rec: dict) -> dict:
        """
        Returns the derived block to merge into the unit's record:
        quaternion, Euler angles, the source of each, and the health numbers.
        """
        accel = rec.get("accel")
        gyro = rec.get("gyro")
        dev_q = rec.get("quat")
        # A source that has not yet established what units its gyroscope
        # reports in marks its rows. Integrating those would be integrating a
        # number whose scale is unknown, so the gyro is ignored until the
        # source is sure; orientation still comes from gravity meanwhile.
        if rec.get("_units_pending"):
            gyro = None

        # Integration step, from the sensor clock, GUARDED. An unguarded dt
        # is the one input that can destroy the estimate outright: a single
        # packet carrying a stale or bogus timestamp yields a dt of hours,
        # and Madgwick integrates the gyroscope over all of it in one step.
        # Outside the plausible band the step is dropped rather than trusted,
        # and the fact is counted and reported rather than hidden.
        dt = 0.0
        if self.t_last is not None:
            raw_dt = t - self.t_last
            if 0.0 < raw_dt <= MAX_STEP_S:
                dt = raw_dt
            else:
                self.n_clock_jumps += 1
                self.sensor_clock_ok = False
        self.t_last = t
        self.n += 1

        # Arrival rate over a sliding window of the last few hundred packets,
        # on the host clock.
        now_host = time.monotonic()
        self._arrivals.append(now_host)
        if len(self._arrivals) >= 2:
            span = self._arrivals[-1] - self._arrivals[0]
            if span > 0.05:
                self.rate_hz = (len(self._arrivals) - 1) / span

        if accel and gyro:
            self.bias.update(accel, gyro, dt)
            g = self.bias.apply(gyro)
            if not self.seeded:
                self.madgwick.reset_from_accel(accel)
                self.comp.reset_from_accel(accel)
                self.seeded = True
            qm = self.madgwick.update(g, accel, dt)
            qc = self.comp.update(g, accel, dt)
            self.est_quat = qm
            self.disagreement_deg = q_angle_deg(qm, qc)

        if dev_q:
            self.quat = q_normalise(dev_q)
            self.quat_source = "device"
            if self.seeded:
                self.device_vs_est_deg = q_angle_deg(self.quat, self.est_quat)
                # The whole-rotation difference is NOT a fair comparison here.
                # The estimator runs on gyroscope and accelerometer only, so
                # its heading has no reference and starts at zero; the device
                # fuses a magnetometer and reports a real heading. Differencing
                # the two whole rotations therefore reports the heading offset
                # -- routinely over a hundred degrees -- as if it were an
                # error, which reads as a broken sensor and is not one.
                # What both CAN see is the direction of gravity, so that is
                # what is compared.
                ga = gravity_from_quat(self.quat)
                gb = gravity_from_quat(self.est_quat)
                dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(ga, gb))))
                self.device_vs_est_tilt_deg = math.degrees(math.acos(dot))
        elif self.seeded:
            self.quat = list(self.est_quat)
            self.quat_source = "estimated"

        euler = q_to_euler_deg(self.quat)
        out = {
            "quat": [round(v, 6) for v in self.quat],
            "quat_source": self.quat_source,
            "euler_deg": [round(v, 3) for v in euler],
            "yaw_observable": bool(dev_q) or bool(rec.get("mag")),
            "rate_hz": round(self.rate_hz, 1),
        }
        if not self.sensor_clock_ok:
            out["clock_jumps"] = self.n_clock_jumps
        if accel:
            out["tilt_deg"] = [round(v, 3) for v in tilt_from_accel(accel)]
            out["accel_norm"] = round(
                math.sqrt(sum(float(v) * float(v) for v in accel)), 4)
            # What is left of the acceleration once gravity is taken out: the
            # part that comes from the arm actually moving. It is the channel
            # the benchmark scores against a simulated trajectory, because a
            # simulator reproduces motion and does not reproduce the 9.81 a
            # stationary sensor reads.
            g = gravity_from_quat(self.quat)
            lin = [float(accel[i]) - g[i] * GRAVITY for i in range(3)]
            out["linear_accel"] = [round(v, 4) for v in lin]
            out["linear_accel_norm"] = round(
                math.sqrt(sum(v * v for v in lin)), 4)
        if gyro:
            out["gyro_norm_deg_s"] = round(math.degrees(
                math.sqrt(sum(float(v) * float(v) for v in gyro))), 3)
        if self.seeded:
            out["filter_disagreement_deg"] = round(self.disagreement_deg, 3)
            out["still"] = self.bias.still
            out["gyro_bias_deg_s"] = [round(math.degrees(b), 4)
                                      for b in self.bias.bias]
        if dev_q and self.seeded:
            out["device_vs_estimate_deg"] = round(self.device_vs_est_deg, 3)
            out["device_vs_estimate_tilt_deg"] = round(
                self.device_vs_est_tilt_deg, 3)
        return out

    def status(self) -> dict:
        return {"unit": self.unit, "samples": self.n,
                "rate_hz": round(self.rate_hz, 1),
                "quat_source": self.quat_source,
                "sensor_clock_ok": self.sensor_clock_ok,
                "clock_jumps": self.n_clock_jumps,
                "seeded": self.seeded, **self.bias.status()}
