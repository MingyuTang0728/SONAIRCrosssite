"""
ur_jog.py — host-side jog controller.

The browser must never set the cadence of robot motion. A page that is also
decoding camera frames and running a 3D view cannot hold a 100 ms timer: on
this project's own console, `setInterval(100)` measured a median of 248 ms with
nearly half the ticks past 250 ms. Every tick later than a `speedl`'s `t`
expires the command, so the arm decelerates to zero and re-accelerates — the
stutter that started this.

So the browser sends INTENT and nothing else:

    jog_vel   here is the velocity I want, and how long to trust it
    jog_stop  stop now

and this module owns the timing. A thread re-issues the command at a steady
rate with generous overlap, slew-limits direction changes so they are ramps
rather than steps, and stops on its own if the browser goes quiet. A 300 ms
browser stall now changes nothing at the arm.

The watchdog is the safety-relevant half. It is driven by the host's monotonic
clock, never by a timestamp the browser supplies, because a stalled or
disconnected browser is exactly the case where its clock cannot be trusted and
exactly the case where the arm must stop.
"""
from __future__ import annotations

import logging
import math
import threading
import time

log = logging.getLogger("ur.jog")

# 20 Hz with t = 0.15 s gives 3x overlap: two consecutive sends may be lost
# and the arm still never sees a gap. Raising the rate further mostly buys
# more chances for the controller to interrupt itself on the secondary
# interface, which is the other half of what made jogging rough.
TICK_HZ = 20.0
COMMAND_T = 0.15

# Ramps, in units per second. A UR will happily step its commanded velocity;
# the mechanism will not, and the operator feels the difference as a knock.
LINEAR_SLEW = 0.60      # m/s per second
ANGULAR_SLEW = 2.50     # rad/s per second

DEFAULT_TTL = 0.40      # seconds of silence before the watchdog stops the arm


def _approach(cur: float, target: float, max_delta: float) -> float:
    if target > cur:
        return min(target, cur + max_delta)
    return max(target, cur - max_delta)


class JogController:
    """
    One velocity target, one thread, one watchdog.

    Deliberately not re-entrant per axis: a jog is a single intent, and letting
    two sources each own three axes produces motion neither of them asked for.
    """

    def __init__(self, controller, tick_hz: float = TICK_HZ):
        self.ctrl = controller
        self.period = 1.0 / max(1.0, tick_hz)
        self._lock = threading.Lock()
        self._target = [0.0] * 6      # what the operator is asking for
        self._current = [0.0] * 6     # what we are actually commanding
        self._deadline = 0.0          # host monotonic time; browser cannot set it
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._sent = 0
        self._stops = 0
        self._watchdog_trips = 0
        self._last_err = ""
        self._moving = False

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="ur-jog")
        self._thread.start()
        log.info("jog controller running at %.0f Hz (command t=%.2fs)",
                 1.0 / self.period, COMMAND_T)

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    # --- intent from the browser -------------------------------------------

    def set_velocity(self, xd, ttl_s: float = DEFAULT_TTL,
                     max_linear: float | None = None,
                     max_angular: float | None = None) -> dict:
        """
        Set the target. Cheap on purpose — it takes a lock, writes six floats
        and returns, so it can be handled inline on the socket's own task
        without a thread hop. A jog message that waits on a thread pool has
        already lost the latency budget it was trying to protect.
        """
        v = [0.0] * 6
        for i in range(min(6, len(xd))):
            try:
                v[i] = float(xd[i])
            except (TypeError, ValueError):
                v[i] = 0.0
            if not math.isfinite(v[i]):
                v[i] = 0.0

        env = getattr(self.ctrl, "envelope", None)
        lim_lin = max_linear if max_linear is not None else (
            getattr(env, "max_linear_speed", 0.25) if env else 0.25)
        lim_ang = max_angular if max_angular is not None else 1.5

        lin = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        if lin > lim_lin > 0:
            k = lim_lin / lin
            v[0] *= k; v[1] *= k; v[2] *= k
        ang = math.sqrt(v[3] ** 2 + v[4] ** 2 + v[5] ** 2)
        if ang > lim_ang > 0:
            k = lim_ang / ang
            v[3] *= k; v[4] *= k; v[5] *= k

        with self._lock:
            self._target = v
            self._deadline = time.monotonic() + max(0.05, min(2.0, float(ttl_s)))
        return {"ok": True, "applied": v}

    def stop(self) -> dict:
        """Ask for zero. The ramp still applies, so this is a controlled stop."""
        with self._lock:
            self._target = [0.0] * 6
            self._deadline = time.monotonic() + DEFAULT_TTL
        return {"ok": True}

    def halt(self) -> dict:
        """Immediate stop, no ramp. For the stop button, not for releasing a pad."""
        with self._lock:
            self._target = [0.0] * 6
            self._current = [0.0] * 6
            self._deadline = 0.0
        ok, msg = self.ctrl.stop(a=2.0)
        self._stops += 1
        return {"ok": ok, "msg": msg}

    # --- the loop -----------------------------------------------------------

    def _loop(self) -> None:
        next_t = time.monotonic()
        idle_sent = True
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(self.period, next_t - now))
                continue
            next_t += self.period
            if now - next_t > 0.5:      # fell far behind; resynchronise
                next_t = now

            with self._lock:
                target = list(self._target)
                expired = now > self._deadline
                if expired and any(target):
                    self._target = [0.0] * 6
                    self._watchdog_trips += 1
                    log.debug("jog watchdog: no update, stopping")
                if expired:
                    target = [0.0] * 6
                cur = list(self._current)

            lin_step = LINEAR_SLEW * self.period
            ang_step = ANGULAR_SLEW * self.period
            for i in range(3):
                cur[i] = _approach(cur[i], target[i], lin_step)
            for i in range(3, 6):
                cur[i] = _approach(cur[i], target[i], ang_step)

            with self._lock:
                self._current = cur

            moving = any(abs(c) > 1e-6 for c in cur)
            if moving:
                # Acceleration high enough that the arm tracks the ramp we are
                # already applying, rather than adding a second, slower one.
                ok, msg = self.ctrl.speedl(cur, a=1.5, t=COMMAND_T)
                self._sent += 1
                self._moving = True
                idle_sent = False
                if not ok:
                    self._last_err = msg
            elif not idle_sent:
                # One stop on the way down, then silence. Streaming zeros keeps
                # interrupting the controller's program for no benefit.
                self.ctrl.stop(a=2.0)
                self._stops += 1
                self._moving = False
                idle_sent = True

    # --- status -------------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            cur = list(self._current)
            tgt = list(self._target)
            remaining = max(0.0, self._deadline - time.monotonic())
        return {
            "running": bool(self._thread and self._thread.is_alive()),
            "rate_hz": 1.0 / self.period,
            "moving": self._moving,
            "current": cur,
            "target": tgt,
            "ttl_remaining_s": remaining,
            "commands_sent": self._sent,
            "stops": self._stops,
            "watchdog_trips": self._watchdog_trips,
            "last_error": self._last_err,
        }


# ---------------------------------------------------------------------------
# step jog
# ---------------------------------------------------------------------------

def step(controller, current_pose, axis: str, distance_mm: float,
         frame: str = "base", speed: float = 0.05) -> dict:
    """
    Move a fixed increment and stop.

    Immune to timing by construction: it is one `movel` to an absolute target,
    so a browser stall cannot shorten or extend it. This is how an operator
    positions a tool to a millimetre, and it is the right default for anything
    that needs to be repeatable — the continuous jog is for getting roughly
    there quickly.

    `frame` is "base" (world axes) or "tool" (along the tool's own axes).
    """
    if not current_pose or len(current_pose) < 6:
        return {"ok": False, "error": "no current tool position — is the robot connected?"}

    axis = str(axis).lower()
    idx = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}.get(axis)
    if idx is None:
        return {"ok": False, "error": f"unknown axis {axis!r}"}

    target = list(current_pose[:6])
    if idx < 3:
        d = float(distance_mm) / 1000.0
        if frame == "tool":
            try:
                import numpy as np
                from scan3d import rotvec_to_matrix
                R = rotvec_to_matrix(current_pose[3:6])
                offset = R @ (np.array([1.0 if k == idx else 0.0 for k in range(3)]) * d)
                for k in range(3):
                    target[k] += float(offset[k])
            except Exception as e:
                return {"ok": False, "error": f"tool-frame step needs numpy: {e}"}
        else:
            target[idx] += d
    else:
        # Rotation steps are given in degrees; a rotation vector is not a set
        # of independent angles, so this is only valid for small increments
        # and is capped accordingly.
        deg = max(-15.0, min(15.0, float(distance_mm)))
        target[idx] += math.radians(deg)

    ok, msg = controller.movel(target, a=0.5, v=max(0.005, float(speed)), r=0.0)
    return {"ok": ok, "msg": msg, "target": target}
