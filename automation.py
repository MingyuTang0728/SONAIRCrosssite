"""
automation.py — the cell runs the job, not the operator.

Everything this console could do, it could do one button at a time. That is
fine for bringing a cell up and useless for building a dataset: a benchmark
wants forty runs that differ in three parameters and are identical in every
other respect, and forty runs pressed by hand differ in every respect a human
cannot hold still — the pause before the start, the exact pose the arm was
left in, whether the log was running yet.

So a job is declared once and executed by the machine. The same job run twice
produces two runs that differ only where the job says they differ, which is
the whole basis on which two numbers from them can be compared.

THREE THINGS THIS MODULE REFUSES TO DO.

It will not move the arm until the pre-flight checks pass. Not warn — refuse.
A capture campaign that starts against a robot in local mode, or with no
calibration loaded, produces a folder full of files that look exactly like
good ones and are not, and the cost of that is not discovered until someone
tries to use them weeks later.

It will not carry on past a step that failed. A half-finished run in a
dataset is worse than a missing one, because a missing one is visible.

It will not run two jobs at once. There is one robot.

The steps are deliberately small and each maps onto something the console
already does by hand, so anything that can be driven here can be driven there
and debugged there.
"""
from __future__ import annotations

import json
import logging
import math
import shutil
import statistics as st

try:
    import scan_paths
except Exception:       # noqa: BLE001
    scan_paths = None
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

log = logging.getLogger("automation")

# The step vocabulary. Small on purpose: every one of these is a thing an
# operator can also do by hand on some page of the console, so a job that
# misbehaves can be taken apart and each step tried on its own.
# The joint acceleration every joint_move is commanded with, rad/s^2. It is a
# module constant rather than a default buried in the move call because the
# excursion a swept speed needs is DERIVED from it: reaching v and holding it
# for T takes v^2/a + v*T radians. Change it here and the sweep resizes itself.
JOINT_ACCEL = 1.2

STEP_KINDS = (
    "path", "ur_log_start", "ur_log_stop",
    "preflight",        # run the checks; fail the job if any check fails
    "dwell",            # wait, so a move can settle before a capture
    "move",             # go to one tool pose
    "trajectory",       # go through a list of tool poses
    "joint_move",       # drive ONE joint at a commanded angular velocity
    "record_start",     # begin a benchmark run file
    "record_stop",
    "imu_log_start",    # begin the continuous inertial CSV
    "imu_log_stop",
    "export",           # write the dataset folder
    "message",          # put a line in the job log; no side effect
)


# ---------------------------------------------------------------------------
# pre-flight
# ---------------------------------------------------------------------------

@dataclass
class Check:
    """One thing that has to be true before the cell is allowed to move."""
    key: str
    label: str
    state: str = "unknown"     # "pass" | "warn" | "fail" | "unknown"
    detail: str = ""
    blocking: bool = True      # a warn never blocks; a failed check blocks
                               # only when this is set

    def as_dict(self):
        return {"key": self.key, "label": self.label, "state": self.state,
                "detail": self.detail, "blocking": self.blocking}


def preflight(ctx, requires=None) -> dict:
    """
    Ask the cell whether it is fit to run, and say so in sentences.

    `ctx` is the bridge's capability bundle (see `Runner`), which is how this
    module stays testable: nothing here imports the bridge, opens a socket or
    touches hardware. Everything it knows, it is handed.

    Ordered the way a person would check: can we talk to the robot, will the
    robot accept commands, can we see, do we know where the camera is, are the
    sensors live, is there room to write.

    `requires` says what the job actually uses, and only those checks can
    block. This matters more than it looks. The benchmark's scored channels --
    orientation, angular rate, acceleration, and the tool pose the robot
    computes from its own joint angles -- involve no camera at all. Gating a
    capture campaign on the hand-eye calibration would hold up weeks of arm
    time waiting on a measurement none of those channels depend on. The
    calibration is still checked, and still reported, because a run recorded
    without it cannot later be used for anything in the camera's frame -- but
    for an inertial campaign that is a note, not a stop.
    """
    requires = set(requires or ("robot", "imu", "camera"))
    needs_cam = "camera" in requires
    checks: list[Check] = []

    # --- robot ---------------------------------------------------------
    st = ctx.robot_state() or {}
    health = ctx.robot_health() or {}
    rate = float(health.get("rate_hz") or 0.0)
    if not ctx.robot_enabled():
        checks.append(Check("robot_link", "Robot link", "fail",
                            "The robot link has not been started. Connect page "
                            "→ Robot link → Connect to the robot."))
    elif rate < 5.0:
        checks.append(Check("robot_link", "Robot link", "fail",
                            f"The link is open but only {rate:.0f} readings a "
                            "second are arriving. Check the cable and that no "
                            "other program holds the robot's RTDE connection."))
    else:
        checks.append(Check("robot_link", "Robot link", "pass",
                            f"{rate:.0f} readings a second from "
                            f"{ctx.robot_host()}."))

    # --- is the robot being read properly, or merely at all? ------------
    #
    # Blocking, and it is the check this file most needed. A two-minute arc
    # scan was recorded against the FALLBACK interface on port 30003 because
    # RTDE would not start and nothing said so: the console showed a healthy
    # link, pre-flight passed, the run file filled up. Afterwards the data
    # showed the robot state refreshing for five seconds in every nineteen --
    # 88% of the rows were a repeat of a stale state, joints jumped up to 52
    # degrees when it caught up, and only 16% of the run carried anything
    # fresh. Every one of those numbers was invisible while it was happening.
    #
    # "Connected" was never the right question. WHICH INTERFACE is.
    health = ctx.robot_health() or {}
    degraded = health.get("degraded") or ""
    if ctx.robot_enabled() and degraded:
        checks.append(Check("robot_interface", "Robot interface", "fail",
                            (degraded[:1].upper() + degraded[1:])
                            + ". RTDE is the only interface that carries the "
                              "full field set at a steady rate; the fallback "
                              "gives a fraction of the fields and, on this "
                              "cell, five seconds of data in every nineteen. "
                              "Recording a campaign on it produces run files "
                              "that look complete and are mostly one stale "
                              "state repeated. Check that nothing else holds "
                              "port 30004 — another copy of this agent, or a "
                              "program left running — then reconnect.",
                            blocking="robot" in requires))
    elif ctx.robot_enabled():
        nf = len(health.get("fields") or [])
        dropped = health.get("dropped_fields") or []
        checks.append(Check("robot_interface", "Robot interface", "pass",
                            f"RTDE on port 30004, {nf} fields"
                            + (f" ({len(dropped)} the controller does not have)"
                               if dropped else "") + "."))

    mode = str(st.get("robot_mode_text") or "").upper()
    safety = str(st.get("safety_mode_text") or "").upper()
    if mode and mode not in ("RUNNING", "IDLE"):
        checks.append(Check("robot_mode", "Robot powered", "fail",
                            f"The robot is {mode.title()}. Power it on and "
                            "release the brakes on the Robot page."))
    elif mode == "IDLE":
        checks.append(Check("robot_mode", "Robot powered", "fail",
                            "The brakes are still on. Release them on the "
                            "Robot page before a job moves the arm."))
    elif mode:
        checks.append(Check("robot_mode", "Robot powered", "pass", "Running."))
    else:
        checks.append(Check("robot_mode", "Robot powered", "warn",
                            "The robot has not reported a mode yet."))

    if safety and safety != "NORMAL":
        checks.append(Check("safety", "Safety state", "fail",
                            f"Safety state is {safety.title()}. Clear it on "
                            "the pendant; a job will not move the arm while "
                            "the robot is in a protective or emergency state."))
    elif safety:
        checks.append(Check("safety", "Safety state", "pass", "Normal."))
    else:
        checks.append(Check("safety", "Safety state", "warn",
                            "No safety state reported yet."))

    # Remote control is not readable over RTDE on every controller, so this is
    # a warning rather than a gate — but it is the commonest reason a job's
    # first move silently does nothing, so it is always shown.
    checks.append(Check("remote", "Remote control", "warn",
                        "Cannot be read back from the controller. If the first "
                        "move does nothing, the pendant is in Local mode.",
                        blocking=False))

    # --- camera --------------------------------------------------------
    cam_age = ctx.camera_age_s()
    if cam_age is None:
        checks.append(Check("camera", "Camera",
                            "fail" if needs_cam else "warn",
                            "No camera. Steps that need a picture will fail; "
                            "motion and inertial capture will not.",
                            blocking=needs_cam))
    elif cam_age > 2.0:
        checks.append(Check("camera", "Camera",
                            "fail" if needs_cam else "warn",
                            f"The last picture arrived {cam_age:.0f} s ago.",
                            blocking=needs_cam))
    else:
        checks.append(Check("camera", "Camera", "pass", "Streaming."))

    # --- calibration ---------------------------------------------------
    cal = ctx.calibration()
    if not cal:
        checks.append(Check("calibration", "Camera position known",
                            "fail" if needs_cam else "warn",
                            "No hand-eye calibration is loaded, so nothing the "
                            "camera sees can be placed in the robot's frame. "
                            + ("Run it on the Calibrate page, or load the "
                               "saved one." if needs_cam else
                               "This job does not use the camera, so it can "
                               "run — but these runs will carry no usable "
                               "camera geometry."),
                            blocking=needs_cam))
    else:
        spread = cal.get("target_spread_mm")
        if spread is not None and spread > 5.0:
            checks.append(Check("calibration", "Camera position known",
                                "fail" if needs_cam else "warn",
                                f"The loaded calibration reconstructs the board "
                                f"to {spread:.1f} mm, which is too loose to "
                                "build a dataset on. Run it again.",
                                blocking=needs_cam))
        else:
            checks.append(Check("calibration", "Camera position known", "pass",
                                f"{cal.get('calib_version', 'loaded')}"
                                + (f", good to {spread:.1f} mm" if spread is not None
                                   else "")))

    # --- inertial ------------------------------------------------------
    units = ctx.imu_status() or {}
    live = {u: v for u, v in units.items()
            if (v.get("age_s") is None or v["age_s"] < 2.0)
            and (v.get("rate_hz") or 0) > 1}
    if not live:
        checks.append(Check("imu", "Motion sensors", "fail",
                            "No inertial unit is streaming. Connect one on the "
                            "Sensors page — it is the channel the benchmark is "
                            "scored on."))
    else:
        worst = min((v.get("rate_hz") or 0) for v in live.values())
        checks.append(Check("imu", "Motion sensors", "pass",
                            f"{len(live)} streaming, slowest {worst:.0f} Hz."))

    # --- do we know what the numbers MEAN? -----------------------------
    #
    # A unit can be streaming perfectly and still be unreadable. Two things
    # about an inertial stream are not self-evident and are therefore measured
    # from the data rather than assumed, and until each is settled that unit's
    # angular rate and orientation must not be recorded:
    #
    #   the gyroscope's units    degrees and radians are the same numbers 57x
    #                            apart, and an LPMS-B2 through FusionHub
    #                            publishes degrees where this console's own
    #                            columns say radians;
    #   the quaternion's sense   published the other way round it still looks
    #                            healthy, but pitch changes sign and taking
    #                            gravity out of the accelerometer leaves 1.9
    #                            m/s^2 of fiction behind on a still arm.
    #
    # Both are settled by MOVING THE ARM: a slow elbow sweep gives each test
    # the rotation it needs. That is why this is a gate and not a note — the
    # fix takes ten seconds before the campaign and is unrecoverable after it.
    pending_units, pending_quat = [], []
    for u, v in live.items():
        if str(v.get("gyro_units") or "") not in ("deg", "rad"):
            pending_units.append(u)
        if v.get("quat") and str(v.get("quat_convention") or "") not in \
                ("direct", "conjugate"):
            pending_quat.append(u)
    if pending_units or pending_quat:
        bits = []
        if pending_units:
            bits.append("the gyroscope's units are not established on "
                        + ", ".join(pending_units))
        if pending_quat:
            bits.append("which way round the quaternion is published is not "
                        "established on " + ", ".join(pending_quat))
        checks.append(Check("imu_units", "Sensor scales established", "fail",
                            "; ".join(bits).capitalize()
                            + ". Run the \"settle_sensors\" job — it sweeps "
                              "the elbow twenty degrees and back, which is all "
                              "either measurement needs — or jog the elbow by "
                              "hand, then run pre-flight again. Recording "
                              "before this is settled writes angular rates "
                              "that may be 57x wrong and an orientation that "
                              "may be inverted.",
                            blocking="imu" in requires))
    else:
        detail = []
        for u, v in live.items():
            unit_word = ("degrees per second" if v.get("gyro_units") == "deg"
                         else "radians per second")
            detail.append(f"{u} publishes {unit_word}")
        checks.append(Check("imu_units", "Sensor scales established", "pass",
                            "; ".join(detail) + " — measured, not assumed."))

    # --- what is on the end of the arm? --------------------------------
    #
    # Blocking, and blocking on "has anybody answered" rather than on "is it
    # non-zero" -- a bare flange is a perfectly good answer. What cannot be
    # allowed is the default zero, which produces a simulated arm carrying
    # nothing against a real arm carrying a sensor, a bracket and a cable, and
    # charges the difference to the sim-to-real gap.
    car = ctx.carrier() or {}
    if not car.get("measured"):
        checks.append(Check("carrier", "Carrier described", "fail",
                            "Nobody has said what is bolted to the flange. The "
                            "simulated arm's payload is built from this, so a "
                            "run recorded without it gets compared against a "
                            "simulation carrying nothing — and that difference "
                            "is indistinguishable from the sim-to-real gap "
                            "afterwards. Weigh the carrier and enter it on the "
                            "Automate page. Zero is a fine answer if the flange "
                            "really is bare.",
                            blocking="imu" in requires or "robot" in requires))
    else:
        checks.append(Check("carrier", "Carrier described", "pass",
                            _carrier_words(car)))

    # --- does one g read as one g, whichever way up? --------------------
    #
    # A warning rather than a gate. An uncalibrated accelerometer still gives
    # usable orientation and angular rate; what it corrupts is acceleration and
    # the gravity removal that produces `linear_accel`, and it corrupts them as
    # a function of POSE -- which looks like a pose-dependent sim-to-real gap
    # and survives averaging. Worth stopping a campaign for once it is known,
    # not worth blocking a bring-up over.
    worst = None
    for u, v in live.items():
        sp = v.get("accel_scale_spread_pct")
        if isinstance(sp, (int, float)) and (worst is None or sp > worst[1]):
            worst = (u, sp, bool(v.get("accel_calibrated")))
    if worst and worst[1] > 3.0:
        checks.append(Check("accel_scale", "Accelerometer scale", "warn",
                            f"On {worst[0]}, one g reads over a range of "
                            f"{worst[1]:.1f}% of g depending on which way up "
                            f"the carrier is"
                            + (" — and it is already calibrated, so the "
                               "calibration no longer fits; redo it"
                               if worst[2] else
                               ", which is a per-axis scale error and is not "
                               "calibrated out") +
                            ". It leaks gravity into the acceleration channel "
                            "as a function of pose, which is indistinguishable "
                            "from a pose-dependent gap afterwards. Run the "
                            "six-face calibration: rest the carrier on each of "
                            "its six sides for a few seconds each.",
                            blocking=False))
    elif worst:
        checks.append(Check("accel_scale", "Accelerometer scale", "pass",
                            f"one g reads to within {worst[1]:.1f}% of g "
                            f"whichever way up {worst[0]} is"
                            + (", calibrated" if worst[2] else "")))

    # --- is time real on every channel? --------------------------------
    clk = ctx.clock_status() or {}
    stalled = [c for c, b in (clk.get("channels") or {}).items()
               if b.get("clock") == "stalled"]
    if stalled:
        checks.append(Check("clock", "Sample timing", "warn",
                            "These channels publish a timestamp that never "
                            "advances, so their samples are timed by arrival at "
                            "the host: " + ", ".join(stalled)
                            + ". The average rate is sound and every channel "
                              "shares one timeline, but per-sample spacing "
                              "carries the transport's jitter.",
                            blocking=False))
    else:
        checks.append(Check("clock", "Sample timing", "pass",
                            "Every channel's timing is usable."))

    # --- somewhere to put it -------------------------------------------
    try:
        free_gb = shutil.disk_usage(ctx.out_dir()).free / 1e9
        if free_gb < 1.0:
            checks.append(Check("disk", "Disk space", "fail",
                                f"{free_gb:.1f} GB free where runs are written. "
                                "A job that fills the disk mid-run leaves a "
                                "truncated file that looks complete."))
        elif free_gb < 10.0:
            checks.append(Check("disk", "Disk space", "warn",
                                f"{free_gb:.1f} GB free.", blocking=False))
        else:
            checks.append(Check("disk", "Disk space", "pass",
                                f"{free_gb:.0f} GB free."))
    except Exception as e:      # noqa: BLE001
        checks.append(Check("disk", "Disk space", "warn", str(e), blocking=False))

    blocking = [c for c in checks if c.state == "fail" and c.blocking]
    return {
        "ok": not blocking,
        "checks": [c.as_dict() for c in checks],
        "blocking": [c.key for c in blocking],
        "summary": ("Ready." if not blocking else
                    f"{len(blocking)} thing{'s' if len(blocking) > 1 else ''} "
                    "must be fixed before the cell can run a job."),
    }


# ---------------------------------------------------------------------------
# jobs
# ---------------------------------------------------------------------------

@dataclass
class Job:
    """
    A named sequence, optionally repeated with one parameter swept.

    The sweep is the point. A benchmark campaign is "the same motion at four
    speeds, three times each"; writing that as twelve hand-built jobs is how
    two of the twelve end up subtly different.
    """
    name: str = "capture"
    steps: list = field(default_factory=list)
    repeats: int = 1
    sweep_key: str = ""                 # e.g. "joint_vel"
    sweep_values: list = field(default_factory=list)
    notes: str = ""
    # What this job actually uses. Only these can block it.
    requires: list = field(default_factory=lambda: ["robot", "imu"])

    def expand(self) -> list:
        """The flat list of (iteration, params, step) this job will execute."""
        values = self.sweep_values or [None]
        plan = []
        idx = 0
        for value in values:
            for rep in range(max(1, int(self.repeats))):
                params = {"repeat_idx": rep, "iteration": idx}
                if self.sweep_key and value is not None:
                    params[self.sweep_key] = value
                for step in self.steps:
                    plan.append((idx, dict(params), dict(step)))
                idx += 1
        return plan

    def as_dict(self):
        return {"name": self.name, "steps": self.steps, "repeats": self.repeats,
                "sweep_key": self.sweep_key, "sweep_values": self.sweep_values,
                "notes": self.notes, "requires": list(self.requires)}


class Runner:
    """
    Executes one job on a background thread and reports where it is.

    `ctx` carries every capability this needs as a plain callable, so the
    runner can be exercised with no robot, no camera and no sensors attached —
    which is the only way the stop path and the failure path get tested at
    all. A runner that has only ever been tried against real hardware has
    never had its abort tested.
    """

    def __init__(self, ctx):
        self.ctx = ctx
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.job: Job | None = None
        self.log: list = []
        self.state = "idle"        # idle | running | stopping | done | failed
        self.step_i = 0
        self.step_n = 0
        self.current = ""
        self.started_at = 0.0
        self.finished_at = 0.0
        self.error = ""
        self.produced: list = []   # files this job created

    # -- reporting -------------------------------------------------------
    def _say(self, text: str, level: str = "info") -> None:
        entry = {"t": round(time.time(), 2), "level": level, "text": text}
        with self._lock:
            self.log.append(entry)
            if len(self.log) > 400:
                del self.log[:100]
        log.info("[job] %s", text)

    def status(self) -> dict:
        with self._lock:
            return {
                "state": self.state,
                "job": self.job.name if self.job else "",
                "step": self.step_i,
                "steps": self.step_n,
                # How many steps make one pass, so the page can highlight the
                # live step without having to guess from whichever job happens
                # to be selected in the dropdown -- which is not necessarily
                # the one running.
                "steps_per_pass": len(self.job.steps) if self.job else 0,
                "current": self.current,
                "error": self.error,
                "seconds": round((self.finished_at or time.time()) - self.started_at, 1)
                if self.started_at else 0.0,
                "log": self.log[-40:],
                "produced": list(self.produced),
                "running": self.state in ("running", "stopping"),
            }

    # -- control ---------------------------------------------------------
    def start(self, job: Job) -> dict:
        if self.state in ("running", "stopping"):
            return {"ok": False, "error": "a job is already running"}
        if not job.steps:
            return {"ok": False, "error": "this job has no steps"}
        bad = [s.get("kind") for s in job.steps if s.get("kind") not in STEP_KINDS]
        if bad:
            return {"ok": False, "error": f"unknown step: {bad[0]}"}

        # The gate. Checked here, before a thread exists, so a refusal is
        # immediate and carries the reason rather than appearing as a job that
        # started and died.
        pre = preflight(self.ctx, job.requires)
        if not pre["ok"]:
            return {"ok": False, "error": pre["summary"], "preflight": pre}

        with self._lock:
            self.job = job
            self.log = []
            self.produced = []
            self.state = "running"
            self.error = ""
            self.step_i = 0
            self.started_at = time.time()
            self.finished_at = 0.0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(job,),
                                        daemon=True, name="sonair-job")
        self._thread.start()
        return {"ok": True, "job": job.name, **self.status()}

    def stop(self) -> dict:
        if self.state not in ("running",):
            return {"ok": False, "error": "no job is running"}
        with self._lock:
            self.state = "stopping"
        self._stop.set()
        self._say("Stop requested — finishing the current step and closing "
                  "any open recording.", "warn")
        return {"ok": True, **self.status()}

    # -- execution -------------------------------------------------------
    def _run(self, job: Job) -> None:
        plan = job.expand()
        with self._lock:
            self.step_n = len(plan)
        self._say(f"{job.name}: {len(plan)} steps"
                  + (f", sweeping {job.sweep_key} over {job.sweep_values}"
                     if job.sweep_key else ""))

        # START FROM A KNOWN STATE.
        #
        # A recording left open by anything -- a run stopped from the Record
        # page, a previous job whose process was killed, an agent restarted
        # mid-capture -- makes record_start refuse, and the job dies three
        # steps in with "already recording". The runner cleaned up after
        # itself but never before itself, so one interrupted run poisoned
        # every job after it until somebody found the Stop button on another
        # page. Closing here costs nothing when there is nothing to close.
        self._clear_stale()

        failed = ""
        try:
            for i, (iteration, params, step) in enumerate(plan):
                if self._stop.is_set():
                    self._say("Stopped by the operator.", "warn")
                    break
                with self._lock:
                    self.step_i = i + 1
                    self.current = step.get("kind", "?")
                ok, detail = self._do(step, params, iteration, job)
                if not ok:
                    failed = f"{step.get('kind')}: {detail}"
                    self._say(f"FAILED at step {i+1} — {failed}", "bad")
                    break
        except Exception as e:      # noqa: BLE001
            failed = f"{type(e).__name__}: {e}"
            self._say(f"The job hit an unexpected error: {failed}", "bad")
        finally:
            # Whatever happened, nothing is left open. A run file still being
            # written when a job dies is the file that looks complete and is
            # not.
            self._close_everything()
            with self._lock:
                self.finished_at = time.time()
                self.error = failed
                self.state = "failed" if failed else "done"
                self.current = ""
            self._say("Finished." if not failed else "Stopped after a failure.",
                      "ok" if not failed else "bad")

    def _clear_stale(self) -> None:
        """Close anything a previous run left open, and say so if it did."""
        try:
            if self.ctx.is_recording():
                res = self.ctx.record_stop()
                self._say("A recording was still open from an earlier run and "
                          "has been closed: " + str(res.get("path", "")) +
                          ". It is kept; nothing was discarded.", "warn")
        except Exception as e:      # noqa: BLE001
            self._say(f"Could not close the earlier recording: {e}", "bad")
        try:
            if self.ctx.imu_logging():
                res = self.ctx.imu_log_stop()
                if res.get("path"):
                    self._say("An inertial log was still open and has been "
                              "closed: " + str(res["path"]), "warn")
        except Exception as e:      # noqa: BLE001
            self._say(f"Could not close the earlier inertial log: {e}", "bad")
        try:
            if self.ctx.ur_logging():
                res = self.ctx.ur_log_stop()
                if res.get("path"):
                    self._say("A robot log was still open and has been closed: "
                              + str(res["path"]), "warn")
        except Exception as e:      # noqa: BLE001
            self._say(f"Could not close the earlier robot log: {e}", "bad")

    def _close_everything(self) -> None:
        try:
            if self.ctx.is_recording():
                res = self.ctx.record_stop()
                self._say(f"Closed the run file: {res.get('path', '')}", "warn")
        except Exception as e:      # noqa: BLE001
            self._say(f"Could not close the run file: {e}", "bad")
        try:
            if self.ctx.imu_logging():
                res = self.ctx.imu_log_stop()
                if res.get("path"):
                    self.produced.append(res["path"])
                self._say(f"Closed the inertial log: {res.get('path', '')}", "warn")
        except Exception as e:      # noqa: BLE001
            self._say(f"Could not close the inertial log: {e}", "bad")
        try:
            if self.ctx.ur_logging():
                res = self.ctx.ur_log_stop()
                if res.get("path"):
                    self.produced.append(res["path"])
                self._say(f"Closed the robot log: {res.get('path', '')}", "warn")
        except Exception as e:      # noqa: BLE001
            self._say(f"Could not close the robot log: {e}", "bad")
        try:
            self.ctx.halt()
        except Exception:
            pass

    def _do(self, step: dict, params: dict, iteration: int, job: Job):
        kind = step.get("kind")

        if kind == "message":
            self._say(str(step.get("text", "")))
            return True, ""

        if kind == "dwell":
            secs = float(step.get("seconds", 1.0))
            self._say(f"Waiting {secs:g} s.")
            # Interruptible: a 30 s settle must not make Stop take 30 s.
            end = time.monotonic() + secs
            while time.monotonic() < end:
                if self._stop.is_set():
                    return True, ""
                time.sleep(0.05)
            return True, ""

        if kind == "preflight":
            pre = preflight(self.ctx, job.requires)
            for c in pre["checks"]:
                if c["state"] != "pass":
                    self._say(f"{c['label']}: {c['detail']}",
                              "bad" if c["state"] == "fail" else "warn")
            if not pre["ok"]:
                return False, pre["summary"]
            self._say("Pre-flight passed.", "ok")
            return True, ""

        if kind == "move":
            pose = step.get("pose")
            if not pose or len(pose) < 6:
                return False, "no pose given"
            speed = float(step.get("speed", 0.12))
            self._say(f"Moving to [{', '.join(f'{v:.3f}' for v in pose[:3])}].")
            ok, why = self.ctx.move_to(pose, speed)
            if not ok:
                return False, why
            return self._await_arrival(pose, step)

        if kind == "path":
            # ONE CONTINUOUS PATH, not a queue of moves.
            #
            # `trajectory` sends a movel per waypoint and waits for each to
            # land within a couple of millimetres, which means the arm comes to
            # a complete stop at every one. For a scan of two hundred waypoints
            # that is wrong twice: it takes several times as long, and the
            # sensors see two hundred start-stop transients instead of the
            # smooth sweep a real inspection traces. Here the whole path goes
            # to the controller as a single blended program and the arm never
            # stops until the end.
            poses = step.get("poses") or []
            if not poses:
                return False, "no waypoints given"
            speed = float(step.get("speed", 0.045))
            cap = self.ctx.max_linear_speed()
            if cap and speed > cap + 1e-9:
                return False, (f"{speed:g} m/s is above this cell's linear "
                               f"speed limit of {cap:g} m/s; the arm would be "
                               "held at the limit while the run recorded the "
                               "higher figure.")
            blend = float(step.get("blend_m", 0.004))
            shape = scan_paths.describe(poses, speed) if scan_paths else {}
            self._say(
                f"Scanning: {len(poses)} waypoints, "
                f"{shape.get('path_length_m', 0):.2f} m of tool travel at "
                f"{speed:g} m/s, about {shape.get('duration_s', 0):.0f} s. The "
                f"tool stays normal to the part, so it also turns through "
                f"{shape.get('total_rotation_deg', 0):.0f}° — about "
                f"{shape.get('mean_angular_rate_deg_s', 0):.1f}°/s, which is "
                f"what makes this worth recording.")
            ok, why = self.ctx.move_path(poses, speed, blend)
            if not ok:
                return False, why
            # Generous, and derived: a path that has to travel two metres
            # cannot be judged stalled on the same timeout as a single move.
            budget = float(step.get("timeout_s", 0)) or \
                max(30.0, 3.0 * float(shape.get("duration_s", 30.0)) + 15.0)
            ok, why = self._await_arrival(
                poses[-1], {**step, "timeout_s": budget,
                            "tolerance_mm": step.get("tolerance_mm", 6.0)})
            if not ok:
                return False, why
            return True, ""

        if kind == "ur_log_start":
            res = self.ctx.ur_log_start(step.get("path"))
            if not res.get("ok"):
                return False, res.get("error", "could not start the robot log")
            self._say(f"Logging every packet the robot sends to "
                      f"{res.get('path', '')} — {res.get('columns', 0)} columns.",
                      "ok")
            return True, ""

        if kind == "ur_log_stop":
            if not self.ctx.ur_logging():
                return True, ""
            res = self.ctx.ur_log_stop()
            if not res.get("ok"):
                return False, res.get("error", "could not close the robot log")
            self._say(f"Robot log closed: {res.get('rows', 0)} rows at "
                      f"{res.get('rate_hz', 0):.0f} a second.", "ok")
            return True, ""

        if kind == "trajectory":
            poses = step.get("poses") or []
            if not poses:
                return False, "no poses given"
            # Deliberately NOT params["joint_vel"]: that is an angular
            # velocity in rad/s and this is a linear speed in m/s. Feeding one
            # into the other is what mislabelled the campaign.
            speed = float(step.get("speed", 0.12))
            cap = self.ctx.max_linear_speed()
            if cap and speed > cap + 1e-9:
                return False, (f"{speed:g} m/s is above this cell's linear "
                               f"speed limit of {cap:g} m/s; the arm would be "
                               "held at the limit while the run recorded the "
                               "higher figure.")
            self._say(f"Running {len(poses)} poses at {speed:g} m/s "
                      "(tool-space linear speed).")
            for j, pose in enumerate(poses):
                if self._stop.is_set():
                    return True, ""
                ok, why = self.ctx.move_to(pose, speed)
                if not ok:
                    return False, f"pose {j+1}: {why}"
                ok, why = self._await_arrival(pose, step)
                if not ok:
                    return False, f"pose {j+1}: {why}"
            return True, ""

        if kind == "joint_move":
            # THE CAMPAIGN'S SWEEP FACTOR, COMMANDED AS WHAT IT IS.
            #
            # `joint_vel` in the run manifest is documented as the commanded
            # ELBOW ANGULAR VELOCITY in rad/s, and it is half of the condition
            # cell every result is grouped by. Driving a tool-space move and
            # writing its linear speed into that field labelled 0.4 m/s as
            # 0.4 rad/s -- so the sweep never crossed the ~0.6 rad/s region the
            # whole design is built around, and every cell key in the dataset
            # named a quantity that had not been commanded. Nothing would have
            # shown it; the runs look perfectly good.
            #
            # A single joint driven through a fixed excursion at a commanded
            # angular velocity is exactly the quantity, exactly reproducible,
            # and bounded enough to repeat unattended.
            q0 = self.ctx.joints()
            if not q0 or len(q0) < 6:
                return False, ("the robot is not reporting joint angles, so a "
                               "joint move cannot be commanded")
            idx = int(step.get("joint", 2))          # 0-based; 2 is the elbow
            if not 0 <= idx < 6:
                return False, f"joint index {idx} is not in 0..5"
            speed = float(params.get("joint_vel", step.get("joint_vel", 0.4)))
            if speed <= 0:
                return False, "joint speed must be positive"

            # HOW FAR THE JOINT HAS TO TRAVEL TO ACTUALLY REACH THAT SPEED.
            #
            # A fixed excursion is the wrong thing to hold constant across a
            # speed sweep, and the cell's own data showed why. At 25 deg and
            # this controller's 1.2 rad/s^2, the elbow held 0.2 rad/s for two
            # full seconds, held 0.4 for under a second, never settled at 0.6 at
            # all -- its rate profile ran 0.49, 0.51, 0.65, 0.56, 0.65, 0.47,
            # ringing rather than cruising -- and at 0.9 could not have got
            # there at any point, because a 25 deg move under that acceleration
            # peaks at 0.72 rad/s and then has to start braking.
            #
            # So two of the campaign's four cells would have been labelled with
            # a speed the arm never reached, and the third barely. The factor
            # the whole design sweeps would not have spanned what its own cell
            # keys claimed, Gate C would have been asked whether the gap varies
            # with a condition that hardly varied, and nothing downstream could
            # have detected any of it -- the runs look perfectly good.
            #
            # The excursion is therefore DERIVED from the speed so that every
            # cell gets a cruise of the same duration: ramp up, hold for
            # `plateau_s`, ramp down. What is held constant across the sweep is
            # the thing that should be -- how long the joint spends at the speed
            # the cell is named after.
            accel = float(step.get("joint_accel", JOINT_ACCEL))
            plateau = float(step.get("plateau_s", 1.0))
            explicit = step.get("amplitude_deg")
            if explicit is not None:
                amp = math.radians(float(explicit))
                held = (abs(amp) - speed * speed / accel) / speed
                # 0.3 s, because the cell's own 0.6 rad/s runs computed to a
                # 0.23 s cruise and were measured never to settle -- their rate
                # profile rang between 0.47 and 0.65 rad/s instead of holding.
                # Anything shorter is a ramp with a corner on it, not a cell.
                if held < 0.3:
                    reach = math.sqrt(accel * abs(amp))
                    return False, (
                        f"a {math.degrees(abs(amp)):.0f} deg move at "
                        f"{accel:g} rad/s^2 cannot hold {speed:g} rad/s: it "
                        + (f"only reaches {reach:.2f} rad/s before it has to "
                           "brake" if reach < speed else
                           f"holds it for {max(held, 0):.2f} s, which is not a "
                           "cruise") +
                        f". This run would be labelled {speed:g} rad/s and would "
                        f"not be that. Use at least "
                        f"{math.degrees(speed * speed / accel + speed * 0.5):.0f} "
                        f"deg, or leave amplitude_deg out and let it be sized "
                        f"from the speed.")
            else:
                amp = speed * speed / accel + speed * plateau
                sign = 1.0 if float(step.get("direction", 1)) >= 0 else -1.0
                amp *= sign
            span = float(step.get("max_amplitude_deg", 150.0))
            if abs(math.degrees(amp)) > span:
                return False, (
                    f"reaching {speed:g} rad/s for {plateau:g} s needs "
                    f"{abs(math.degrees(amp)):.0f} deg of joint {idx + 1}, past "
                    f"the {span:g} deg this step allows. Shorten plateau_s, drop "
                    "the top speed from the sweep, or raise max_amplitude_deg "
                    "once you have checked the arm has room.")
            # The cell clamps a speed it considers unsafe, silently. That is
            # correct for a jog and wrong for a swept factor: the run file
            # would record the speed that was ASKED for while the arm moved at
            # the limit, so two cells of the campaign would hold identical
            # motion under different labels.
            cap = self.ctx.max_joint_speed()
            if cap and speed > cap + 1e-9:
                return False, (f"{speed:g} rad/s is above this cell's joint "
                               f"speed limit of {cap:g} rad/s. The robot would "
                               "be held at the limit while the run file "
                               "recorded the higher figure, so the two would "
                               "disagree. Raise the limit deliberately or "
                               "sweep within it.")
            held = (abs(amp) - speed * speed / accel) / speed
            self._say(f"Joint {idx + 1} through {math.degrees(amp):.0f} deg at "
                      f"{speed:g} rad/s — ramping for "
                      f"{speed / accel:.2f} s each end and holding the speed "
                      f"for {held:.2f} s.")
            for leg, delta in (("out", +amp), ("back", 0.0)):
                if self._stop.is_set():
                    return True, ""
                target = list(q0)
                target[idx] = q0[idx] + delta
                ok, why = self.ctx.move_joints(target, speed)
                if not ok:
                    return False, f"{leg}: {why}"
                ok, why = self._await_joints(target, step)
                if not ok:
                    return False, f"{leg}: {why}"
            return True, ""

        if kind == "record_start":
            run_id = self._run_id(step, params, iteration, job)
            args = {
                "run_id": run_id,
                # rad/s at the elbow. From the sweep when the job sweeps it,
                # from the step otherwise. Never from a linear speed.
                "joint_vel": float(params.get("joint_vel",
                                              step.get("joint_vel", 0.4))),
                "arm_config": step.get("arm_config", "mid_workspace"),
                "traj_type": step.get("traj_type", "contour"),
                "repeat_idx": int(params.get("repeat_idx", 0)),
                # "none" rather than a plausible-looking version string. The
                # inertial channels do not need a hand-eye transform, so these
                # runs are valid -- but a run that names a calibration it was
                # not taken under is one nobody can later separate out.
                "calib_version": (self.ctx.calibration() or {}).get(
                    "calib_version") or "none",
                "rate_hz": float(step.get("rate_hz", 125.0)),
                "operator": step.get("operator", "automation"),
                "notes": job.notes,
            }
            # What is bolted to the flange goes in every run, because the
            # simulated twin's payload is built from it. A run stamped 0 kg
            # gets a simulated bare flange, and the difference lands in the
            # sim-to-real gap where nothing can separate it out later.
            car = self.ctx.carrier() or {}
            for k in ("carrier_id", "carrier_mass_kg", "carrier_com_m"):
                if car.get(k) is not None:
                    args[k] = car[k]
            res = self.ctx.record_start(args)
            if not res.get("ok"):
                return False, res.get("error", "could not start the run file")
            self.produced.append(res.get("path", run_id))
            self._say(f"Recording {run_id}.", "ok")
            return True, ""

        if kind == "record_stop":
            if not self.ctx.is_recording():
                return True, ""
            res = self.ctx.record_stop()
            if not res.get("ok"):
                return False, res.get("error", "could not close the run file")
            rate = res.get("achieved_rate_hz")
            self._say(f"Saved {res.get('n', 0)} samples to "
                      f"{res.get('path', '')}"
                      + (f" at {rate:.0f} a second." if rate else "."), "ok")
            # A run that did not keep up is still a good run, but the person
            # who will later generate its simulated twin has to be told at
            # what rate, and the only moment they are certainly paying
            # attention is now.
            if res.get("rate_warning"):
                self._say(res["rate_warning"], "warn")
            if res.get("worst_gap_s", 0) > 0.25:
                self._say(f"The longest gap between samples in that run was "
                          f"{res['worst_gap_s']:.2f} s, over "
                          f"{res.get('skipped_intervals', 0)} interruptions.",
                          "warn")
            return True, ""

        if kind == "imu_log_start":
            res = self.ctx.imu_log_start(step.get("path"))
            if not res.get("ok"):
                return False, res.get("error", "could not start the inertial log")
            self._say(f"Logging every inertial sample to {res.get('path','')}.", "ok")
            return True, ""

        if kind == "imu_log_stop":
            if not self.ctx.imu_logging():
                return True, ""
            res = self.ctx.imu_log_stop()
            if res.get("path"):
                self.produced.append(res["path"])
            self._say(f"Inertial log closed: {res.get('rows', 0)} rows.", "ok")
            return True, ""

        if kind == "export":
            res = self.ctx.export_dataset(step.get("name") or job.name)
            if not res.get("ok"):
                return False, res.get("error", "export failed")
            self.produced.append(res["path"])
            self._say(f"Dataset written to {res['path']} "
                      f"({res.get('runs', 0)} runs).", "ok")
            return True, ""

        return False, f"step {kind!r} is not implemented"

    def _await_joints(self, target, step) -> tuple[bool, str]:
        """Wait until every joint is where it was sent, or say that it is not."""
        tol = math.radians(float(step.get("tolerance_deg", 0.5)))
        timeout = float(step.get("timeout_s", 30.0))
        end = time.monotonic() + timeout
        worst = None
        while time.monotonic() < end:
            if self._stop.is_set():
                return True, ""
            now = self.ctx.joints()
            if now and len(now) >= 6:
                worst = max(abs(now[i] - target[i]) for i in range(6))
                if worst <= tol:
                    return True, ""
            time.sleep(0.02)
        return False, (f"the arm did not reach the joint target within "
                       f"{timeout:g} s"
                       + (f" (worst joint still {math.degrees(worst):.1f} deg "
                          "away)" if worst is not None
                          else " \u2014 no joint angles are being reported")
                       + ". The commonest cause is the pendant being in Local "
                         "mode, where the robot accepts the connection and "
                         "ignores the command.")

    def _await_arrival(self, pose, step) -> tuple[bool, str]:
        """
        Wait until the tool is where it was sent, or say that it is not.

        A move command that returns "sent" is not a move that happened. Without
        this the next step captures at the previous pose, which produces a
        dataset whose poses are all one step out of date — consistent,
        plausible and wrong.
        """
        tol = float(step.get("tolerance_mm", 2.0)) / 1000.0
        timeout = float(step.get("timeout_s", 25.0))
        end = time.monotonic() + timeout
        last = None
        while time.monotonic() < end:
            if self._stop.is_set():
                return True, ""
            now = self.ctx.tcp_pose()
            if now and len(now) >= 3:
                last = math.dist(now[:3], list(pose)[:3])
                if last <= tol:
                    return True, ""
            time.sleep(0.05)
        return False, (f"the arm did not reach the pose within {timeout:g} s"
                       + (f" (still {last*1000:.0f} mm away)" if last is not None
                          else " — no position is being reported")
                       + ". The commonest cause is the pendant being in Local "
                         "mode, where the robot accepts the connection and "
                         "ignores the command.")

    @staticmethod
    def _run_id(step, params, iteration, job) -> str:
        base = step.get("run_id") or job.name.replace(" ", "_")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return f"{base}_{stamp}_i{iteration}_r{params.get('repeat_idx', 0)}"


# ---------------------------------------------------------------------------
# built-in jobs
# ---------------------------------------------------------------------------

def builtin_jobs(tcp_pose=None, arm_config="mid_workspace") -> dict:
    """
    Jobs that are useful on day one, written against whatever pose the arm is
    in so they can be run without anybody typing coordinates.

    They are templates, not fixtures: the console shows the steps and the
    operator can change them before pressing Run.

    `arm_config` is carried rather than defaulted because it is one of the
    three factors the benchmark groups results by, and a run labelled
    "mid_workspace" that was actually taken with the arm extended is a
    mislabelled cell that nothing downstream can detect.
    """
    p = list(tcp_pose or [0.4, 0.0, 0.35, 0.0, 3.14, 0.0])

    def shifted(dx=0.0, dy=0.0, dz=0.0):
        q = list(p)
        q[0] += dx
        q[1] += dy
        q[2] += dz
        return q

    # A closed box in the plane the tool is already in, so the motion is
    # bounded and returns to where it started — which is what makes it safe to
    # repeat unattended. This is the INSPECTION-SHAPED motion; it is not what
    # the campaign sweeps (see below).
    square = [shifted(), shifted(dx=0.08), shifted(dx=0.08, dy=0.08),
              shifted(dy=0.08), shifted()]

    # THE ONE THAT IS WORTH RECORDING.
    #
    # Generated from a PART rather than from a box: a zig-zag over a curved
    # surface with the tool held normal to it, which is how a probe, a camera
    # at fixed standoff or an ultrasonic wheel is actually carried. Sweeping
    # across the curve therefore rotates the tool by the part's own curvature
    # -- continuously, and for free, because it is what the job requires rather
    # than a wiggle added to give the sensors something to look at.
    #
    # The flat box above was measured on the real cell: 0.9 degrees of rotation
    # in 46 seconds, linear acceleration flat on the sensor's noise floor.
    # Orientation and angular rate are two of the three channels the benchmark
    # scores, so that run carried nothing to score. This one turns the tool
    # roughly 14 degrees a second, which is over a hundred times the measured
    # noise floor, for about a minute.
    #
    # The part is assumed to sit a little in front of and below the tool's
    # current pose, with its axis along the robot's X. Nobody has to type
    # coordinates; if the real part is somewhere else, the poses are visible in
    # the step list and can be changed before pressing Run.
    arc = []
    if scan_paths is not None:
        try:
            arc = scan_paths.arc_zigzag(
                centre=(p[0], p[1] - 0.14, p[2] - 0.10),
                axis=(1.0, 0.0, 0.0),
                radius=0.12, standoff=0.05, arc_deg=70.0,
                length=0.18, passes=12, points_per_pass=16,
                start_offset=-0.09)
        except Exception as e:      # noqa: BLE001
            log.warning("could not build the arc scan path: %s", e)
            arc = []

    # The campaign's own factor. campaign.py defines the sweep over elbow
    # angular velocity in rad/s, bracketing ~0.6 rad/s where the simulator is
    # already known to change behaviour, so the motion that carries it has to
    # be commanded in joint space. A tool-space move at 0.4 m/s is not
    # 0.4 rad/s at the elbow and never was.
    ELBOW = 2

    return {
        "checkout": Job(
            name="checkout",
            notes="Prove the cell is fit before trusting a campaign to it. "
                  "Moves nothing.",
            requires=["robot", "imu"],
            steps=[
                {"kind": "preflight"},
                {"kind": "message", "text": "Everything the cell needs is present."},
            ],
        ),
        "settle_sensors": Job(
            name="settle_sensors",
            notes="Move the elbow gently so each inertial unit's scales can be "
                  "measured. Records nothing. Run it once after connecting the "
                  "sensors, before anything that records.",
            # Deliberately does NOT require "imu" for its gate: this is the job
            # whose whole purpose is to satisfy that gate, so gating it on the
            # gate would be a lock with its key inside. It still needs the
            # robot, because the measurement needs the arm to move.
            requires=["robot"],
            steps=[
                {"kind": "preflight"},
                {"kind": "message",
                 "text": "Sweeping the elbow slowly. Two things are being "
                         "measured from the data itself: whether each unit "
                         "reports turn rate in degrees or radians, and which "
                         "way round it publishes its orientation. Neither is "
                         "printed on the sensor and both are wrong by a lot if "
                         "assumed."},
                {"kind": "joint_move", "joint": ELBOW, "amplitude_deg": 20.0,
                 "joint_vel": 0.35},
                {"kind": "dwell", "seconds": 0.5},
                {"kind": "joint_move", "joint": ELBOW, "amplitude_deg": -20.0,
                 "joint_vel": 0.35},
                {"kind": "dwell", "seconds": 0.5},
                {"kind": "message",
                 "text": "Done. Run pre-flight again — Sensor scales "
                         "established should now pass, and it will say what "
                         "each unit was found to be reporting."},
            ],
        ),
        "single_run": Job(
            name="single_run",
            notes="One recorded pass at a single elbow speed, for checking a "
                  "change before committing to a campaign.",
            requires=["robot", "imu"],
            steps=[
                {"kind": "preflight"},
                {"kind": "imu_log_start"},
                {"kind": "record_start", "traj_type": "point_to_point",
                 "joint_vel": 0.4, "arm_config": arm_config},
                {"kind": "dwell", "seconds": 1.5},
                {"kind": "joint_move", "joint": ELBOW, "plateau_s": 1.0,
                 "joint_vel": 0.4},
                {"kind": "dwell", "seconds": 1.5},
                {"kind": "record_stop"},
                {"kind": "imu_log_stop"},
            ],
        ),
        "elbow_sweep": Job(
            name="elbow_sweep",
            notes="The benchmark campaign: the same elbow motion at a range of "
                  "commanded angular velocities, repeated, then exported as one "
                  "dataset. This is the factor campaign.py defines the cells by.",
            requires=["robot", "imu"],
            repeats=3,
            sweep_key="joint_vel",
            sweep_values=[0.2, 0.4, 0.6, 0.9],
            steps=[
                {"kind": "preflight"},
                {"kind": "imu_log_start"},
                {"kind": "record_start", "traj_type": "point_to_point",
                 "arm_config": arm_config},
                {"kind": "dwell", "seconds": 1.5},
                # No amplitude: it is sized from the swept speed so that every
                # cell holds the speed it is named after for the same second.
                # A fixed 25 deg gave the 0.2 cell a two second cruise, the 0.6
                # cell none at all, and the 0.9 cell a speed the arm could not
                # reach -- three cells that would have carried the same label
                # scheme and not the same experiment.
                {"kind": "joint_move", "joint": ELBOW, "plateau_s": 1.0},
                {"kind": "dwell", "seconds": 1.5},
                {"kind": "record_stop"},
                {"kind": "imu_log_stop"},
            ],
        ),
        "arc_scan": Job(
            name="arc_scan",
            notes="A minute of realistic inspection: a zig-zag over a curved "
                  "part with the tool held normal to the surface, so it turns "
                  "through the part's curvature as it sweeps. Records the "
                  "benchmark run, every inertial sample, AND every packet the "
                  "robot sends. This is the one to replay in a simulator.",
            requires=["robot", "imu"],
            steps=[
                {"kind": "preflight"},
                {"kind": "ur_log_start"},
                {"kind": "imu_log_start"},
                {"kind": "record_start", "traj_type": "contour",
                 "joint_vel": 0.0, "arm_config": arm_config,
                 "rate_hz": 125.0},
                {"kind": "dwell", "seconds": 2.0},
                {"kind": "path", "poses": arc, "speed": 0.045,
                 "blend_m": 0.004},
                {"kind": "dwell", "seconds": 2.0},
                {"kind": "record_stop"},
                {"kind": "imu_log_stop"},
                {"kind": "ur_log_stop"},
            ],
        ),
        "scan_shaped": Job(
            name="scan_shaped",
            notes="A tool-space box, the shape a real inspection scan traces. "
                  "For the INSPECTION case, not the benchmark: measured on the "
                  "real cell it turned the carrier 0.9 deg in 46 s and left "
                  "linear acceleration flat on the sensor's own noise floor, "
                  "so its inertial channels carry nothing to score. Record it "
                  "for the application story; do not put it in a gap.",
            requires=["robot", "imu"],
            steps=[
                {"kind": "preflight"},
                {"kind": "imu_log_start"},
                {"kind": "record_start", "traj_type": "contour",
                 "joint_vel": 0.4, "arm_config": arm_config},
                {"kind": "dwell", "seconds": 1.5},
                {"kind": "trajectory", "poses": square, "speed": 0.08},
                {"kind": "dwell", "seconds": 1.5},
                {"kind": "record_stop"},
                {"kind": "imu_log_stop"},
            ],
        ),
    }


# ---------------------------------------------------------------------------
# the dataset
# ---------------------------------------------------------------------------

DATASET_VERSION = "sonair-dataset/1"


def export_dataset(name: str, *, out_root, runs_dir, imu_dir, ctx,
                   ur_dir="ur_logs") -> dict:
    """
    Gather a campaign into one self-describing folder.

    A dataset that needs the person who made it to explain it is not a
    dataset. Everything needed to read these files later is written beside
    them: what the channels are and what they mean, which of them are scored
    and which are evidence, the calibration the geometry depends on, the
    camera intrinsics, the clock the timestamps are on, and the software that
    produced it.

    The manifest is the contract. Anything downstream -- the scorer, the
    Isaac replay, whatever multimodal model gets trained on this -- reads the
    manifest and never has to guess at a column.
    """
    out_root = Path(out_root)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    folder = out_root / f"{name}_{stamp}"
    try:
        (folder / "runs").mkdir(parents=True, exist_ok=True)
        (folder / "inertial").mkdir(parents=True, exist_ok=True)
        (folder / "robot").mkdir(parents=True, exist_ok=True)
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not create {folder}: {e}"}

    # Only what this campaign produced. Sweeping up everything in the runs
    # folder would quietly fold last week's runs into this week's dataset.
    since = ctx.job_started_at() or 0.0
    copied_runs, copied_imu, copied_ur = [], [], []
    for src, dest, bucket in (
        (Path(runs_dir), folder / "runs", copied_runs),
        (Path(imu_dir), folder / "inertial", copied_imu),
        (Path(ur_dir), folder / "robot", copied_ur),
    ):
        if not src.exists():
            continue
        for f in sorted(src.iterdir()):
            try:
                if f.is_file() and f.stat().st_mtime >= since - 1.0:
                    shutil.copy2(f, dest / f.name)
                    bucket.append(f.name)
            except Exception as e:      # noqa: BLE001
                log.warning("could not copy %s: %s", f, e)

    cal = ctx.calibration() or {}
    manifest = {
        "dataset_version": DATASET_VERSION,
        "name": name,
        "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "produced_by": "SONAIR inspection console",
        "cell": {
            "robot": "UR5e",
            "robot_host": ctx.robot_host(),
            # WHICH INTERFACE, and how well it was working. The manifest used
            # to say "UR5e" and stop, so a dataset recorded over the degraded
            # fallback interface was indistinguishable from a good one until
            # somebody analysed the files. Recorded here, with the reason RTDE
            # was not used when it was not.
            "robot_link": _robot_link(ctx),
            "camera": ctx.camera_info(),
            "inertial": ctx.imu_status(),
            # The payload the simulated twin has to be given. Recorded at the
            # dataset level as well as per run, so a reader who opens the
            # folder rather than a single file still finds it.
            "carrier": ctx.carrier(),
        },
        # The transform every 3D number in here depends on. Copied in rather
        # than referenced: a dataset that points at a calibration file on
        # somebody's laptop is a dataset that cannot be read next year.
        "hand_eye": cal or None,
        "clock": _clock_block(ctx),
        "channels": ctx.channel_registry(),
        "contents": {
            "runs": copied_runs,
            "inertial": copied_imu,
            "robot": copied_ur,
        },
        # Measured from the files, not copied from what was asked for. A run
        # file's own manifest records the rate the recorder was ASKED for; if
        # the cell could not keep up, that number is a claim the file does not
        # support, and the simulated side generated to match it will be
        # resampled against a grid the real side never used.
        "measured": _audit_runs(folder / "runs"),
        "reading_it": {
            "runs/*.jsonl": (
                "One benchmark run each. First line is the run manifest "
                "(parameters, calibration version, sample rate); every line "
                "after it is one sample on a fixed time grid."),
            "robot/*.csv": (
                "Every packet the controller sent, at its own rate, one row "
                "per packet -- joint angles, velocities, currents, torques and "
                "TEMPERATURES, the tool pose, speed and force, the wrist "
                "accelerometer, voltages, safety and runtime state. A fixed "
                "column set derived from the RTDE recipe, so a field this "
                "controller does not provide is a present, empty column rather "
                "than a missing one. This is the complete record; the run "
                "files hold the subset the gap is scored on, on the "
                "benchmark's fixed sample grid."),
            "inertial/*.csv": (
                "Every inertial sample at the sensor's own rate, one row per "
                "reading, fixed column set. Columns a unit does not provide "
                "are present and empty."),
            "timestamps": (
                "Every file in here is stamped on ONE clock: the host "
                "monotonic clock, seconds since the agent started. "
                "A channel is allowed to place its own samples using its own "
                "timestamps only once those have been shown to advance AND an "
                "offset onto the host clock has been measured for it; "
                "otherwise its rows are stamped as they arrive at the host. "
                "`clock.inertial_files_on` says, per channel, which of the two "
                "happened and why, and `clock.arrival_timed` lists the "
                "channels on arrival time. Arrival time is monotonic and "
                "shared, so channels stay comparable; what it costs is "
                "per-sample spacing, which carries the transport's jitter -- "
                "over Bluetooth that is a burst of readings in one "
                "millisecond and then a gap. Treat the average rate as sound "
                "and individual gaps as approximate. "
                "A channel listed under `clock.unaligned` additionally has no "
                "measured offset of its own; that is fine while it is on "
                "arrival time and matters the moment it is not. "
                "Neither clock is wall time, deliberately: wall time can step "
                "backwards under NTP correction, and a run containing a "
                "backwards step is silently unusable."),
            "units": (
                "An inertial unit's scales are MEASURED on arrival, not taken "
                "from a datasheet, and the verdict for each is in "
                "`cell.inertial`. `gyro_units` says whether that unit "
                "publishes degrees or radians per second -- the rig's LPMS-B2 "
                "through FusionHub publishes degrees, and the two are the same "
                "numbers a factor of 57.3 apart. `quat_convention` says which "
                "way round it publishes its quaternion; `conjugate` means it "
                "publishes world-to-sensor and was inverted on the way in. "
                "Whatever those say, the columns in these files are always in "
                "the units their headings name: radians per second, metres per "
                "second squared, and a sensor-to-world quaternion."),
            "scored_channels": (
                "Only channels marked role=benchmark in `channels` count "
                "toward a GCR number. The rest are evidence for the "
                "inspection case. Scoring an unscored channel is the easiest "
                "way to produce a number that means nothing."),
        },
    }
    try:
        (folder / "manifest.json").write_text(
            json.dumps(manifest, indent=2, default=str), encoding="utf-8")
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not write the manifest: {e}"}

    # A README that a person opens, saying the same thing in sentences.
    try:
        (folder / "README.md").write_text(_readme(manifest), encoding="utf-8")
    except Exception:
        pass

    size = sum(f.stat().st_size for f in folder.rglob("*") if f.is_file())
    return {"ok": True, "path": str(folder.resolve()),
            "runs": len(copied_runs), "inertial": len(copied_imu),
            "bytes": size,
            "note": (f"{len(copied_runs)} run files and {len(copied_imu)} "
                     f"inertial logs, with the calibration, the channel "
                     f"definitions and the clock they are all on.")}


def _robot_link(ctx) -> dict:
    h = ctx.robot_health() or {}
    return {
        "interface": ("RTDE, port 30004" if h.get("source") == "rtde"
                      else "primary, port 30003 (FALLBACK)"
                      if h.get("source") else "none"),
        "source": h.get("source"),
        "rate_hz": h.get("rate_hz"),
        "fields": len(h.get("fields") or []),
        "controller_version": h.get("controller_version"),
        "reconnects": h.get("reconnects"),
        "errors": h.get("errors"),
        "rtde_unavailable_reason": h.get("rtde_unavailable_reason") or "",
        "degraded": h.get("degraded") or "",
        "warning": (
            "" if not (h.get("degraded") or "") else
            "These runs were recorded over the fallback interface, which "
            "carries a fraction of the fields and does not stream steadily. "
            "Check `measured` for how much of each run holds fresh robot "
            "state before using any of it."),
    }


def _carrier_words(car: dict) -> str:
    m = float(car.get("carrier_mass_kg") or 0.0)
    com = car.get("carrier_com_m") or [0.0, 0.0, 0.0]
    if m <= 0.0:
        return f"{car.get('carrier_id', '?')}: nothing on the flange."
    where = (f", centre of mass {com[0] * 1000:.0f}/{com[1] * 1000:.0f}/"
             f"{com[2] * 1000:.0f} mm from it" if any(com) else "")
    return f"{car.get('carrier_id', '?')}: {m:.3f} kg on the flange{where}."


def _audit_runs(runs_dir) -> dict:
    """
    Read back what was actually written, per run: how many samples, over what
    span, at what rate, with what worst gap, and whether the commanded joint
    trajectory is in there.

    Written by reading the files rather than by remembering what was intended,
    because the two differed: a smoke-test run declared 125 Hz in its own
    manifest and held 43 Hz with a 1.5 s hole in it, and nothing said so.
    """
    out = {}
    runs_dir = Path(runs_dir)
    if not runs_dir.exists():
        return out
    for f in sorted(runs_dir.iterdir()):
        if not f.is_file() or f.suffix != ".jsonl":
            continue
        declared, ts, with_target = None, [], 0
        labelled_vel, peak_qd, held = None, 0.0, 0.0
        qd_hist = []
        ages, jumps = [], []
        last_q, repeats, n_q = None, 0, 0
        try:
            with f.open(encoding="utf-8") as fh:
                for i, line in enumerate(fh):
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        row = json.loads(line)
                    except Exception:       # noqa: BLE001
                        continue
                    if i == 0 and "_manifest" in row:
                        declared = row["_manifest"].get("sample_rate_hz")
                        labelled_vel = row["_manifest"].get("joint_vel")
                        continue
                    t = row.get("t")
                    if isinstance(t, (int, float)):
                        ts.append(float(t))
                    if row.get("target_q"):
                        with_target += 1
                    # DID THE RUN ACTUALLY REACH THE SPEED ITS CELL IS NAMED
                    # AFTER? The commanded joint velocity is recorded, so this
                    # is answerable from the file rather than assumed from the
                    # command. On the real cell a fixed-excursion sweep produced
                    # runs labelled 0.6 rad/s whose joint never settled there
                    # and a 0.9 rad/s cell the arm could not reach at all -- a
                    # mislabelled condition cell that nothing downstream can
                    # detect, which is the one defect that silently invalidates
                    # a whole campaign rather than one run.
                    if isinstance(row.get("robot_age_s"), (int, float)):
                        ages.append(float(row["robot_age_s"]))
                    qv = row.get("q")
                    if isinstance(qv, list):
                        qt = tuple(qv)
                        if qt == last_q:
                            repeats += 1
                        else:
                            if last_q is not None:
                                jumps.append(max(abs(a - b)
                                                 for a, b in zip(qt, last_q)))
                            last_q = qt
                        n_q += 1
                    tqd = row.get("target_qd")
                    if isinstance(tqd, list) and tqd and isinstance(t, (int, float)):
                        w = max(abs(float(v)) for v in tqd)
                        qd_hist.append((float(t), w))
                        peak_qd = max(peak_qd, w)
        except Exception as e:               # noqa: BLE001
            out[f.name] = {"error": str(e)}
            continue
        n = len(ts)
        span = (ts[-1] - ts[0]) if n >= 2 else 0.0
        gaps = [b - a for a, b in zip(ts, ts[1:])] if n >= 2 else []
        block = {
            "samples": n,
            "span_s": round(span, 3),
            "declared_rate_hz": declared,
            "achieved_rate_hz": round((n - 1) / span, 1) if span > 0 else None,
            "worst_gap_s": round(max(gaps), 3) if gaps else None,
            "monotonic": all(g > 0 for g in gaps) if gaps else True,
            "distinct_timestamps": len(set(ts)),
            "with_commanded_trajectory": with_target,
        }
        notes = []
        if declared and block["achieved_rate_hz"] and \
                block["achieved_rate_hz"] < 0.8 * float(declared):
            notes.append(f"held {block['achieved_rate_hz']:.0f} Hz of the "
                         f"{float(declared):.0f} Hz its manifest declares; "
                         f"generate or resample the simulated side at the "
                         f"achieved rate, not the declared one")
        if n and block["distinct_timestamps"] < n:
            notes.append(f"{n - block['distinct_timestamps']} samples share a "
                         f"timestamp with another sample")
        if not block["monotonic"]:
            notes.append("time does not increase monotonically through this file")
        if n and with_target < n:
            notes.append(f"{n - with_target} samples carry no commanded joint "
                         f"trajectory, so those cannot be replayed in a "
                         f"simulator")

        # How long the commanded joint velocity actually sat at the cell's label.
        if labelled_vel and qd_hist:
            want = float(labelled_vel)
            # ACCUMULATED time at speed, not the span from the first sample at
            # speed to the last. The span counts the dwell between the out and
            # back legs as though the joint had been cruising through it, and
            # on a real sweep that turned 4.4 s of motion into 16 s of
            # "held" -- which would wave through exactly the too-short cells
            # this check exists to catch.
            # Each interval contributes at most a few times the file's own
            # median spacing. On a recording that stalled -- and one campaign's
            # did, for over two seconds at a stretch -- a single sample at
            # speed followed by a long gap would otherwise be counted as
            # seconds of cruising, and a 25 deg move at 0.2 rad/s came out as
            # twelve seconds at speed when the joint only travels for four.
            med = st.median(gaps) if gaps else 0.01
            cap = max(4.0 * med, 0.02)
            held = 0.0
            for (ta, wa), (tb, _) in zip(qd_hist, qd_hist[1:]):
                if wa >= 0.95 * want and tb > ta:
                    held += min(tb - ta, cap)
            block["labelled_joint_vel"] = want
            block["peak_commanded_joint_vel"] = round(peak_qd, 4)
            block["held_at_labelled_vel_s"] = round(held, 3)
            if peak_qd < 0.95 * want:
                notes.append(
                    f"this run is labelled {want:g} rad/s but the joint was "
                    f"never commanded above {peak_qd:.3f} rad/s. The cell key "
                    f"names a speed that did not happen -- do not group results "
                    f"by it")
            elif held < 0.5:
                # Summed over the whole run, so both legs of an out-and-back
                # count -- a weaker bar than the pre-move check on purpose,
                # catching only runs that plainly never cruised.
                notes.append(
                    f"labelled {want:g} rad/s but the joint only held that for "
                    f"{held:.2f} s: the move is a ramp up and straight back "
                    f"down, not a cruise, so this cell is not really distinct "
                    f"from a slower one. Give the move room (plateau_s) and "
                    f"re-record")
        # IS THE ROBOT STATE IN THIS FILE ACTUALLY MOVING?
        #
        # A stalled link does not produce an empty file, it produces a full one
        # in which the same reading is written over and over. Every row is
        # well-formed; the trajectory is a staircase. This is measured from the
        # file because it cannot be seen in it.
        if n_q:
            frac = repeats / n_q
            block["repeated_robot_state_frac"] = round(frac, 4)
            block["distinct_robot_states"] = n_q - repeats
            if jumps:
                block["worst_joint_jump_deg"] = round(math.degrees(max(jumps)), 2)
            if frac > 0.25:
                notes.append(
                    f"{100 * frac:.0f}% of this run's rows repeat the previous "
                    f"row's robot state, and when it does change the joints "
                    f"jump by up to "
                    f"{math.degrees(max(jumps)) if jumps else 0:.0f} degrees. "
                    f"The robot link was stalling: this file holds "
                    f"{n_q - repeats} real robot readings wearing the shape of "
                    f"{n_q} samples. Do not measure a gap against it — check "
                    f"the robot is on RTDE and re-record")
        if ages:
            block["robot_age_median_s"] = round(st.median(ages), 4)
            block["robot_age_p95_s"] = round(sorted(ages)[int(.95 * len(ages))], 4)
            block["robot_age_max_s"] = round(max(ages), 3)
            if st.median(ages) > 0.05:
                notes.append(
                    f"the robot reading in a typical row of this run was "
                    f"{st.median(ages) * 1000:.0f} ms old when it was written, "
                    f"and the worst was {max(ages):.1f} s")
        block["notes"] = notes
        out[f.name] = block
    return out


def _clock_block(ctx) -> dict:
    """
    What is actually known about time in this dataset, including what is not.

    A manifest that says "everything is on one clock" when two channels have
    never been tied together is worse than one that says nothing: it invites
    exactly the cross-channel subtraction that produces a plausible wrong
    number. So the channels with a measured offset and the channels without
    are both listed, by name.
    """
    st = ctx.clock_status() or {}
    offsets = st.get("offsets_ms", {}) or {}
    chans = st.get("channels", {}) or {}
    units = list((ctx.imu_status() or {}).keys())
    aligned = sorted(k for k in offsets)
    unaligned = sorted(u for u in units if u not in offsets)
    # Which timebase each channel's rows were actually written on, in its own
    # words. A dataset that says "each sensor's own clock" while one sensor's
    # clock never advanced is a dataset that will be differenced wrongly by
    # whoever reads it next -- including its author, months later.
    timebase = {c: b.get("why", "") for c, b in chans.items()}
    stalled = sorted(c for c, b in chans.items() if b.get("clock") == "stalled")
    notes = []
    if unaligned:
        notes.append("These channels have no measured offset to the host clock: "
                     + ", ".join(unaligned)
                     + ". Do not difference them against another channel until "
                       "one is measured — the tap check on the Record page is "
                       "what measures it.")
    if stalled:
        notes.append("These channels publish a timestamp that never advances, so "
                     "their rows are timed by arrival at the host instead: "
                     + ", ".join(stalled)
                     + ". Arrival time is monotonic and shared with every other "
                       "channel, but it carries the transport's jitter — for the "
                       "LPMS-B2 over Bluetooth that is a burst of packets in one "
                       "millisecond and then a gap, so treat per-sample spacing "
                       "as approximate and the average rate as sound.")
    return {
        **st,
        "run_files_on": "host monotonic, seconds since the agent started",
        "inertial_files_on": timebase or "host monotonic",
        "aligned": aligned,
        "unaligned": unaligned,
        "arrival_timed": sorted(st.get("arrival_timed", []) or []),
        "warning": " ".join(notes),
    }


def _readme(m: dict) -> str:
    cal = m.get("hand_eye") or {}
    return f"""# {m['name']}

Written {m['written_utc']} by the {m['produced_by']}.
Format `{m['dataset_version']}`.

## What is in here

| Folder | What it holds |
|---|---|
| `runs/` | {len(m['contents']['runs'])} benchmark run files (`.jsonl`) |
| `inertial/` | {len(m['contents']['inertial'])} continuous inertial logs (`.csv`) |
| `robot/` | {len(m['contents'].get('robot', []))} continuous robot logs (`.csv`), every RTDE packet |
| `manifest.json` | the machine-readable version of this file |

## Before you use it for anything

**Check `clock` in `manifest.json` before you difference two channels.**
Run files are stamped on the host monotonic clock. Inertial CSVs are stamped
on the sensor's own clock plus whatever offset has been measured for it. Any
channel listed under `clock.unaligned` has never been tied to the host clock,
so its times are internally consistent and are *not* comparable with another
channel's. Neither is wall time, deliberately: wall time can step backwards
under NTP correction, and a run containing a backwards step is silently
unusable.

**Only channels marked `role: benchmark` in `manifest.json` are scored.**
The others are there because the inspection case needs them. A GCR computed
over an unscored channel is a number with nothing behind it.

**The hand-eye calibration in the manifest is the one these runs were taken
with**: {cal.get('calib_version', 'none recorded')}{
    f", reconstructing the board to {cal['target_spread_mm']} mm"
    if cal.get('target_spread_mm') is not None else ''}.
Every camera-derived 3D quantity in here inherits it. If it is wrong, they
are all wrong together, in a way that grows with standoff and rotates with
the tool.

## Reading a run

```python
import json
with open("runs/<name>.jsonl") as f:
    manifest = json.loads(f.readline())["_manifest"]   # the run's parameters
    samples  = [json.loads(line) for line in f]
```

## Reading an inertial log

```python
import csv
with open("inertial/<name>.csv") as f:
    rows = list(csv.DictReader(f))
```

Empty cells mean the sensor does not provide that channel, which is a
statement. A missing column would be a question.
"""
