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
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

log = logging.getLogger("automation")

# The step vocabulary. Small on purpose: every one of these is a thing an
# operator can also do by hand on some page of the console, so a job that
# misbehaves can be taken apart and each step tried on its own.
STEP_KINDS = (
    "preflight",        # run the checks; fail the job if any check fails
    "dwell",            # wait, so a move can settle before a capture
    "move",             # go to one tool pose
    "trajectory",       # go through a list of tool poses
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


def preflight(ctx) -> dict:
    """
    Ask the cell whether it is fit to run, and say so in sentences.

    `ctx` is the bridge's capability bundle (see `Runner`), which is how this
    module stays testable: nothing here imports the bridge, opens a socket or
    touches hardware. Everything it knows, it is handed.

    Ordered the way a person would check: can we talk to the robot, will the
    robot accept commands, can we see, do we know where the camera is, are the
    sensors live, is there room to write.
    """
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
        checks.append(Check("camera", "Camera", "warn",
                            "No camera. Steps that need a picture will fail; "
                            "motion and inertial capture will not.",
                            blocking=False))
    elif cam_age > 2.0:
        checks.append(Check("camera", "Camera", "fail",
                            f"The last picture arrived {cam_age:.0f} s ago."))
    else:
        checks.append(Check("camera", "Camera", "pass", "Streaming."))

    # --- calibration ---------------------------------------------------
    cal = ctx.calibration()
    if not cal:
        checks.append(Check("calibration", "Camera position known", "fail",
                            "No hand-eye calibration is loaded, so nothing the "
                            "camera sees can be placed in the robot's frame. "
                            "Run it on the Calibrate page, or load the saved "
                            "one."))
    else:
        spread = cal.get("target_spread_mm")
        if spread is not None and spread > 5.0:
            checks.append(Check("calibration", "Camera position known", "fail",
                                f"The loaded calibration reconstructs the board "
                                f"to {spread:.1f} mm, which is too loose to "
                                "build a dataset on. Run it again."))
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
                "notes": self.notes}


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
        pre = preflight(self.ctx)
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
            pre = preflight(self.ctx)
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

        if kind == "trajectory":
            poses = step.get("poses") or []
            if not poses:
                return False, "no poses given"
            speed = float(params.get("joint_vel", step.get("speed", 0.12)))
            self._say(f"Running {len(poses)} poses at {speed:g} m/s.")
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

        if kind == "record_start":
            run_id = self._run_id(step, params, iteration, job)
            args = {
                "run_id": run_id,
                "joint_vel": float(params.get("joint_vel", step.get("joint_vel", 0.4))),
                "arm_config": step.get("arm_config", "mid_workspace"),
                "traj_type": step.get("traj_type", "contour"),
                "repeat_idx": int(params.get("repeat_idx", 0)),
                "calib_version": (self.ctx.calibration() or {}).get("calib_version", "calib-0"),
                "rate_hz": float(step.get("rate_hz", 125.0)),
                "operator": step.get("operator", "automation"),
                "notes": job.notes,
            }
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
            self._say(f"Saved {res.get('n', 0)} samples to "
                      f"{res.get('path', '')}.", "ok")
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

def builtin_jobs(tcp_pose=None) -> dict:
    """
    Jobs that are useful on day one, written against whatever pose the arm is
    in so they can be run without anybody typing coordinates.

    They are templates, not fixtures: the console shows the steps and the
    operator can change them before pressing Run.
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
    # repeat unattended.
    square = [shifted(), shifted(dx=0.08), shifted(dx=0.08, dy=0.08),
              shifted(dy=0.08), shifted()]

    return {
        "checkout": Job(
            name="checkout",
            notes="Prove the cell is fit before trusting a campaign to it.",
            steps=[
                {"kind": "preflight"},
                {"kind": "message", "text": "Everything the cell needs is present."},
            ],
        ),
        "single_run": Job(
            name="single_run",
            notes="One recorded pass, for checking a change before a campaign.",
            steps=[
                {"kind": "preflight"},
                {"kind": "imu_log_start"},
                {"kind": "record_start", "traj_type": "contour", "joint_vel": 0.25},
                {"kind": "dwell", "seconds": 1.0},
                {"kind": "trajectory", "poses": square, "speed": 0.08},
                {"kind": "dwell", "seconds": 1.0},
                {"kind": "record_stop"},
                {"kind": "imu_log_stop"},
            ],
        ),
        "speed_sweep": Job(
            name="speed_sweep",
            notes=("The benchmark campaign: the same motion at four speeds, "
                   "three times each, then exported as one dataset."),
            repeats=3,
            sweep_key="joint_vel",
            sweep_values=[0.05, 0.10, 0.20, 0.35],
            steps=[
                {"kind": "preflight"},
                {"kind": "imu_log_start"},
                {"kind": "record_start", "traj_type": "contour"},
                {"kind": "dwell", "seconds": 1.0},
                {"kind": "trajectory", "poses": square},
                {"kind": "dwell", "seconds": 1.0},
                {"kind": "record_stop"},
                {"kind": "imu_log_stop"},
            ],
        ),
    }


# ---------------------------------------------------------------------------
# the dataset
# ---------------------------------------------------------------------------

DATASET_VERSION = "sonair-dataset/1"


def export_dataset(name: str, *, out_root, runs_dir, imu_dir, ctx) -> dict:
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
    except Exception as e:      # noqa: BLE001
        return {"ok": False, "error": f"could not create {folder}: {e}"}

    # Only what this campaign produced. Sweeping up everything in the runs
    # folder would quietly fold last week's runs into this week's dataset.
    since = ctx.job_started_at() or 0.0
    copied_runs, copied_imu = [], []
    for src, dest, bucket in (
        (Path(runs_dir), folder / "runs", copied_runs),
        (Path(imu_dir), folder / "inertial", copied_imu),
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
            "camera": ctx.camera_info(),
            "inertial": ctx.imu_status(),
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
        },
        "reading_it": {
            "runs/*.jsonl": (
                "One benchmark run each. First line is the run manifest "
                "(parameters, calibration version, sample rate); every line "
                "after it is one sample on a fixed time grid."),
            "inertial/*.csv": (
                "Every inertial sample at the sensor's own rate, one row per "
                "reading, fixed column set. Columns a unit does not provide "
                "are present and empty."),
            "timestamps": (
                "Run files are stamped on the host monotonic clock (seconds "
                "since the agent started). Inertial CSVs are stamped on the "
                "SENSOR's own clock plus whatever offset has been measured "
                "for it -- see `clock.aligned` and `clock.unaligned` in this "
                "manifest. A channel listed under `unaligned` has NOT been "
                "tied to the host clock, so its timestamps are internally "
                "consistent and are NOT directly comparable with another "
                "channel's. Align it, or difference it only against itself. "
                "Neither clock is wall time, deliberately: wall time can step "
                "backwards under NTP correction, and a run containing a "
                "backwards step is silently unusable."),
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
    units = list((ctx.imu_status() or {}).keys())
    aligned = sorted(k for k in offsets)
    unaligned = sorted(u for u in units if u not in offsets)
    return {
        **st,
        "run_files_on": "host monotonic, seconds since the agent started",
        "inertial_files_on": "each sensor's own clock, plus its measured offset",
        "aligned": aligned,
        "unaligned": unaligned,
        "warning": ("" if not unaligned else
                    "These channels have no measured offset to the host clock: "
                    + ", ".join(unaligned)
                    + ". Do not difference them against another channel until "
                      "one is measured — the tap check on the Record page is "
                      "what measures it."),
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
