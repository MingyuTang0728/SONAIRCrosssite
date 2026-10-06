"""
E1, the identification set, and the campaign's protocol 2.

  1. the program the controller runs is the profile the safety check walks:
     the URScript is executed here line for line and integrated, and lands
     on the same path to a micro-radian;
  2. every planned excitation stays under the speed and acceleration caps;
  3. an excitation that would leave the envelope is turned round or made
     smaller, and one that cannot fit is refused with a reason in words;
  4. end to end on a simulated cell: the bare-carrier set is recorded as E1
     runs, kept in their own book, and left out of scoring; the added-mass
     set is refused until the carrier is re-described heavier;
  5. protocol 2: pilot runs (protocol 1) do not count as done, and point-to-
     point and stop-start travel the same 45 deg at every speed.

Run:  python tests/test_e1.py
"""
from __future__ import annotations

import math
import sys
import threading
import time
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import automation                                     # noqa: E402
import bench_agent                                    # noqa: E402
import campaign_runner as cr                          # noqa: E402
import carrier                                        # noqa: E402
import ident_set as e1                                # noqa: E402
import ur_control                                     # noqa: E402
import ur_telemetry as urt                            # noqa: E402
import test_campaign as tc                            # noqa: E402
from sonair_benchmark.schema import read_run          # noqa: E402

Q_MID = [0.0, -1.85005, 1.85005, -1.57, -1.57, 0.0]


def to_python(prog: str) -> str:
    """URScript as ident_set writes it is Python once its `end`s are gone."""
    return "\n".join(l for l in prog.splitlines() if l.strip() != "end")


def execute(prog: str, q0, speedj=None, movej=None):
    """Run a program; by default only integrate, no waiting."""
    q = list(q0)
    path = [list(q)]

    def _speedj(v, a, t):
        for i in range(6):
            q[i] += v[i] * t
        path.append(list(q))
    env = {"sin": math.sin, "cos": math.cos, "pow": pow,
           "speedj": speedj or _speedj, "stopj": lambda a: None,
           "movej": movej or (lambda target, a=1.2, v=0.3: None)}
    exec(compile(to_python(prog), "<urscript>", "exec"), env)
    return path


def test_program_is_the_profile():
    for spec in ({"kind": "chirp", "joint": 2, "sign": -1, "scale": 0.7},
                 {"kind": "fourier", "seed": 1, "sign": 1, "scale": 1.0}):
        _, prof = e1.profile(spec, 0.8)
        path = execute(e1.urscript(spec, Q_MID, 0.8), Q_MID)
        assert len(path) == len(prof), (len(path), len(prof))
        err = max(abs((p[i] - Q_MID[i]) - d[i])
                  for p, d in zip(path, prof) for i in range(6))
        assert err < 1e-6, err
    print(f"  pass  the controller program, executed, follows the checked "
          f"profile to {err:.1e} rad")


def test_caps():
    worst_v = worst_a = 0.0
    specs = [{"kind": "chirp", "joint": j} for j in range(6)] + \
            [{"kind": "fourier", "seed": s} for s in e1.FOURIER_SEEDS]
    for spec in specs:
        _, p = e1.profile(spec, 0.8)
        for j in range(6):
            v = [(b[j] - a[j]) / e1.DT for a, b in zip(p, p[1:])]
            acc = [(b - a) / e1.DT for a, b in zip(v, v[1:])]
            worst_v = max(worst_v, max(abs(x) for x in v))
            worst_a = max(worst_a, max(abs(x) for x in acc))
    assert worst_v <= 0.8 + 1e-9 and worst_a <= e1.A_MAX * 1.02, (worst_v, worst_a)
    print(f"  pass  every excitation under the caps: fastest joint "
          f"{worst_v:.2f} rad/s, sharpest {worst_a:.2f} rad/s^2")


def test_fit_and_refuse():
    base = cr._tool_at(Q_MID, None)
    # a limit on the side the shoulder chirp swings the tool towards: it has
    # to be turned round (or made smaller) to stay inside
    spec = {"kind": "chirp", "joint": 1, "sign": 1, "scale": 1.0}
    _, p = e1.profile(spec)
    zs = [cr._tool_at([a + b for a, b in zip(Q_MID, d)], None)[2] for d in p[::25]]
    up = max(zs) - base[2] > base[2] - min(zs)
    lim = (lambda q: (q[2] <= base[2] + 0.005, "above the limit")) if up else \
          (lambda q: (q[2] >= base[2] - 0.005, "below the limit"))
    assert e1.check(spec, Q_MID, lim)
    res = e1.fit({"kind": "chirp", "joint": 1}, Q_MID, lim)
    assert res["ok"] and (res["spec"]["sign"] == -1 or res["spec"]["scale"] < 1), res
    # a box the tool is already at the edge of: nothing fits, and it says why
    box = lambda p: (abs(p[2] - base[2]) < 0.002, "outside a 2 mm box")
    res = e1.fit({"kind": "fourier", "seed": 0}, Q_MID, box)
    assert not res["ok"] and "safe envelope" in res["why"], res
    print("  pass  an excitation that would leave the envelope is turned round "
          "or shrunk; one that cannot fit is refused: " + res["why"][:50] + "...")


class E1Cell(tc.Cell):
    """The campaign test's simulated cell, able to run an excitation program."""

    def run_script(self, prog):
        self.programs = getattr(self, "programs", 0) + 1

        def speedj(v, a, t):
            with self.lock:
                self.q = [c + x * t for c, x in zip(self.q, v)]
                self.qd = list(v)
            time.sleep(t)

        def movej(target, a=1.2, v=0.3):
            with self.lock:
                self.qd = [0.0] * 6
            self.move_joints(target, v, a)
        threading.Thread(target=execute, args=(prog, self.joints(), speedj, movej),
                         daemon=True).start()
        return True, ""


def test_end_to_end():
    import os
    os.chdir(tc.ROOT)
    # short versions of every motion, so the whole set runs in a minute or two
    e1.CHIRP_S, e1.TAPER_S = 3.0, 0.5
    e1.FOURIER_PERIOD_S, e1.FOURIER_PERIODS = 2.0, 1
    e1.FOURIER_SEEDS, e1.FOURIER_REPEATS = (0,), 1
    state = tc.ROOT / "state_e1.json"
    cell = E1Cell()
    svc = urt.URTelemetry("sim", use_process=False)
    mod = types.ModuleType("ur_bridge_ext")
    mod.UR = type("UR", (), {"enabled": True, "telemetry": svc})
    sys.modules["ur_bridge_ext"] = mod
    stop = threading.Event()

    def feed():
        while not stop.is_set():
            q = cell.state()
            svc._publish({"actual_q": q["q"], "actual_qd": q["qd"],
                          "target_q": q["q"], "target_qd": q["qd"],
                          "actual_TCP_pose": q["tcp"], "robot_mode": 7,
                          "safety_mode": 1}, "rtde")
            time.sleep(0.008)
    threading.Thread(target=feed, daemon=True).start()

    sug = cr.suggest(cell.q, cell.tcp_pose(), ur_control.Envelope().as_dict(),
                     cell.pose_allowed)
    for c, g in sug["configs"].items():
        assert cr.teach(c, g["q"], g["direction"], state)["ok"]
    st = cr.load_state(state)
    pv = e1.preview("bare", st, cell.pose_allowed, cell.tcp_pose(), cell.joints(),
                    carrier=cell.carrier())
    assert pv["ok"] and pv["runs"] == 9, pv["problems"]

    R = automation.Runner(cell)
    job = e1.build_job("bare", st, pv["fitted"], state)
    job.requires = ["robot"]
    t0 = time.time()
    out = tc.run_job(R, job)
    assert out["state"] == "done", [e["text"] for e in out["log"][-6:]]
    st = cr.load_state(state)
    assert len(st["e1"]["done"]) == 9 and not st["done"], st.get("e1")
    assert cell.programs == 9
    names = sorted(Path(v["path"]).name for v in st["e1"]["done"].values())
    run = read_run(Path(st["e1"]["done"][
        "e1_bare_mid_workspace_chirp_j3_r00"]["path"]))
    m = run.manifest
    assert m.experiment == "E1" and m.traj_type == "chirp", m
    assert '"joint": 2' in m.notes and run.samples
    moved = max(abs(s["q"][2] - run.samples[0]["q"][2]) for s in run.samples)
    assert moved > math.radians(3), math.degrees(moved)
    print(f"  pass  bare-carrier set: {len(names)} E1 runs recorded and "
          f"checked in {time.time() - t0:.0f}s, in their own book "
          f"(evaluation done: {len(st['done'])}); the elbow chirp moved "
          f"{math.degrees(moved):.0f} deg")

    # left out of scoring
    from sonair_benchmark import cli
    pairs = cli._load_pairs(bench_agent.RECORDER.out_dir, bench_agent.RECORDER.out_dir)
    assert not pairs
    # the added set: refused on the same carrier, accepted once it is heavier
    pv = e1.preview("added", st, cell.pose_allowed, cell.tcp_pose(),
                    cell.joints(), carrier=cell.carrier())
    assert not pv["ok"] and any("not 0.2 kg more" in p for p in pv["problems"])
    carrier.save({"carrier_id": "carrier-v1+0.5kg", "carrier_mass_kg": 0.69,
                  "carrier_com_m": [-0.01, -0.03, 0.02]}, cell.carfile)
    pv = e1.preview("added", st, cell.pose_allowed, cell.tcp_pose(),
                    cell.joints(), carrier=cell.carrier())
    assert pv["ok"], pv["problems"]
    # and the bare set is guarded against a changed carrier
    assert "was there" in e1.load_problem("bare", cr.load_state(state),
                                          cell.carrier())
    stop.set()
    print("  pass  added-mass set refused until the carrier is described "
          "heavier; E1 runs are left out of scoring")


def test_protocol_2():
    st = {"configs": {}, "done": {"x": {"session": 0}}, "rejected": {},
          "sessions": {}}
    assert cr.done_ids(st) == set()
    st["done"]["x"]["protocol"] = cr.PROTOCOL
    assert cr.done_ids(st) == {"x"}
    for v in (0.2, 0.5, 0.9):
        for tt in ("point_to_point", "stop_start"):
            ex = max(abs(x) for x in cr.motion(tt, v)["excursion"])
            assert abs(math.degrees(ex) - 45.0) < 1e-6, (tt, v, ex)
    print("  pass  protocol 2: pilot runs are not counted as done; point-to-"
          "point and stop-start travel 45 deg at every speed")


def main():
    test_program_is_the_profile()
    test_caps()
    test_fit_and_refuse()
    test_protocol_2()
    test_end_to_end()
    print("all passed")


if __name__ == "__main__":
    main()
