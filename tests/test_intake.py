"""
Bring your own data: a robot owner's log in, a sim-to-real report out.

A log is made the way a UR owner would have one: the UR RTDE client's CSV
(space-separated, timestamp, target_q_0.., actual_q_0.., actual_TCP_pose_0..)
of an arm carrying a 10 cm tool, doing five moves at different speeds with
pauses between, its measured joints trailing the command a little, as a real
servo does.

  1. the format is recognised without help, and the log is cut into its five
     motions, each labelled as user data (experiment U) for that robot;
  2. every motion is replayed through S0 with the tool offset worked out from
     the log itself (the frame check passes), and scored by the benchmark's
     own rule -- faster motions show the bigger gap;
  3. report.json and report.html are written; the page loads cleanly;
  4. the same data as a generic CSV in degrees and milliseconds, through a
     mapping file, gives the same answer;
  5. what is wrong is said plainly: no commanded joints; degrees with no
     mapping; an unknown format with no mapping; a folder already in use;
  6. user data never reaches a leaderboard.

Run:  python tests/test_intake.py   (2-4 need mujoco + a menagerie clone)
"""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

import ur_kin                                         # noqa: E402
from sonair_benchmark import cli                      # noqa: E402
from sonair_benchmark.intake import (IntakeError, read_log, check, motions,  # noqa: E402
                                     run_intake)
from sonair_benchmark.schema import read_dataset      # noqa: E402

TMP = Path(tempfile.mkdtemp())
TOOL = [0.0, 0.0, 0.10]                               # 10 cm along the tool axis


def menagerie():
    for d in (os.environ.get("MENAGERIE"), ROOT / "mujoco_menagerie",
              Path.home() / ".sonair" / "mujoco_menagerie"):
        if d and (Path(d) / "universal_robots_ur5e" / "scene.xml").exists():
            return Path(d)
    return None


def make_log(robot="ur5e"):
    """Five elbow+shoulder moves, 0.2 .. 1.2 rad/s, with pauses; 125 Hz."""
    dh = ur_kin.DH[robot]
    q0 = [0.0, -1.85, 1.85, -1.57, -1.57, 0.0]
    dt, t = 0.008, 0.0
    cmd, rows = list(q0), []
    act, vel = list(q0), [0.0] * 6
    plan = [(0.2, 0.5), (0.5, -0.5), (0.8, 0.6), (1.2, -0.6), (0.35, 0.3)]
    seq = []
    for v, amp in plan:
        seq += [("hold", 1.2)] + [("move", v, amp)]
    seq += [("hold", 1.5)]
    for step in seq:
        if step[0] == "hold":
            n = int(step[1] / dt)
            targets = [list(cmd)] * n
        else:
            v, amp = step[1], step[2]
            a = 3.0
            ramp = v / a
            cruise = max(0.0, (abs(amp) - v * v / a) / v)
            T = 2 * ramp + cruise
            start = list(cmd)
            targets = []
            for k in range(int(T / dt) + 1):
                tt = k * dt
                s = (0.5 * a * tt * tt if tt < ramp else
                     0.5 * v * ramp + v * (tt - ramp) if tt < ramp + cruise else
                     abs(amp) - 0.5 * a * max(0.0, T - tt) ** 2)
                s = math.copysign(min(s, abs(amp)), amp)
                q = list(start)
                q[2] += s
                q[1] -= 0.4 * s
                targets.append(q)
        for q in targets:
            cmd = q
            # a servo: second-order follow of the command
            for j in range(6):
                acc = 900.0 * (cmd[j] - act[j]) - 55.0 * vel[j]
                vel[j] += acc * dt
                act[j] += vel[j] * dt
            T4 = ur_kin.fk(act, dh)
            p = T4[:3, 3] + T4[:3, :3] @ ur_kin.np.asarray(TOOL)
            rv = ur_kin.rotvec(T4[:3, :3])
            rows.append((1000.0 + t, list(cmd), list(act), list(p) + list(rv)))
            t += dt
    return rows, len(plan)


def write_rtde(rows, path):
    cols = (["timestamp"] + [f"target_q_{i}" for i in range(6)]
            + [f"actual_q_{i}" for i in range(6)]
            + [f"actual_TCP_pose_{i}" for i in range(6)])
    with open(path, "w") as fh:
        fh.write(" ".join(cols) + "\n")
        for t, c, a, p in rows:
            fh.write(" ".join(f"{v:.6f}" for v in [t] + c + a + p) + "\n")


def write_generic(rows, path):
    with open(path, "w") as fh:
        fh.write("time_ms,cmd1,cmd2,cmd3,cmd4,cmd5,cmd6,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz\n")
        for t, c, a, p in rows:
            vals = ([t * 1000] + [math.degrees(v) for v in c] + [math.degrees(v) for v in a]
                    + p)
            fh.write(",".join(f"{v:.6f}" for v in vals) + "\n")


def test_reading_and_cutting(rtde, n_moves):
    tb = read_log(rtde)
    assert tb.fmt == "ur-rtde" and tb.tcp_pos and tb.tcp_rot
    info = check(tb, "ur5e")
    segs = motions(tb, info)
    assert abs(info["rate_hz"] - 125) < 1, info
    assert len(segs) == n_moves, segs
    peaks = [round(p, 2) for _, _, p in segs]
    assert all(abs(a - b) < 0.1 for a, b in zip(peaks, [0.2, 0.5, 0.8, 1.2, 0.35])), peaks
    print(f"  pass  UR RTDE log recognised: {info['rows']} rows at "
          f"{info['rate_hz']} Hz, cut into {len(segs)} motions, peak speeds {peaks}")


def test_full_report(rtde):
    if menagerie() is None:
        print("  skipped  the S0 replay and report (needs mujoco and a menagerie clone)")
        return None
    out = TMP / "report"
    rep = run_intake(rtde, out, robot="ur5e", payload_kg=0.0,
                     menagerie=menagerie(), log=lambda *_: None)
    assert not rep["problems"], rep["problems"]
    ms = rep["motions"]
    assert len(ms) == 5 and all("tool_p95_mm" in m for m in ms), ms
    real = read_dataset(out / "runs" / "real", side="real")
    assert all(r.manifest.experiment == "U" and r.manifest.robot == "ur5e" for r in real)
    by = sorted(ms, key=lambda m: m["peak_speed"])
    assert by[-1]["tool_p95_mm"] > by[0]["tool_p95_mm"], by
    assert (out / "report.html").exists() and json.loads((out / "report.json").read_text())
    ov = rep["overall"]
    print(f"  pass  every motion replayed through S0 (frame check passed with the "
          f"tool offset from the log) and scored: {ov['tool_median_mm']:.1f} mm "
          f"median, {ov['tool_p95_mm']:.1f} mm p95; slowest motion p95 "
          f"{by[0]['tool_p95_mm']:.1f} mm, fastest {by[-1]['tool_p95_mm']:.1f} mm")
    return rep


def test_generic_mapping(rows, rep):
    if rep is None:
        return
    g = TMP / "my_robot.csv"
    write_generic(rows, g)
    mp = {"time": "time_ms", "time_unit": "ms", "angle_unit": "deg",
          "target_q": [f"cmd{i}" for i in range(1, 7)], "q": [f"j{i}" for i in range(1, 7)],
          "tcp_pos": ["x", "y", "z"], "tcp_unit": "m", "tcp_rot": ["rx", "ry", "rz"]}
    mpath = TMP / "mapping.json"
    mpath.write_text(json.dumps(mp))
    out = TMP / "report_generic"
    assert cli.main(["intake", str(g), "--out", str(out), "--mapping", str(mpath),
                     "--menagerie", str(menagerie())]) == 0
    rep2 = json.loads((out / "report.json").read_text())
    a, b = rep["overall"]["tool_p95_mm"], rep2["overall"]["tool_p95_mm"]
    assert abs(a - b) < 0.05 * a, (a, b)
    print(f"  pass  the same data as a generic CSV (degrees, milliseconds) through a "
          f"mapping file gives the same answer ({a:.2f} / {b:.2f} mm p95)")


def test_plain_errors(rows):
    bad = TMP / "no_cmd.csv"
    bad.write_text("timestamp actual_q_0 actual_q_1\n0 0 0\n")
    try:
        read_log(bad, fmt="ur-rtde")
        raise AssertionError("accepted a log with no commanded joints")
    except IntakeError as e:
        assert "COMMANDED" in str(e), e
    g = TMP / "deg.csv"
    write_generic(rows, g)
    try:
        read_log(g)
        raise AssertionError("read an unknown CSV with no mapping")
    except IntakeError as e:
        assert "mapping" in str(e) and "target_q" in str(e)
    mp = {"time": "time_ms", "time_unit": "ms", "angle_unit": "rad",
          "target_q": [f"cmd{i}" for i in range(1, 7)], "q": [f"j{i}" for i in range(1, 7)]}
    tb = read_log(g, mapping=mp)
    try:
        check(tb, "ur5e")
        raise AssertionError("took degrees for radians")
    except IntakeError as e:
        assert "degrees" in str(e), e
    used = TMP / "used"
    used.mkdir()
    (used / "x.txt").write_text("x")
    try:
        run_intake(TMP / "deg.csv", used, mapping=mp, log=lambda *_: None)
        raise AssertionError("wrote into a folder in use")
    except IntakeError as e:
        assert "empty folder" in str(e)
    print("  pass  plain refusals: no commanded joints, an unknown CSV with no "
          "mapping (an example is shown), degrees read as radians, a folder in use")


def test_never_benchmarked(rep):
    if rep is None:
        return
    out = TMP / "report"
    lb = TMP / "lb.json"
    rc = cli.main(["score", "--real", str(out / "runs" / "real"),
                   "--sim", str(out / "runs" / "sim_s0"), "--out", str(lb)])
    doc = json.loads(lb.read_text()) if lb.exists() else {"entries": []}
    assert all(e.get("n_runs", 0) == 0 for e in doc.get("entries", [])), (rc, doc)
    print("  pass  user data is left out of benchmark scoring")


def test_page(rep):
    if rep is None:
        return
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("  skipped  the report page (playwright not installed)")
        return
    exe = os.environ.get("CHROMIUM", "/opt/pw-browsers/chromium")
    with sync_playwright() as pw:
        b = pw.chromium.launch(**({"executable_path": exe} if Path(exe).exists() else {}))
        pg = b.new_page(viewport={"width": 1100, "height": 900})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.goto((TMP / "report" / "report.html").as_uri())
        rows = pg.eval_on_selector_all("table tbody tr", "r => r.length")
        if os.environ.get("SHOT_DIR"):
            pg.screenshot(path=os.path.join(os.environ["SHOT_DIR"], "intake_report.png"),
                          full_page=True)
        b.close()
    assert not errs and rows >= 5 + 3, (errs, rows)
    print(f"  pass  report.html renders ({rows} table rows, no script error)")


def test_other_arms():
    men = menagerie()
    if men is None or not (men / "universal_robots_ur10e" / "ur10e.xml").exists():
        print("  skipped  UR10e (needs the menagerie's UR10e: python install_sim.py)")
    else:
        rows, n = make_log("ur10e")
        log = TMP / "ur10e_log.csv"
        write_rtde(rows, log)
        rep = run_intake(log, TMP / "rep10", robot="ur10e", menagerie=men,
                         log=lambda *_: None)
        assert not rep["problems"] and rep["overall"]["motions"] == n, rep["problems"]
        print(f"  pass  a UR10e log is replayed on the menagerie UR10e: "
              f"{rep['overall']['tool_p95_mm']:.1f} mm p95 over {n} motions")
    rows, n = make_log("ur3e")
    log = TMP / "ur3e_log.csv"
    write_rtde(rows, log)
    rep = run_intake(log, TMP / "rep3", robot="ur3e", menagerie=men,
                     log=lambda *_: None)
    assert rep["overall"] is None and len(rep["motions"]) == n
    assert any("UR3e" in p for p in rep["problems"]), rep["problems"]
    assert (TMP / "rep3" / "report.html").exists()
    print("  pass  a UR3e log is read, checked and cut, and the report says plainly "
          "that S0 has no UR3e model yet")


def main():
    rows, n = make_log()
    rtde = TMP / "ur5e_cell_log.csv"
    write_rtde(rows, rtde)
    test_reading_and_cutting(rtde, n)
    rep = test_full_report(rtde)
    test_generic_mapping(rows, rep)
    test_plain_errors(rows)
    test_never_benchmarked(rep)
    test_page(rep)
    test_other_arms()
    print("all passed")


if __name__ == "__main__":
    main()
