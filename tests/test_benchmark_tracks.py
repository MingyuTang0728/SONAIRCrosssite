"""
The benchmark's two tracks, end to end, and the page that shows them.

  1. the demo writes a leaderboard with both tracks, the S0 reference at 0 and
     each entry labelled baseline / example / submission;
  2. Track A for real: recorded runs are replayed through S0 and through a
     SUBMITTED MuJoCo model (the menagerie UR5e with a stiffer servo) by the
     same harness, the submission's runs say which model made them, and it
     scores above S0 against an arm that tracks its command closely;
  3. Track B prediction files still score as before;
  4. benchmark.html loads those results with no script error, shows both
     tracks, and filters by track.

Run:  python tests/test_benchmark_tracks.py   (2 needs mujoco + MENAGERIE,
                                               4 needs playwright)
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from sonair_benchmark import cli                      # noqa: E402
from sonair_benchmark.schema import RunManifest, RunWriter, Sample, read_run  # noqa: E402

TMP = Path(tempfile.mkdtemp())


def menagerie():
    for d in (os.environ.get("MENAGERIE"), ROOT / "mujoco_menagerie",
              Path.home() / ".sonair" / "mujoco_menagerie"):
        if d and (Path(d) / "universal_robots_ur5e" / "scene.xml").exists():
            return Path(d)
    return None


def test_demo_tracks():
    out = TMP / "demo"
    assert cli.main(["demo", "--out", str(out)]) == 0
    doc = json.loads((out / "site" / "leaderboard.json").read_text())
    by = {e["name"]: e for e in doc["entries"]}
    tracks = {e["track"] for e in doc["entries"]}
    assert tracks == {"A", "B"}, tracks
    s0 = next(e for e in doc["entries"] if e["name"].startswith("S0"))
    assert s0["kind"] == "baseline" and abs(s0["gcr_p95"]) < 1e-12
    assert doc["tracks"]["A"] and doc["dataset"]["held_out_cells"] > 0
    ex = [e for e in doc["entries"] if e["kind"] == "example"]
    assert len(ex) == 2 and all(e["gcr_p95"] > 0 for e in ex), by
    print(f"  pass  demo leaderboard: tracks A and B, S0 at 0, "
          f"{len(doc['entries'])} entries labelled by kind")
    return out


def ideal_runs(folder: Path):
    """Elbow moves recorded from an arm that tracks its command exactly."""
    import ur_kin
    q0 = [0.0, -1.85, 1.85, -1.57, -1.57, 0.0]
    for v in (0.3, 0.9):
        a, amp = 3.0, math.radians(45)
        ramp = v / a; cruise = (amp - v * v / a) / v; T = 2 * ramp + cruise
        m = RunManifest(run_id=f"v{v:.2f}_mid_workspace_point_to_point_r00".replace(".", "p"),
                        side="real", calib_version="t", joint_vel=v,
                        arm_config="mid_workspace", traj_type="point_to_point",
                        repeat_idx=0)
        with RunWriter(folder / f"{m.run_id}.jsonl", m) as w:
            for k in range(int((T + 1.0) / 0.008)):
                tt = min(max(k * 0.008 - 0.4, 0.0), T)
                s = (0.5 * a * tt * tt if tt < ramp else
                     0.5 * v * ramp + v * (tt - ramp) if tt < ramp + cruise else
                     amp - 0.5 * a * (T - tt) ** 2)
                q = list(q0); q[2] -= s
                p = ur_kin.fk_pose(q)
                w.write(Sample(t=k * 0.008, q=q, tcp_pos=p[:3], tcp_rot=p[3:6],
                               target_q=q, target_qd=[0.0] * 6))


def test_track_a():
    men = menagerie()
    try:
        import mujoco  # noqa: F401
    except ImportError:
        men = None
    if men is None:
        print("  skipped  Track A replay (needs mujoco and a menagerie clone)")
        return None
    d = men / "universal_robots_ur5e"
    stiff = d / "_test_team_stiff.xml"
    stiff.write_text((d / "ur5e.xml").read_text()
                     .replace('gainprm="2000" biasprm="0 -2000 -400"',
                              'gainprm="20000" biasprm="0 -20000 -1500"')
                     .replace('gainprm="500" biasprm="0 -500 -100"',
                              'gainprm="5000" biasprm="0 -5000 -350"'))
    real, s0, team = TMP / "real", TMP / "sim_s0", TMP / "sim_team"
    real.mkdir()
    ideal_runs(real)
    import sim_mujoco
    assert sim_mujoco.main(["--real", str(real), "--out", str(s0),
                            "--menagerie", str(men), "--imu-cal", "none"]) == 0
    assert cli.main(["simulate", "--model", str(stiff), "--real", str(real),
                     "--out", str(team), "--menagerie", str(men)]) == 0
    one = read_run(next(team.glob("*.jsonl")))
    assert "_test_team_stiff.xml" in one.manifest.notes, one.manifest.notes
    lb = TMP / "site" / "leaderboard.json"
    assert cli.main(["score", "--real", str(real), "--sim", str(s0),
                     "--track-a", f"Team Stiff={team}", "--out", str(lb)]) == 0
    doc = json.loads(lb.read_text())
    e = next(x for x in doc["entries"] if x["name"] == "Team Stiff")
    assert e["track"] == "A" and e["gcr_p95"] > 0.3, e
    stiff.unlink()
    print(f"  pass  Track A: a submitted MuJoCo model replayed by the same "
          f"harness as S0 scores GCR-p95 {e['gcr_p95']:.2f} against it")
    return lb


def test_track_b(demo: Path):
    from sonair_benchmark.scoring import write_submission, SubmissionRun
    from sonair_benchmark.schema import read_dataset
    sims = read_dataset(demo / "data" / "sim", side="sim")
    sub = TMP / "team_b.jsonl"
    write_submission(sub, [SubmissionRun(
        run_id=r.manifest.run_id, t=[s["t"] for s in r.samples],
        tcp_pos=[s["tcp_pos"] for s in r.samples], mode="absolute") for r in sims])
    lb = TMP / "site_b" / "leaderboard.json"
    assert cli.main(["score", "--real", str(demo / "data" / "real"),
                     "--sim", str(demo / "data" / "sim"),
                     "--plan", str(demo / "campaign" / "plan.json"),
                     "--track-b", f"Team B={sub}", "--out", str(lb)]) == 0
    e = next(x for x in json.loads(lb.read_text())["entries"] if x["name"] == "Team B")
    assert e["track"] == "B" and abs(e["gcr_p95"]) < 1e-9, e     # it IS S0
    print("  pass  Track B: a prediction file scores as before (S0 handed back "
          "scores 0)")


def test_page(demo: Path):
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("  skipped  the page (playwright not installed)")
        return
    shutil.copy(ROOT / "benchmark.html", demo / "benchmark.html")
    port = 8890
    http = subprocess.Popen([sys.executable, "-m", "http.server", str(port),
                             "--bind", "127.0.0.1"], cwd=demo,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    exe = os.environ.get("CHROMIUM", "/opt/pw-browsers/chromium")
    try:
        with sync_playwright() as pw:
            b = pw.chromium.launch(**({"executable_path": exe} if Path(exe).exists() else {}))
            pg = b.new_page()
            errs = []
            pg.on("pageerror", lambda e: errs.append(str(e)))
            pg.goto(f"http://127.0.0.1:{port}/benchmark.html")
            pg.wait_for_timeout(1500)
            rows = pg.eval_on_selector_all("#lbTable tbody tr", "r => r.length")
            badges = set(pg.eval_on_selector_all("#lbTable tbody .badge.A, #lbTable tbody .badge.B",
                                                 "r => r.map(x => x.textContent)"))
            pg.click("#trackFilter button[data-t='A']")
            only_a = pg.eval_on_selector_all("#lbTable tbody tr .badge.B", "r => r.length")
            for t in ("tracks", "data", "gap", "floor", "submit"):
                pg.click(f"nav.tabs button[data-tab='{t}']")
            heat = pg.eval_on_selector_all("#gapHeatmap td.cell", "r => r.length")
            b.close()
    finally:
        http.terminate()
    assert not errs, errs
    assert rows == 4 and badges == {"A", "B"} and only_a == 0, (rows, badges, only_a)
    assert heat > 0
    print(f"  pass  benchmark.html: {rows} entries over both tracks, the track "
          f"filter works, the gap map draws {heat} cells, no script errors")


def main():
    demo = test_demo_tracks()
    test_track_a()
    test_track_b(demo)
    test_page(demo)
    print("all passed")


if __name__ == "__main__":
    main()
