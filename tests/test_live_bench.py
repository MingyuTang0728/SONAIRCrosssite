"""
The benchmark, scored live, agrees with the benchmark scored offline.

Two recorded runs (an elbow move at two speeds, from an arm that tracks its
command closely) are played into the digital twin packet by packet, exactly
as the robot link delivers them, with S0 and a CANDIDATE model (the menagerie
UR5e with a stiffer servo) running side by side. The same run files are then
replayed and scored by the offline harness (sim_mujoco + the `simulate` and
`score` commands).

  1. the live scorer scores each run the moment it ends, under its own cell,
     with S0 as the reference and a GCR for the candidate;
  2. the live GCR-p95 of each run agrees with the offline one;
  3. the session aggregate is the mean over runs, per cell and overall;
  4. simulated-cell rehearsals are kept apart; a run too short to score is
     not scored;
  5. the console shows it: the live table, the session line, no script error.

Run:  python tests/test_live_bench.py   (needs mujoco + a menagerie clone;
                                          5 needs playwright)
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))

import live_bench                                     # noqa: E402

TMP = Path(tempfile.mkdtemp())


def menagerie():
    for d in (os.environ.get("MENAGERIE"), ROOT / "mujoco_menagerie",
              Path.home() / ".sonair" / "mujoco_menagerie"):
        if d and (Path(d) / "universal_robots_ur5e" / "scene.xml").exists():
            return Path(d)
    return None


class Feed:
    def __init__(self):
        self.sinks = []

    def subscribe(self, fn):
        self.sinks.append(fn)

    def unsubscribe(self, fn):
        self.sinks.remove(fn)

    def push(self, st):
        for fn in list(self.sinks):
            fn(st)


def test_scoring_rules():
    lb = live_bench.LiveBench(out_dir=TMP / "rules")
    run = {"run_id": "r1", "cell": "0.300|mid_workspace|point_to_point",
           "experiment": "E2", "simulated": False}
    for k in range(100):                    # S0 off by 10 mm, candidate by 4
        lb.add(k * 0.03, {"S0": 10.0 + (k % 5), "cand": 4.0 + (k % 5) * 0.4}, run)
    lb.add(3.1, {"S0": 9.0, "cand": 4.0}, None)
    short = {"run_id": "r2", "cell": run["cell"], "experiment": "E2", "simulated": False}
    for k in range(5):
        lb.add(4 + k * 0.03, {"S0": 9.0, "cand": 4.0}, short)
    lb.run_ended()
    sim = {"run_id": "s1", "cell": run["cell"], "experiment": "E2", "simulated": True}
    for k in range(60):
        lb.add(6 + k * 0.03, {"S0": 2.0, "cand": 1.0}, sim)
    lb.run_ended()
    snap = lb.snapshot()
    runs = snap["session"]["runs"]
    assert [r["run_id"] for r in runs] == ["r1"], runs          # r2 too short
    m = runs[0]["models"]
    exp = 1 - live_bench.percentile([4.0 + (k % 5) * 0.4 for k in range(100)], 95) / \
        live_bench.percentile([10.0 + (k % 5) for k in range(100)], 95)
    assert abs(m["cand"]["gcr_p95"] - exp) < 1e-3, (m, exp)
    assert [r["run_id"] for r in snap["rehearsal"]["runs"]] == ["s1"]
    assert snap["session"]["overall"]["cand"]["runs"] == 1
    assert json.loads((TMP / "rules" / "live_score.json").read_text())["session"]["runs"]
    print(f"  pass  live rules: each run scored when it ends (GCR-p95 "
          f"{m['cand']['gcr_p95']:.3f}), short runs skipped, rehearsals kept "
          f"apart, written to live_score.json")


def test_live_agrees_with_offline():
    men = menagerie()
    try:
        import mujoco  # noqa: F401
    except ImportError:
        men = None
    if men is None:
        print("  skipped  live vs offline (needs mujoco and a menagerie clone)")
        return None
    os.environ["MENAGERIE"] = str(men)
    from test_benchmark_tracks import ideal_runs
    from sonair_benchmark import cli
    from sonair_benchmark.schema import read_run
    import twin as twin_mod

    d = men / "universal_robots_ur5e"
    stiff = TMP / "team_stiff" / "ur5e_stiff.xml"
    stiff.parent.mkdir()
    for f in d.iterdir():                     # meshes travel with the model
        if f.is_dir():
            (stiff.parent / f.name).symlink_to(f)
    stiff.write_text((d / "ur5e.xml").read_text()
                     .replace('gainprm="2000" biasprm="0 -2000 -400"',
                              'gainprm="20000" biasprm="0 -20000 -1500"')
                     .replace('gainprm="500" biasprm="0 -500 -100"',
                              'gainprm="5000" biasprm="0 -5000 -350"'))
    real = TMP / "real"
    real.mkdir()
    ideal_runs(real)

    # --- offline, by the harness ------------------------------------------
    import sim_mujoco
    s0, team = TMP / "sim_s0", TMP / "sim_team"
    assert sim_mujoco.main(["--real", str(real), "--out", str(s0),
                            "--menagerie", str(men), "--imu-cal", "none"]) == 0
    assert cli.main(["simulate", "--model", str(stiff), "--real", str(real),
                     "--out", str(team), "--menagerie", str(men)]) == 0
    from sonair_benchmark.scoring import score_run, submission_from_sim_runs
    from sonair_benchmark.schema import read_dataset
    reals = {r.manifest.run_id: r for r in read_dataset(real, side="real")}
    sims = {r.manifest.run_id: r for r in read_dataset(s0, side="sim")}
    cands = read_dataset(team, side="sim")
    pairs = [(reals[k], sims[k]) for k in sorted(reals)]
    sub = submission_from_sim_runs(cands, pairs)
    offline = {k: score_run(reals[k], sims[k], sub[k]).gcr_p95 for k in reals}

    # --- live, packet by packet through the twin ----------------------------
    live_bench.LIVE = live_bench.LiveBench(out_dir=TMP / "live")
    tw = twin_mod.Twin()
    assert tw.load(0.0), tw.why
    assert tw.set_candidate(str(stiff), "Team Stiff")["ok"]
    cur = {"run": None}
    tw.run_fn = lambda: cur["run"]
    feed = Feed()
    assert tw.start(feed)["ok"]
    t_off = 0.0
    for k in sorted(reals):
        r = read_run(real / f"{k}.jsonl")
        cur["run"] = {"run_id": k, "cell": r.manifest.cell_key(),
                      "experiment": "E2", "simulated": False}
        for s in r.samples:
            feed.push({"timestamp": 1000 + t_off + s["t"], "target_q": s["target_q"],
                       "actual_q": s["q"],
                       "actual_TCP_pose": list(s["tcp_pos"]) + list(s["tcp_rot"])})
        t_off += r.samples[-1]["t"] + 1.0     # a pause: the twin restarts
        cur["run"] = None
        live_bench.LIVE.run_ended()
    tw.stop()
    snap = live_bench.LIVE.snapshot()
    live = {r["run_id"]: r["models"]["Team Stiff"]["gcr_p95"]
            for r in snap["session"]["runs"]}
    assert set(live) == set(offline), (live, offline)
    for k in offline:
        assert abs(live[k] - offline[k]) < 0.05, (k, live[k], offline[k])
    ov = snap["session"]["overall"]["Team Stiff"]["gcr_p95"]
    print("  pass  live GCR-p95 agrees with the offline harness: "
          + ", ".join(f"{k.split('_')[0]} live {live[k]:.2f} / offline "
                      f"{offline[k]:.2f}" for k in sorted(offline))
          + f"; session {ov:.2f}")
    return snap


def test_console(snap):
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("  skipped  the console (playwright not installed)")
        return
    if snap is None:
        snap = live_bench.LiveBench(out_dir=TMP / "x").snapshot()
    exe = os.environ.get("CHROMIUM", "/opt/pw-browsers/chromium")
    with sync_playwright() as pw:
        b = pw.chromium.launch(**({"executable_path": exe} if Path(exe).exists() else {}))
        pg = b.new_page(viewport={"width": 1600, "height": 1000})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.goto((ROOT / "SONAIR_Console.html").as_uri())
        pg.wait_for_timeout(800)
        pg.evaluate("""s => {
            document.getElementById('twinBox').hidden = false;
            window.__sonairTest && window.__sonairTest(s);
        }""", snap)
        has_hook = pg.evaluate("typeof window.__sonairLive === 'function'")
        if has_hook:
            pg.evaluate("s => window.__sonairLive(s)", snap)
        rows = pg.eval_on_selector_all("#lbBody tr", "r => r.map(x => x.innerText)")
        if os.environ.get("SHOT_DIR"):
            pg.locator(".monitor").screenshot(
                path=os.path.join(os.environ["SHOT_DIR"], "console_live.png"))
        sess = pg.inner_text("#lbSess")
        b.close()
    assert not errs, errs
    assert has_hook, "console exposes no hook for the live view"
    assert len(rows) == 2 and rows[0].startswith("S0"), rows
    assert "This session: 2 runs" in sess and "GCR-p95" in sess, sess
    print(f"  pass  console: live table {rows}, session line shown, no script error")


def test_bench_page(snap):
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("  skipped  benchmark.html Live tab (playwright not installed)")
        return
    import shutil
    import subprocess
    import time
    site = TMP / "site"
    (site / "results").mkdir(parents=True)
    shutil.copy(ROOT / "benchmark.html", site / "benchmark.html")
    doc = snap or live_bench.LiveBench(out_dir=TMP / "y").snapshot()
    (site / "results" / "live_score.json").write_text(json.dumps(doc))
    port = 8897
    http = subprocess.Popen([sys.executable, "-m", "http.server", str(port),
                             "--bind", "127.0.0.1"], cwd=site,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    exe = os.environ.get("CHROMIUM", "/opt/pw-browsers/chromium")
    try:
        with sync_playwright() as pw:
            b = pw.chromium.launch(**({"executable_path": exe} if Path(exe).exists() else {}))
            pg = b.new_page(viewport={"width": 1280, "height": 900})
            errs = []
            pg.on("pageerror", lambda e: errs.append(str(e)))
            pg.goto(f"http://127.0.0.1:{port}/benchmark.html")
            pg.click("nav.tabs button[data-tab='live']")
            pg.wait_for_timeout(1200)
            now = pg.eval_on_selector_all("#liveNow tbody tr", "r => r.length")
            cells = pg.eval_on_selector_all("#liveCells tbody tr", "r => r.map(x => x.innerText)")
            stats = pg.inner_text("#liveStats")
            if os.environ.get("SHOT_DIR"):
                pg.screenshot(path=os.path.join(os.environ["SHOT_DIR"], "bench_live.png"), full_page=True)
            b.close()
    finally:
        http.terminate()
    assert not errs, errs
    if snap is not None:
        assert now == 2 and len(cells) == 2 and "RUNS SCORED" in stats.upper(), (now, cells, stats)
    print(f"  pass  benchmark.html Live tab reads results/live_score.json: "
          f"{now} models now, {len(cells)} cells this session")


def main():
    test_scoring_rules()
    snap = test_live_agrees_with_offline()
    test_console(snap)
    test_bench_page(snap)
    print("all passed")


if __name__ == "__main__":
    main()
