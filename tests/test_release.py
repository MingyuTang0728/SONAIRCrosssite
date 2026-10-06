"""
The public release gives nothing away.

A messy runs folder like a real one -- published and held-out cells, E1, E3,
a rejected run, a simulated-cell rehearsal, a superseded copy and the
continuous robot log -- is packaged, and:

  1. the public folder holds E1 in full, the published cells' real runs with
     their S0, and for the held-out cells ONLY the commanded trajectory and
     S0; no E3, no log, no rehearsal, no rejected or superseded run;
  2. the held-out command files carry no measured channel at all;
  3. the private set holds what scoring needs (held-out real + S0, and E3),
     and the harness scores straight from it;
  4. the independent check passes on the release -- and catches each way it
     could be spoilt afterwards: a held-out real run copied in, a measured
     field added to a command file, a log dropped in, a file edited;
  5. a release is never built over an existing one.

Run:  python tests/test_release.py
"""
from __future__ import annotations

import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from sonair_benchmark import cli                      # noqa: E402
from sonair_benchmark.release import (build_release, verify_release,  # noqa: E402
                                      held_out_cells, COMMAND_SAMPLE_KEYS)
from sonair_benchmark.schema import RunManifest, RunWriter, Sample, read_run  # noqa: E402

TMP = Path(tempfile.mkdtemp())


def write_run(folder: Path, run_id: str, cell: str, side="real", experiment="E2",
              traj=None, notes="", repeat=0, n=150):
    v, cfg, tt = cell.split("|")
    m = RunManifest(run_id=run_id, side=side, calib_version="t", joint_vel=float(v),
                    arm_config=cfg, traj_type=traj or tt, repeat_idx=repeat,
                    experiment=experiment, notes=notes, carrier_mass_kg=0.19)
    folder.mkdir(parents=True, exist_ok=True)
    with RunWriter(folder / f"{run_id}.jsonl", m) as w:
        for k in range(n):
            q = [0.0, -1.85, 1.85 - 0.3 * math.sin(k * 0.02), -1.57, -1.57, 0.0]
            w.write(Sample(t=k * 0.008, q=q, qd=[0.0] * 6, tcp_pos=[0.4, 0.1, 0.3],
                           tcp_rot=[0, 3.14, 0], target_q=q, target_qd=[0.0] * 6,
                           target_moment=[0.0, 20.0, 5.0, 1.0, 0.0, 0.0],
                           speed_scaling=1.0, robot_age_s=0.004,
                           aux={"controller_t": 1000 + k * 0.008},
                           imu={"ind0": {"gyro": [0.0, 0.0, 0.1],
                                         "accel": [0.0, 0.0, 9.81]}}))
    return m


def fake_s0(real: Path, sim: Path):
    """S0 stand-in: what sim_mujoco writes, without needing MuJoCo here."""
    for p in real.glob("*.jsonl"):
        r = read_run(p)
        m = r.manifest
        man = RunManifest(**{**m.__dict__, "side": "sim", "notes": "S0 test"})
        with RunWriter(sim / p.name, man) as w:
            for s in r.samples:
                w.write(Sample(t=s["t"], q=s["q"], tcp_pos=[0.401, 0.1, 0.3],
                               tcp_rot=[0, 3.14, 0], target_q=s["target_q"],
                               imu={"ind0": {"gyro": [0, 0, 0.1], "accel": [0, 0, 9.81]}}))


def main():
    held = sorted(held_out_cells())
    pub_cell = "0.200|mid_workspace|contour"
    assert pub_cell not in held
    hcell = held[0]
    runs = TMP / "bench_runs"
    write_run(runs, "pub_r00", pub_cell)
    write_run(runs, "pub_r01", pub_cell, repeat=1)
    write_run(runs, "held_r00", hcell)
    write_run(runs, "rejected_r00", pub_cell, repeat=2)            # not marked done
    write_run(runs, "e1_bare_mid_workspace_chirp_j3_r00", pub_cell, experiment="E1",
              traj="chirp")
    write_run(runs, "e3_arc_r00", pub_cell, experiment="E3")
    write_run(runs / "simcell", "rehearsal_r00", pub_cell, side="sim",
              notes="SIMULATED CELL (URSim controller + MuJoCo plant)")
    shutil.copy(runs / "held_r00.jsonl", runs / "held_r00.jsonl.20261006T120000.superseded")
    (runs / "ur_20261006_174129.csv").write_text("t_s,actual_q_base\n0,0\n")
    sim = TMP / "sim_s0"
    sim.mkdir()
    fake_s0(runs, sim)
    state = TMP / "state.json"
    state.write_text(json.dumps({"done": {k: {} for k in
                                          ("pub_r00", "pub_r01", "held_r00", "e3_arc_r00")},
                                 "e1": {"done": {"e1_bare_mid_workspace_chirp_j3_r00": {}}}}))

    out = TMP / "release" / "v1"
    res = build_release(runs, out, sim_dir=sim, state_path=state, make_s0=False,
                        version="v1-test", log=lambda *_: None)
    assert res["ok"], res["problems"]
    c = res["counts"]
    assert c == {"E1": 1, "E2_published": 2, "E2_heldout": 1, "E3": 1}, c
    pub_files = sorted(str(p.relative_to(out)) for p in out.rglob("*") if p.is_file())
    names = " ".join(pub_files)
    for bad in ("rehearsal", "rejected", "superseded", "ur_2026", "e3_arc",
                "heldout/real", "held_r00.jsonl.2026"):
        assert bad not in names, (bad, pub_files)
    assert "E2/heldout/commands/held_r00.jsonl" in names
    assert any("rejected_r00" in s for s in res["skipped"])
    print(f"  pass  public release: E1 {c['E1']}, published {c['E2_published']} real+S0, "
          f"held-out {c['E2_heldout']} as commands+S0; no E3, log, rehearsal, "
          f"rejected or superseded run")

    lines = (out / "E2/heldout/commands/held_r00.jsonl").read_text().splitlines()
    rows = [json.loads(x) for x in lines[1:]]
    keys = set().union(*rows)
    assert keys <= set(COMMAND_SAMPLE_KEYS) | {"aux"}, keys
    assert "q" not in keys and "imu" not in keys and "tcp_pos" not in keys
    assert json.loads(lines[0])["_manifest"]["side"] == "commands"
    assert len(rows) == 150 and rows[0]["aux"] == {"controller_t": 1000.0}
    print(f"  pass  held-out command files carry only {sorted(keys)} -- nothing measured")

    priv = Path(res["private"])
    assert (priv / "E2/heldout/real/held_r00.jsonl").exists()
    assert (priv / "E3/real/e3_arc_r00.jsonl").exists()
    lb = TMP / "lb.json"
    assert cli.main(["score", "--real", str(out / "E2/published/real"),
                     str(priv / "E2/heldout/real"),
                     "--sim", str(out / "E2/published/sim_s0"),
                     str(priv / "E2/heldout/sim_s0"),
                     "--plan", str(out / "plan.json"), "--out", str(lb)]) == 0
    doc = json.loads(lb.read_text())
    b1 = next(e for e in doc["entries"] if e["name"].startswith("B1"))
    assert b1["n_runs"] == 1, b1          # scored on the held-out run only
    # fitted on the published cells (offset 1 mm in x) -- not on the answer
    assert b1["gcr_p95"] > 0.5, b1
    assert verify_release(priv), "the private set must never pass as public"
    print("  pass  the private set holds the held-out real runs and E3, scores "
          "directly, and is refused as a public release")

    assert verify_release(out) == []
    spoilt = []
    # a held-out real run copied into the public folder
    t1 = TMP / "t1"; shutil.copytree(out, t1)
    shutil.copy(runs / "held_r00.jsonl", t1 / "E2/published/real/held_r00.jsonl")
    spoilt.append(("held-out real copied in", verify_release(t1), "REAL run of a held-out"))
    # a measured channel added to a command file
    t2 = TMP / "t2"; shutil.copytree(out, t2)
    f = t2 / "E2/heldout/commands/held_r00.jsonl"
    ls = f.read_text().splitlines()
    row = json.loads(ls[5]); row["q"] = [0] * 6; ls[5] = json.dumps(row)
    f.write_text("\n".join(ls) + "\n")
    spoilt.append(("measured field in a command file", verify_release(t2), "measured data"))
    # a log dropped in
    t3 = TMP / "t3"; shutil.copytree(out, t3)
    shutil.copy(runs / "ur_20261006_174129.csv", t3 / "E1" / "ur_20261006_174129.csv")
    spoilt.append(("robot log dropped in", verify_release(t3), "logs and state never go public"))
    # a published file edited after the build
    t4 = TMP / "t4"; shutil.copytree(out, t4)
    g = next((t4 / "E2/published/sim_s0").glob("*.jsonl"))
    g.write_text(g.read_text() + "\n")
    spoilt.append(("file edited after build", verify_release(t4), "checksum"))
    for what, probs, expect in spoilt:
        assert any(expect in p for p in probs), (what, probs)
    print("  pass  the check passes on the release and catches each spoiling: "
          + ", ".join(w for w, _, _ in spoilt))

    try:
        build_release(runs, out, sim_dir=sim, state_path=state, make_s0=False,
                      log=lambda *_: None)
        raise AssertionError("built over an existing release")
    except FileExistsError:
        pass
    assert cli.main(["verify-release", str(out)]) == 0
    print("  pass  never built over an existing release; verify-release CLI agrees")
    print("all passed")


if __name__ == "__main__":
    main()
