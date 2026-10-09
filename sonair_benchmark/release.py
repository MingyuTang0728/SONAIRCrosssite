"""
The public release of the dataset -- and the check that it gives nothing away.

A benchmark is only as good as its held-out set is held out. If the real runs
of a held-out cell reach a team in any form -- the run file, a log that
covers the same minutes, a field copied into a simulated file -- every score
on those cells measures memory, not generalisation, and nobody can tell
afterwards. So the release is built by a whitelist and then checked by an
independent read-back that refuses it on any doubt.

    python -m sonair_benchmark release --runs bench_runs --out release/v1
    python -m sonair_benchmark verify-release release/v1

What goes where:

  PUBLIC  release/v1/
    E1/real/                 the identification set, in full (training)
    E2/published/real/       published cells: the real runs
    E2/published/sim_s0/     ... and the reference simulation of each
    E2/heldout/commands/     held-out cells: the COMMANDED trajectory only
    E2/heldout/sim_s0/       ... and the reference simulation of each
    plan.json                the campaign plan, published/held-out cell lists
    manifest.json            versions, counts, and a SHA-256 for every file
    README.md

  PRIVATE release/v1_PRIVATE/  -- never published; what the scorer needs
    E2/heldout/real/         the held-out real runs
    E3/real/                 the out-of-distribution set
    manifest.json

Never in either: the continuous robot and IMU logs (they cover every run,
held out or not), campaign state, superseded or rejected runs, and rehearsal
runs from the simulated cell.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import time
from pathlib import Path

from . import SCHEMA_VERSION
from .campaign import plan_campaign, split_cells
from .schema import read_run

# What a held-out COMMAND file may carry. Everything here is what the
# controller was told or generated from what it was told; nothing is what the
# arm or a sensor measured.
COMMAND_SAMPLE_KEYS = ("t", "target_q", "target_qd", "target_moment", "speed_scaling")
COMMAND_AUX_KEYS = ("controller_t",)
# Measured channels: never in a command file, never in anything public from
# a held-out cell except the reference simulation's own (simulated) values.
MEASURED_KEYS = ("q", "qd", "tcp_pos", "tcp_rot", "imu", "em", "sensors",
                 "robot_age_s")
PUBLIC_TOP = {"E1", "E2", "plan.json", "manifest.json", "README.md"}


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_commit(root: Path) -> str:
    try:
        r = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True, timeout=10)
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:       # noqa: BLE001
        return ""


def _is_rehearsal(run) -> bool:
    return str(run.manifest.notes).startswith("SIMULATED CELL")


def held_out_cells(plan_path: str | Path | None = None) -> set[str]:
    """The held-out cells: from the plan file, or from the plan's own fixed split."""
    if plan_path and Path(plan_path).exists():
        doc = json.loads(Path(plan_path).read_text(encoding="utf-8"))
        cells = doc.get("meta", {}).get("held_out_cells")
        if cells:
            return set(cells)
    _pub, held = split_cells(plan_campaign())
    return held


def _done_ids(state_path) -> set[str] | None:
    """Runs the campaign marked done (read back and accepted), if a state is given."""
    if not state_path or not Path(state_path).exists():
        return None
    st = json.loads(Path(state_path).read_text(encoding="utf-8"))
    ids = set(st.get("done", {})) | set((st.get("e1") or {}).get("done", {}))
    return ids


def _command_file(src: Path, dst: Path) -> None:
    """A run file reduced to what the controller was COMMANDED."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    with src.open(encoding="utf-8") as fi, dst.open("w", encoding="utf-8") as fo:
        for i, line in enumerate(fi):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue            # a truncated last line of a killed capture
            if i == 0 and "_manifest" in obj:
                man = dict(obj["_manifest"])
                man["side"] = "commands"
                man["notes"] = ("held-out cell: the commanded trajectory only; "
                                "the real run is kept for scoring")
                fo.write(json.dumps({"_manifest": man}) + "\n")
                continue
            row = {k: obj[k] for k in COMMAND_SAMPLE_KEYS if k in obj}
            aux = {k: v for k, v in (obj.get("aux") or {}).items() if k in COMMAND_AUX_KEYS}
            if aux:
                row["aux"] = aux
            if "target_q" in row:
                fo.write(json.dumps(row) + "\n")


def build_release(runs_dir, out, sim_dir=None, plan_path=None, state_path=None,
                  private_out=None, menagerie=None, make_s0: bool = True,
                  version: str = "", log=print, tcp_offset=None,
                  imu_cal_path: str = "calib/imu_cal.json") -> dict:
    """
    Build the public release and its private scoring set. Returns a summary;
    raises nothing for a run that cannot be used -- it is listed, in words.
    """
    runs_dir, out = Path(runs_dir), Path(out)
    private = Path(private_out) if private_out else out.with_name(out.name + "_PRIVATE")
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"{out} is not empty; a release is built into a new folder")
    held = held_out_cells(plan_path)
    done = _done_ids(state_path)
    skipped: list[str] = []

    # what is there: run files only, never logs, never superseded copies
    real_runs = []
    for p in sorted(runs_dir.rglob("*.jsonl")):
        if "simcell" in p.parts:
            continue
        try:
            r = read_run(p)
        except Exception as e:      # noqa: BLE001
            skipped.append(f"{p.name}: unreadable ({e})")
            continue
        if r.manifest.side != "real" or _is_rehearsal(r):
            continue
        if done is not None and r.manifest.run_id not in done:
            skipped.append(f"{r.manifest.run_id}: not marked done in the campaign "
                           "(rejected, or recorded under a superseded protocol)")
            continue
        if not any("target_q" in s for s in r.samples):
            skipped.append(f"{r.manifest.run_id}: no commanded trajectory")
            continue
        real_runs.append((p, r))
    if done is None:
        log("WARNING: no campaign state given (--state), so runs are not checked "
            "against what the campaign accepted")

    # one file per run id; if a run was recorded twice, the later one
    by_id = {}
    for p, r in real_runs:
        if r.manifest.run_id in by_id:
            skipped.append(f"{r.manifest.run_id}: a second copy ({p}) ignored")
            continue
        by_id[r.manifest.run_id] = (p, r)

    # the reference simulation (S0) of every E2 run
    sim_index = {}
    if sim_dir and Path(sim_dir).exists():
        for p in Path(sim_dir).rglob("*.jsonl"):
            try:
                s = read_run(p)
            except Exception:       # noqa: BLE001
                continue
            if s.manifest.side == "sim" and not _is_rehearsal(s):
                sim_index[s.manifest.run_id] = p

    def s0_for(rid, path, run) -> Path | None:
        if rid in sim_index:
            return sim_index[rid]
        if not make_s0:
            return None
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        import sim_mujoco
        if not sim_mujoco.available()[0]:
            return None
        men = Path(menagerie) if menagerie else None
        if men is None:
            import twin
            men = next((d for d in twin.menagerie_dirs()
                        if (d / sim_mujoco.MODEL_DIR / "scene.xml").exists()), None)
        if men is None:
            return None
        tmp = private / "_s0_work"
        imu_cal = None
        try:
            import imu_align
            imu_cal = imu_align.load(imu_cal_path)
        except Exception:       # noqa: BLE001
            pass
        # S0 made here exactly as the scored replay makes it: the pendant's
        # tool offset, the IMU's measured mounting
        res = sim_mujoco.replay(run, tmp, men, tcp_offset=tcp_offset, imu_cal=imu_cal)
        if not res.get("ok"):
            why_s0[rid] = res.get("error", "")
            return None
        return tmp / f"{rid}.jsonl"

    why_s0: dict[str, str] = {}
    counts = {"E1": 0, "E2_published": 0, "E2_heldout": 0, "E3": 0}
    for rid, (p, r) in sorted(by_id.items()):
        exp = r.manifest.experiment
        cell = r.manifest.cell_key()
        if exp == "E1":
            _copy(p, out / "E1" / "real" / p.name)
            counts["E1"] += 1
        elif exp == "E3":
            _copy(p, private / "E3" / "real" / p.name)
            counts["E3"] += 1
        elif exp == "U":
            skipped.append(f"{rid}: a user's own data (intake), never released")
            continue
        else:
            s0 = s0_for(rid, p, r)
            if s0 is None:
                skipped.append(f"{rid}: no S0 simulation -- " + (
                    why_s0.get(rid) or "pass --sim, or install MuJoCo so it can be made"))
                continue
            if cell in held:
                _command_file(p, out / "E2" / "heldout" / "commands" / p.name)
                _copy(s0, out / "E2" / "heldout" / "sim_s0" / p.name)
                _copy(p, private / "E2" / "heldout" / "real" / p.name)
                _copy(s0, private / "E2" / "heldout" / "sim_s0" / p.name)
                counts["E2_heldout"] += 1
            else:
                _copy(p, out / "E2" / "published" / "real" / p.name)
                _copy(s0, out / "E2" / "published" / "sim_s0" / p.name)
                counts["E2_published"] += 1
    shutil.rmtree(private / "_s0_work", ignore_errors=True)

    # the plan, with the split stated
    pub = None
    if plan_path and Path(plan_path).exists():
        pub = json.loads(Path(plan_path).read_text(encoding="utf-8")).get(
            "meta", {}).get("published_cells")
    if not pub:
        pub, _h = split_cells(plan_campaign())
    plan = {"meta": {"held_out_cells": sorted(held),
                     "published_cells": sorted(set(pub) - held),
                     "note": "held-out cells: commands and S0 released, real runs kept"}}
    (out / "plan.json").write_text(json.dumps(plan, indent=2), encoding="utf-8")

    version = version or time.strftime("v%Y%m%d")
    root = Path(__file__).resolve().parent.parent
    try:
        import campaign_runner as cr
        protocol = cr.PROTOCOL
    except Exception:       # noqa: BLE001
        protocol = None
    meta = {"release": version, "built_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "schema": SCHEMA_VERSION, "protocol": protocol,
            "harness_commit": _git_commit(root),
            "counts": counts, "held_out_cells": len(held),
            "reference_simulator": "S0: MuJoCo, menagerie UR5e, unchanged",
            "skipped": skipped}
    (out / "README.md").write_text(_readme(meta), encoding="utf-8")
    private.mkdir(parents=True, exist_ok=True)
    (private / "README.md").write_text(
        "# PRIVATE scoring set -- do not publish\n\nHeld-out real runs (E2) and the "
        "out-of-distribution set (E3) for release " + version + ". Score with the "
        "public release beside it, so the fitted baseline (B1) is fitted on the "
        "published cells and scored on the held-out ones:\n\n"
        "    python -m sonair_benchmark score \\\n"
        "        --real PUBLIC/E2/published/real PRIVATE/E2/heldout/real \\\n"
        "        --sim  PUBLIC/E2/published/sim_s0 PRIVATE/E2/heldout/sim_s0 \\\n"
        "        --plan PUBLIC/plan.json --track-a ... --track-b ...\n",
        encoding="utf-8")
    # checksums last, over everything written
    _write_manifest(out, {**meta, "kind": "public"})
    _write_manifest(private, {**meta, "kind": "PRIVATE -- never publish"})
    problems = verify_release(out)
    return {"ok": not problems, "out": str(out), "private": str(private),
            "counts": counts, "skipped": skipped, "problems": problems}


def _copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)


def _write_manifest(folder: Path, meta: dict) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    files = {}
    for p in sorted(folder.rglob("*")):
        if p.is_file() and p.name != "manifest.json":
            files[str(p.relative_to(folder)).replace("\\", "/")] = _sha256(p)
    (folder / "manifest.json").write_text(json.dumps({**meta, "files": files},
                                                     indent=2), encoding="utf-8")


def verify_release(folder) -> list[str]:
    """
    Read a public release back, independently of how it was built, and list
    every way it could give away what it must not. Empty means safe to publish.
    """
    folder = Path(folder)
    out: list[str] = []
    mpath = folder / "manifest.json"
    if not mpath.exists():
        return ["no manifest.json"]
    meta = json.loads(mpath.read_text(encoding="utf-8"))
    if meta.get("kind") != "public":
        out.append(f"this folder is marked {meta.get('kind')!r}, not public")
    plan = folder / "plan.json"
    held = set(json.loads(plan.read_text())["meta"]["held_out_cells"]) if plan.exists() else set()
    if not held:
        out.append("no held-out cell list in plan.json")

    for p in folder.iterdir():
        if p.name not in PUBLIC_TOP:
            out.append(f"unexpected item at the top level: {p.name}")
    listed = meta.get("files", {})
    for p in folder.rglob("*"):
        if not p.is_file() or p.name == "manifest.json":
            continue
        rel = str(p.relative_to(folder)).replace("\\", "/")
        if p.suffix.lower() in (".csv", ".log", ".bak") or "superseded" in p.name \
                or p.name.startswith(("ur_", "imu_", "simcell_", "state")):
            out.append(f"{rel}: not a run file -- logs and state never go public")
        if rel not in listed:
            out.append(f"{rel}: not in the manifest")
        elif listed[rel] != _sha256(p):
            out.append(f"{rel}: checksum does not match the manifest")
        if p.suffix != ".jsonl":
            continue
        try:
            r = read_run(p) if "/commands/" not in rel else None
        except Exception as e:      # noqa: BLE001
            out.append(f"{rel}: unreadable ({e})")
            continue
        if "/commands/" in rel:
            out += _check_command_file(p, rel)
            continue
        m = r.manifest
        if _is_rehearsal(r):
            out.append(f"{rel}: a simulated-cell rehearsal")
        if m.side == "real" and m.experiment == "E3":
            out.append(f"{rel}: a real run from E3, which is never released")
        if m.side == "real" and m.experiment != "E1" and m.cell_key() in held:
            out.append(f"{rel}: the REAL run of a held-out cell")
        if m.side == "real" and "/real/" not in rel:
            out.append(f"{rel}: a real run outside a real/ folder")
        if m.side == "sim" and "/sim_s0/" not in rel:
            out.append(f"{rel}: a simulated run outside a sim_s0/ folder")
    for rel in listed:
        if not (folder / rel).exists():
            out.append(f"{rel}: listed in the manifest but missing")
    return out


def _check_command_file(p: Path, rel: str) -> list[str]:
    bad = []
    with p.open(encoding="utf-8") as fh:
        for i, line in enumerate(fh):
            obj = json.loads(line)
            if i == 0:
                if obj.get("_manifest", {}).get("side") != "commands":
                    bad.append(f"{rel}: a command file whose manifest is not side=commands")
                continue
            extra = set(obj) - set(COMMAND_SAMPLE_KEYS) - {"aux"}
            aux_extra = set(obj.get("aux") or {}) - set(COMMAND_AUX_KEYS)
            if extra or aux_extra:
                bad.append(f"{rel}: carries measured data ({sorted(extra | aux_extra)})")
                break
    return bad


def _readme(meta: dict) -> str:
    c = meta["counts"]
    return f"""# SONAIR dataset, release {meta['release']}

A UR5e with an instrumented carrier, recorded by the SONAIR platform: one row
per RTDE packet (125 Hz), on the controller's own clock. Each run is one JSON
Lines file: a manifest, then one sample per line (schema {meta['schema']}).

| Folder | What | Runs |
|---|---|---|
| `E1/real/` | identification set: every joint excited, two payloads (training) | {c['E1']} |
| `E2/published/real/` + `sim_s0/` | published cells: real runs and their S0 simulation | {c['E2_published']} |
| `E2/heldout/commands/` + `sim_s0/` | held-out cells: commanded trajectory and S0 only | {c['E2_heldout']} |

The held-out cells are listed in `plan.json`. Their real runs are kept for
scoring, so a score on them measures generalisation to conditions you were
not shown. S0 is the reference simulation (MuJoCo, menagerie UR5e,
unchanged), driven by each run's own commanded joints.

`manifest.json` gives the versions (motion protocol {meta['protocol']},
harness {meta['harness_commit'] or 'unknown'}) and a SHA-256 for every file.
Check a download with `python -m sonair_benchmark verify-release <folder>`.

How to take part: benchmark.html, "Take part", or docs/Benchmark_Tracks.md.
"""
