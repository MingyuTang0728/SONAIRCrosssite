"""
install_sim.py -- everything the simulation side of the platform needs, once:

    python install_sim.py

  1. MuJoCo, into this Python (pip install mujoco). The benchmark's simulator:
     the offline replay of recorded runs, the live digital twin, and the
     simulated cell's arm all use it.
  2. The UR5e model from Google DeepMind's MuJoCo Menagerie, into
     %LOCALAPPDATA%\\SONAIR\\mujoco_menagerie (only the UR5e folder, a few MB).
  3. URSim 5.11.11 -- Universal Robots' own controller software, the same
     generation as the cell's controller -- as a Docker image. Needs Docker
     Desktop. Skipped, with a note, if Docker is not there: the twin and the
     offline replay work without it; only the simulated cell needs it.

Re-running is safe; anything already in place is left alone.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

HOME = (Path(os.environ["LOCALAPPDATA"]) / "SONAIR" if os.environ.get("LOCALAPPDATA")
        else Path.home() / ".sonair")
MENAGERIE = HOME / "mujoco_menagerie"
MODEL_DIR = "universal_robots_ur5e"
REPO = "google-deepmind/mujoco_menagerie"
URSIM = "universalrobots/ursim_e-series:5.11.11"


def step(msg):
    print(f"\n== {msg}", flush=True)


def mujoco() -> bool:
    step("MuJoCo")
    try:
        import mujoco as mj
        print(f"  already installed: {mj.__version__}")
        return True
    except ImportError:
        pass
    r = subprocess.run([sys.executable, "-m", "pip", "install", "mujoco"])
    if r.returncode:
        print("  pip could not install mujoco. Check the internet connection "
              "and run this again.")
        return False
    print("  installed")
    return True


def menagerie(model_dir: str = MODEL_DIR, label: str = "UR5e") -> bool:
    step(f"{label} model (MuJoCo Menagerie)")
    MODEL_DIR = model_dir           # noqa: N806 -- the folder this call fetches
    target = MENAGERIE / MODEL_DIR
    if (target / "scene.xml").exists():
        print(f"  already in {target}")
        return True
    MENAGERIE.mkdir(parents=True, exist_ok=True)
    if shutil.which("git"):
        tmp = MENAGERIE.with_name("menagerie_clone")
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)
        ok = subprocess.run(["git", "clone", "--depth", "1", "--filter=blob:none",
                             "--sparse", f"https://github.com/{REPO}.git", str(tmp)]
                            ).returncode == 0 and subprocess.run(
            ["git", "-C", str(tmp), "sparse-checkout", "set", MODEL_DIR]).returncode == 0
        if ok and (tmp / MODEL_DIR / "scene.xml").exists():
            if target.exists():
                shutil.rmtree(target)
            shutil.copytree(tmp / MODEL_DIR, target)
            shutil.rmtree(tmp, ignore_errors=True)
            print(f"  fetched into {target}")
            return True
        print("  git could not fetch it; trying the GitHub API instead")
    # no git: walk the folder through the GitHub contents API
    try:
        def walk(path, dest):
            url = f"https://api.github.com/repos/{REPO}/contents/{path}"
            req = urllib.request.Request(url, headers={"User-Agent": "sonair-setup"})
            with urllib.request.urlopen(req, timeout=60) as r:
                items = json.loads(r.read())
            dest.mkdir(parents=True, exist_ok=True)
            for it in items:
                if it["type"] == "dir":
                    walk(it["path"], dest / it["name"])
                elif it["type"] == "file":
                    with urllib.request.urlopen(it["download_url"], timeout=120) as r:
                        (dest / it["name"]).write_bytes(r.read())
        walk(MODEL_DIR, target)
        print(f"  fetched into {target}")
        return (target / "scene.xml").exists()
    except Exception as e:      # noqa: BLE001
        print(f"  could not download the model ({e}). Download "
              f"https://github.com/{REPO} as a zip and copy its {MODEL_DIR} "
              f"folder to {target}")
        return False


def ursim() -> bool:
    step("URSim 5.11.11 (Docker)")
    if not shutil.which("docker"):
        print("  Docker is not installed. The twin and the offline replay work "
              "without it; for the simulated cell install Docker Desktop "
              "(https://www.docker.com/products/docker-desktop/) and run this again.")
        return False
    r = subprocess.run(["docker", "info"], capture_output=True, text=True)
    if r.returncode:
        print("  Docker is installed but not running. Start Docker Desktop and "
              "run this again.")
        return False
    for img in (URSIM, "mirror.gcr.io/" + URSIM):
        if subprocess.run(["docker", "image", "inspect", img],
                          capture_output=True).returncode == 0:
            print(f"  already downloaded: {img}")
            return True
    for img in (URSIM, "mirror.gcr.io/" + URSIM):
        print(f"  downloading {img} (about 1 GB, once) ...", flush=True)
        if subprocess.run(["docker", "pull", img]).returncode == 0:
            return True
    print("  the download failed (Docker Hub limits anonymous downloads; "
          "waiting a while, or `docker login`, usually fixes it)")
    return False


def main() -> int:
    print(f"Simulation components -> {HOME}")
    # The UR10e is for users bringing their own robot's data (intake); the
    # cell itself needs only the UR5e.
    got = {"MuJoCo": mujoco(), "UR5e model": menagerie(),
           "UR10e model": menagerie("universal_robots_ur10e", "UR10e"),
           "URSim": ursim()}
    print("\n" + "\n".join(f"  {k:<11} {'ready' if v else 'NOT ready'}"
                           for k, v in got.items()))
    print("\nWhat each enables:\n"
          "  offline replay + live twin : MuJoCo + UR5e model\n"
          "  your own robot's data      : MuJoCo + the model of that arm (intake)\n"
          "  simulated cell             : MuJoCo + UR5e model + URSim -- then run\n"
          "                               start_sim_cell.ps1 and connect the\n"
          "                               console to robot 127.0.0.2")
    return 0 if got["MuJoCo"] and got["UR5e model"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
