# Running this project in PyCharm

## First, two corrections to how this probably looks

**You do not open and run each `.py` file.** Of the 20 Python files here, only
**two** are programs. Everything else is a library that gets imported — pressing
Run on `ur_telemetry.py` does nothing at all, because it has no entry point.

| File | What it is |
|---|---|
| `check_setup.py` | **Run this.** Pre-flight diagnostic |
| `multimodal_bridge.py` | **Run this.** The host agent — leave it running |
| `ur_telemetry.py` | library — imported by the bridge |
| `ur_control.py` | library — imported by the bridge |
| `ur_bridge_ext.py` | library — imported by the bridge |
| `bench_agent.py` | library — imported by the bridge |
| `scan3d.py` | library — imported by the bridge |
| `sonair_benchmark/` | package — run with `python -m sonair_benchmark …` |
| `relay_server.py` | only for cross-site teleoperation with UCL, not needed now |
| `sonair_keygen.py` | only for relay tokens, not needed now |

**The terminal is project-wide, not per-file.** PyCharm has one terminal (bottom
of the window, or `Alt+F12`) and its working directory is the project root no
matter which file is open in the editor. There is no "terminal under a file".

---

## Step 0 — Open the right folder

Open **`C:\SONAIR`**, not the OneDrive copy.

`File → Open…` → `C:\SONAIR` → OK.

If PyCharm currently has the OneDrive folder open, close that project first
(`File → Close Project`). Recording writes 125 lines a second; OneDrive will
compete for the file handle and can lock a run mid-capture.

---

## Step 1 — Set the interpreter

This is the step that makes everything else work, and the one that produces the
most confusing errors when skipped.

1. `File → Settings` (`Ctrl+Alt+S`)
2. `Project: SONAIR → Python Interpreter`
3. Click the gear icon → **Add Interpreter → Add Local Interpreter…**
4. Choose **Virtualenv Environment → New**
5. **Base interpreter:** select **Python 3.11**
   - If 3.11 is not in the list, click `…` and browse to
     `C:\Users\10277\AppData\Local\Programs\Python\Python311\python.exe`
   - If it is not installed at all, install it first — see
     `docs/Windows_Quickstart.md` step 1
6. **Location:** leave it as `C:\SONAIR\.venv`
7. OK

### Why Python 3.11 and not the newest

`pyrealsense2` publishes prebuilt wheels for a limited set of Python versions.
On 3.12 or 3.13 there is often **no wheel**, pip falls back to building from
source, and it fails. Everything except the camera works on any 3.9+; the camera
is what pins the version.

Check the bottom-right status bar — it should read `Python 3.11 (SONAIR)`.

---

## Step 2 — Install the dependencies

Open the terminal: **`Alt+F12`**, or `View → Tool Windows → Terminal`.

The prompt should start with `(.venv)`. If it does not, the interpreter is not
set — go back to Step 1.

```powershell
python -m pip install --upgrade pip
python -m pip install websockets numpy opencv-python
python -m pip install pyrealsense2
```

`pyrealsense2` is installed **last and on its own** on purpose: it is the one
most likely to have no wheel for your Python, and a failure there must not take
the other three down with it.

If it fails, you lose the camera and its built-in IMU. The robot, FusionHub, the
recorder and the whole benchmark toolchain still work.

---

## Step 3 — Run the pre-flight check

Two equivalent ways. Use whichever you prefer.

**Terminal:**
```powershell
python check_setup.py --ur 192.168.0.20
```

**Run button:** the run configurations ship with the project. Pick
**`1 - Check setup`** from the dropdown at the top-right and press ▶.

It checks Python, the path, the packages, the repository files, each UR port
individually, the camera, the FusionHub stream and write permission, then prints
what to fix in order. It touches nothing — it only opens and closes connections.

**Run it again after every fix.**

> The run configurations carry `UR_IP=192.168.0.20`. If your robot is on a
> different address, edit it once in `Run → Edit Configurations… → Environment
> variables`, and both the check and the agent pick it up.

---

## Step 4 — Three terminal tabs

You need more than one terminal, because the agent and the web server both stay
running. Press the **`+`** at the top of the terminal panel to add a tab.

| Tab | Command | Purpose |
|---|---|---|
| 1 | `python multimodal_bridge.py` | the agent — leave it running |
| 2 | `python -m http.server 8000` | serves the console — leave it running |
| 3 | *(free)* | for the `sonair_benchmark` commands |

Set the robot IP in tab 1 before starting the agent:

```powershell
$env:UR_IP = "192.168.0.20"
python multimodal_bridge.py
```

Or use the **`2 - Start agent`** run configuration, which sets it for you.

### Stopping them

- In a **terminal** tab: `Ctrl+C`
- In a **Run** window: the red ■ button. `Ctrl+C` does not reach a PyCharm run
  window unless "Emulate terminal in output console" is on — it is on in the
  shipped configurations.

---

## Step 5 — Open the console

Browser: <http://localhost:8000/SONAIR_Console.html>

**Do not use PyCharm's own "Open in Browser"** on the HTML file. It opens a
`file://` URL, which blocks the browser from fetching `vendor/three/`. The 3D
stage then never initialises and importing a GLB silently does nothing — the
exact symptom you hit before. It must be `http://localhost:8000/…`.

---

## The commands you will actually type

All of them from the project root, in a free terminal tab.

```powershell
# pre-flight, after any change
python check_setup.py --ur 192.168.0.20

# the agent (tab 1, stays running)
$env:UR_IP = "192.168.0.20"
python multimodal_bridge.py

# the web server (tab 2, stays running)
python -m http.server 8000

# see the whole analysis chain on synthetic data, before any real capture
python -m sonair_benchmark demo --out demo

# Phase 0: IMU noise floor from a FusionHub export
python -m sonair_benchmark phase0 --fusionhub stationary.csv --expected-hz 200 --out phase0\ind0.json

# the error budget
python -m sonair_benchmark budget --phase0 phase0\ind0.json --tap-spread-ms 0.8 --tracker-mm 1.2 --frame-fit-mm 0.8 --calib-version calib-1 --out calib\budget.json

# the condition sweep
python -m sonair_benchmark plan --out campaign\plan.json

# after you have real and simulated runs
python -m sonair_benchmark gap --real bench_runs --sim sim_runs --budget calib\budget.json --out results\gap.json
python -m sonair_benchmark score --real bench_runs --sim sim_runs --budget calib\budget.json --plan campaign\plan.json --out site\leaderboard.json
```

---

## PyCharm problems and what they mean

| Symptom | Cause | Fix |
|---|---|---|
| Terminal prompt has no `(.venv)` | Interpreter not configured | Step 1, then close and reopen the terminal tab |
| `ModuleNotFoundError: websockets` | Installed into a different interpreter | Check the bottom-right status bar, then reinstall in the PyCharm terminal |
| Red squiggles under `import numpy` but it runs fine | PyCharm has not re-indexed | `File → Invalidate Caches → Invalidate and Restart` |
| `No module named sonair_benchmark` | Terminal is not in the project root | `cd C:\SONAIR` |
| Run button greyed out | No configuration selected | Pick one from the dropdown at the top-right |
| Running a library file does nothing | It has no `__main__` — that is correct | Run `multimodal_bridge.py` instead |
| Agent starts then exits immediately | Read the traceback in the Run window | Usually a missing package — run the pre-flight check |
| `Address already in use` on 8765 | An agent is already running | Stop the other one, or check the Run windows for a second tab |

---

## What a working session looks like

```
Tab 1  (.venv) PS C:\SONAIR> python multimodal_bridge.py
       UR service:   {'ok': True, 'host': '192.168.0.20', ...}
       RTDE streaming 30 fields at 125 Hz (controller 5.x.x.x)
       FusionHub listener on UDP 5005 as unit ind0
       server listening on 0.0.0.0:8765
       ← leave it

Tab 2  (.venv) PS C:\SONAIR> python -m http.server 8000
       Serving HTTP on :: port 8000 ...
       ← leave it

Tab 3  (.venv) PS C:\SONAIR> _
       ← your working tab
```

Browser on <http://localhost:8000/SONAIR_Console.html>, in the
**Robot & 3D scan** view.
