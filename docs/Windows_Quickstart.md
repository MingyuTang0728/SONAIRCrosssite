# Windows quickstart — first data capture

Written for the workstation wired to the UR5e. Do these in order; each step
tells you how to know it worked.

---

## Before anything: move the project out of OneDrive

Your copy is at:

```
C:\Users\10277\OneDrive - The University of Nottingham\桌面\IMU SONAIR 后期实验\SONAIRCrosssite-...
```

Three separate problems live in that one path:

1. **OneDrive.** Recording writes a JSON line **125 times a second**. OneDrive
   will try to sync every flush, will compete for the file handle, and with
   Files On-Demand it can evict a file to the cloud while the code still expects
   it on disk. The sync icons in your folder listing show this is live.
2. **Non-ASCII characters** (`桌面`, `后期实验`). Several native libraries —
   `pyrealsense2` among them — mishandle non-ASCII paths on Windows.
3. **Spaces**, which mean every `cd` needs quotes.

Move it:

```powershell
# In PowerShell
New-Item -ItemType Directory -Force -Path C:\SONAIR
Copy-Item -Recurse -Force `
  "C:\Users\10277\OneDrive - The University of Nottingham\桌面\IMU SONAIR 后期实验\SONAIRCrosssite-claude-sonair-benchmark-implementation-9knwbe\*" `
  C:\SONAIR\
cd C:\SONAIR
```

Everything below assumes `C:\SONAIR`.

---

## Step 1 — Install Python

`pip : 无法将"pip"项识别为 cmdlet` means **Python is not installed**, or it is
installed without being added to PATH. On Windows 11, typing `python` with no
Python installed opens the Microsoft Store instead of running anything — that
stub is not a working Python and must not be used.

1. Download **Python 3.11** from <https://www.python.org/downloads/windows/>.
   Pick "Windows installer (64-bit)".
2. In the installer, **tick "Add python.exe to PATH"** on the very first screen.
   This is the box everyone misses and it is the whole cause of your error.
3. Close PowerShell completely and open a **new** window — PATH is only read at
   startup.

### Why 3.11 specifically

`pyrealsense2` publishes prebuilt wheels for a limited set of Python versions.
On 3.12 or 3.13 there is often **no wheel**, `pip` tries to build from source,
and it fails. Everything except the camera works on any 3.9+; the camera is the
part that pins the version.

Check it worked:

```powershell
python --version      # expect: Python 3.11.x
py --version
```

---

## Step 2 — Run the setup script

```powershell
cd C:\SONAIR
.\setup_windows.ps1
```

If PowerShell refuses with *"running scripts is disabled on this system"*:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\setup_windows.ps1
```

`-Scope Process` lasts only for that window, which is the smallest change that
works.

The script creates `.venv`, installs `websockets`, `numpy`, `opencv-python` and
`pyrealsense2`, and then runs the pre-flight check.

### If pyrealsense2 fails to install

Not fatal. You lose the camera and its built-in IMU; the robot, FusionHub, the
recorder and the whole benchmark toolchain still work. Either carry on without
the camera for now, or install Python 3.11, delete `.venv`, and re-run.

---

## Step 3 — Pre-flight check

```powershell
.\.venv\Scripts\python.exe check_setup.py --ur 192.168.0.20
```

It checks Python, the path, the packages, the repository files, each UR port
individually, the camera, the FusionHub stream and write permission — then
prints exactly what to fix, in order.

**Run it again after every fix.** Aim for zero failures; warnings are fine if
you understand what each one costs you.

### Reading the UR section

Each port is a separate capability and they fail independently:

| Port | If closed, you lose |
|---|---|
| 29999 | power on/off, brake release, program control, protective-stop recovery |
| 30002 | **all motion**, IO, payload, freedrive |
| 30003 | the 125 Hz telemetry fallback |
| 30004 | RTDE — full telemetry (IO bits, program state, speed scaling) |

If nothing answers: check the controller is powered on, the Ethernet cable is
in, and the PC and robot are on the same subnet. Read the robot's real IP on the
pendant under **Settings → System → Network**.

If only 30004 is closed, telemetry falls back to 30003 and the console says so.
You still get force, currents and temperatures.

---

## Step 4 — Put the pendant in Remote Control

**Top-right of the teach pendant → Remote Control.**

External URScript is refused in Local mode, and the symptom is a *connection
timeout*, not a clear refusal — so it looks like a network fault for as long as
you let it.

---

## Step 5 — Start the agent

```powershell
.\start_agent.ps1
# or, if the robot is on a different address:
.\start_agent.ps1 -UrIp 192.168.1.50
```

**Leave this window open.** Everything the browser shows comes through it.

Expect:

```
UR service:   {'ok': True, 'host': '192.168.0.20', ...}
RTDE streaming 30 fields at 125 Hz (controller 5.x.x.x)
depth intrinsics fx=... fy=... cx=... cy=... scale=0.001
FusionHub listener on UDP 5005 as unit ind0
D435i IMU started (accel 63 Hz, gyro 200 Hz)
server listening on 0.0.0.0:8765
```

---

## Step 6 — Open the console

In a **second** PowerShell window:

```powershell
cd C:\SONAIR
.\serve_console.ps1
```

It serves the folder on port 8000 and opens the console.

> **Do not double-click the .html file.** Opening it as `file://` blocks the
> browser from fetching `vendor/three/`, so the 3D stage never initialises and
> importing a GLB silently does nothing — the exact symptom you hit before.

In the console: **Robot & 3D scan** → *Connect bridge* → *Start UR service*.

You should see the force card reading, the joint table filling, and `ind0` in
the inertial table at about 200 Hz.

---

## Step 7 — Set the payload, then zero the F/T sensor

**In that order**, in *2 · Control → Tool and payload*.

1. Enter the measured mass of your printed bracket plus the IMU and camera.
2. *Set payload*.
3. Hang the tool free.
4. *Zero F/T sensor*.

Zeroing with the payload wrong subtracts gravity along with the offset, and
every force reading afterwards is out by the weight of the tool.

---

## Step 8 — Phase 0: the numbers everything else is measured against

No robot motion in this step.

### 8.1 One-hour stationary log

Clamp the IMU to something isolated from foot traffic. Record an hour in
FusionHub, export CSV, then:

```powershell
.\.venv\Scripts\python.exe -m sonair_benchmark phase0 `
    --fusionhub stationary.csv --expected-hz 200 --out phase0\ind0.json
```

Watch the **jitter** figure. Above about 2 ms, a drifting sample rate will read
downstream as a velocity-dependent gap that is not real.

### 8.2 Tap test

Tap the carrier once, firmly, then press **Verify tap alignment** in the console.
The spread across channels is the temporal row of the error budget and gets
quoted in every later result.

### 8.3 Error budget

```powershell
.\.venv\Scripts\python.exe -m sonair_benchmark budget `
    --phase0 phase0\ind0.json --tap-spread-ms 0.8 `
    --tracker-mm 1.2 --frame-fit-mm 0.8 `
    --calib-version calib-1 --out calib\budget.json
```

---

## Step 9 — The first six runs

```powershell
.\.venv\Scripts\python.exe -m sonair_benchmark plan --out campaign\plan.json
```

Do **not** start the full 270-run sweep. Record six runs first:

| Runs | Elbow velocity | Configuration | Trajectory |
|---|---|---|---|
| 3 | 0.4 rad/s | mid_workspace | contour |
| 3 | 0.9 rad/s | mid_workspace | contour |

0.4 is below the known simulator divergence and 0.9 is well above it. Six runs
is enough to see whether the gap has the velocity dependence the whole benchmark
is built around — before committing four weeks to the full sweep.

For each run, in **Sensor console** (the acquisition panel at the bottom of the
sensor column):

1. Fill the manifest: elbow velocity, configuration, trajectory type, repeat
   index, and **calibration version** (`calib-1`).
2. **Record run**.
3. Execute the motion.
4. **Stop**.

One `.jsonl` file per run lands in `C:\SONAIR\bench_runs\`.

### Check what you got

```powershell
Get-ChildItem bench_runs
Get-Content bench_runs\v0p40_mid_workspace_contour_r00.jsonl -TotalCount 3
```

Line 1 is the manifest. Every later line is one sample. A run killed halfway is
still a valid file up to where it died.

---

## Step 10 — Try the analysis before you have simulation data

```powershell
.\.venv\Scripts\python.exe -m sonair_benchmark demo --out demo
```

This generates a synthetic real/sim pair with a known injected gap and runs the
whole chain — budget, gap, both gates, baselines, leaderboard. Then open
<http://localhost:8000/benchmark.html> and the page populates from
`demo\site\leaderboard.json` and `demo\results\gap.json`.

It proves the toolchain works before a single real sample exists, and it is the
fastest way to show Sam what the benchmark will look like.

---

## Common problems

| Symptom | Cause | Fix |
|---|---|---|
| `pip is not recognized` | Python not installed, or not on PATH | Step 1. Reinstall with "Add python.exe to PATH" ticked, open a **new** PowerShell |
| `running scripts is disabled` | PowerShell execution policy | `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` |
| GLB imports but nothing appears | Page opened as `file://` | Use `.\serve_console.ps1`, open `http://localhost:8000/...` |
| `URScript send ... timed out` | Pendant in Local mode, or wrong IP | Step 4; check the IP on the pendant |
| `RTDE unavailable ... falling back` | RTDE disabled, or another client holds it | Works anyway with fewer fields; close other RTDE clients to get them back |
| Camera not found | USB 2 port or wrong cable | Blue USB 3 port, original cable; confirm in RealSense Viewer first |
| No FusionHub packets | Not configured to emit JSON over UDP | Set output to JSON/UDP `127.0.0.1:5005`, or use the CSV replay path |
| `pyrealsense2` will not install | No wheel for your Python version | Install Python 3.11, delete `.venv`, re-run setup |

---

## Two windows, every session

```
Window 1:  cd C:\SONAIR ;  .\start_agent.ps1
Window 2:  cd C:\SONAIR ;  .\serve_console.ps1
```

Then `http://localhost:8000/SONAIR_Console.html`.
