# "The camera still will not connect"

## The symptom, and what it actually means

```
16:50:58 INFO  [agent]  Benchmark:    runs -> ...\bench_runs  sources={'fusionhub': True}
16:51:18 INFO  [agent] camera_config applied: {...}
```

Two things in that log say the camera was never going to work:

1. **`sources={'fusionhub': True}`** — there is no `d435i` key. That key only
   appears when the agent has RealSense support. Its absence means the vision
   imports failed at startup.
2. **`camera_config applied`** — the agent accepted the configuration and
   replied, and the console showed `CONFIG OK`, **even though no camera thread
   was running.** That was a defect: storing a setting for a device that does
   not exist is not the same as applying it, and the console had no way to tell
   the difference.

Meanwhile the parts that do work, work:

```
RTDE streaming 30 fields at 125 Hz (controller 5.11.1.0)
FusionHub listener on UDP 5005 as unit ind0
```

The robot is fine. The IMU listener is fine. Only the camera is missing.

---

## What was wrong, and what now happens instead

`multimodal_bridge.py` imports `cv2`, `numpy` and `pyrealsense2` at the top. If
any one of them fails, `_HAS_VISION` is False, `camera_thread` never starts, and
the only notice was a `print` at import time that scrolls off the top of a long
PyCharm run window.

Three changes:

1. The startup banner now ends with a **loud warning naming the missing module**,
   the exact pip command with the interpreter path filled in, and a note that
   everything else still works. It is the last thing printed, so it cannot
   scroll away before you read it.
2. `camera_config_ack` now carries `vision_available` and `vision_error`.
3. The console shows **`NO CAMERA ON AGENT`** instead of `CONFIG OK`, and
   the colour pane explains which import failed.

Pull the branch and restart the agent. The new banner tells you which package
is missing without any guessing.

---

## The fix

### 1. Find out which package is missing

Restart the agent and read the end of the banner:

```
================================================================
 CAMERA DISABLED — the vision dependencies are missing:
   No module named 'pyrealsense2'
 No camera thread will start, so no frames will ever reach
 the browser and 3D scanning will refuse to run. Install them
 into THIS interpreter:
   C:\...\.venv\Scripts\python.exe -m pip install pyrealsense2 opencv-python numpy
 If pyrealsense2 has no wheel for this Python, use Python 3.11.
 Everything else — the robot, FusionHub, the recorder — works.
================================================================
```

Copy the command it prints. It has the **right interpreter path already filled
in**, which matters: installing into a different Python is the usual reason a
package looks installed but still will not import.

### 2. Install into the interpreter the agent runs under

In the PyCharm terminal (`Alt+F12`), with `(.venv)` in the prompt:

```powershell
python -m pip install pyrealsense2 opencv-python numpy
```

### 3. If pyrealsense2 refuses to install

PyCharm's status bar shows the interpreter version at the bottom right. **If it
says Python 3.12 or 3.13, that is almost certainly the problem** — Intel
publishes `pyrealsense2` wheels for a limited set of Python versions, and on a
newer one pip finds no wheel, falls back to building from source, and fails.

The error looks like:

```
ERROR: Could not find a version that satisfies the requirement pyrealsense2
ERROR: No matching distribution found for pyrealsense2
```

Fix: install **Python 3.11**, then point the project at it.

1. <https://www.python.org/downloads/windows/> → Python 3.11 → tick
   "Add python.exe to PATH"
2. In PyCharm: `File → Settings → Project → Python Interpreter` → gear →
   **Add Interpreter → Add Local Interpreter → Virtualenv → New**
3. Base interpreter: **Python 3.11**. Location: `C:\SONAIR\.venv311`
4. Reinstall everything into it:
   ```powershell
   python -m pip install websockets numpy opencv-python pyrealsense2
   ```
5. Restart the agent

### 4. Confirm

The banner should now read:

```
 Camera:       RealSense support present
```

and shortly after:

```
RealSense D435i started  serial=...  stereo=640x480@30  rgb=1280x720@30
depth intrinsics fx=... fy=... cx=... cy=... scale=0.001
D435i IMU started (accel 63 Hz, gyro 200 Hz)
```

That `depth intrinsics` line is the one that matters for 3D scanning. Without
it, `scan3d_start` refuses — deliberately, because intrinsics must come from the
camera and a datasheet value for "the D435i" is wrong for any individual unit.

---

## Two smaller things in your screenshots

### You are opening the console as a file

The address bar reads:

```
文件 | C:/Users/10277/OneDrive%20-%20.../Remote_control_Benchmark.html
```

That is `file://`. It happens to work for the 3D stage, because a `<script src>`
tag is allowed from `file://` — which is why the UR model loads. But `fetch()`
is not, so anything the page fetches rather than script-tags will fail, and the
failure mode is silent.

Use the server instead:

```powershell
python -m http.server 8000
```

then <http://localhost:8000/SONAIR_Console.html>.

### Depth was switched off

In your screenshot the toggles read `RGB ON · DEPTH OFF · IR1 OFF · IR2 OFF`.
Depth off is correct for saving bandwidth while you are only looking at colour,
but **3D scanning needs depth on**. Turn it back on before the survey sweep.

---

## What this does not block

Nothing on the critical path. The benchmark modalities are orientation, angular
rate, acceleration and position — from the IMU and the robot, both of which are
already streaming. You can record the first six runs today, with no camera at
all.
