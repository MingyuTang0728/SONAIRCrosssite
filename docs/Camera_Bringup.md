# Bringing the camera up, and what comes next

## First: the camera is not on the critical path

Your goal is the first benchmark dataset. The benchmark modalities are
**orientation, angular rate, acceleration and position** — they come from the
IMU and the robot. **None of them needs the camera.**

The camera serves the *application case* (3D reconstruction and scan planning),
which Sam was explicit is a separate contribution from the benchmark itself.

So: get the camera streaming, confirm it works, then **park it and record the
six runs**. Do not let hand-eye calibration block your first dataset.

---

## Step 1 — Connect the right socket

The console has **two** WebSocket connections and they are not interchangeable:

| Connection | Where | Carries |
|---|---|---|
| **Connect local bridge** | Sensor console → *Connection & Teleoperation* | camera frames, robot state, jogging |
| **Connect bridge** | Robot & 3D scan → *0 · Host agent* | full telemetry, control, 3D scanning |

Camera frames arrive on the **first** one. Either connection now feeds the
camera panel and the monitor bar, but if you have only used the Robot panel so
far, press **CONNECT LOCAL BRIDGE** in Sensor console as well.

The badge next to it should read `LINK ACTIVE`.

---

## Step 2 — Check the agent found the camera

In the terminal running `multimodal_bridge.py`, look for:

```
RealSense D435i started  serial=...  stereo=640x480@30  rgb=1280x720@30
depth intrinsics fx=... fy=... cx=... cy=... scale=0.001
D435i IMU started (accel 63 Hz, gyro 200 Hz)
```

If instead you see `camera startup failed: ... — retrying in 3 s`, the camera is
not reachable. Check it appears in Intel RealSense Viewer first — if Viewer
cannot see it, nothing here will either.

If you see `WARNING: vision deps missing`, `pyrealsense2` is not installed in
the interpreter the agent is running under.

---

## Step 3 — Watch the streams

**Sensor console → Live 2D Streams.**

The badge there is now a real status, not decoration:

| Badge | Meaning |
|---|---|
| `NO BRIDGE` | no socket is open |
| `WAITING` | connected, no frame has arrived yet |
| `LIVE` | frames arriving |
| `STALLED 4s` | frames stopped — the camera dropped out, look at the agent log |
| `CONFIG OK` | the camera accepted a configuration change |

Colour and depth are on by default, so both panes should fill within a second.

---

## Step 4 — Change the configuration

The controls in the RealSense panel were **unbound until now** — Apply Config
and all four view toggles did nothing, which is why changing resolution appeared
to have no effect. They work now.

- **RGB ON / DEPTH ON / IR1 / IR2** — each toggles the pane **and** turns that
  stream on or off at the camera. Hiding a pane while the camera still encodes
  and sends it wastes the whole socket budget on frames nobody looks at, which
  is what starves the IMU stream.
- **Apply Config (Sync to Bridge)** — sends resolution, frame rate, emitter,
  auto-exposure and post-processing, and restarts the pipeline.

After an apply, the fields refresh with what the **camera accepted**, not what
you asked for. The device silently substitutes an unsupported resolution or
rate, and a panel showing the request rather than the result is a panel that
lies about what the data was captured at.

### Sensible settings

| Setting | Value | Why |
|---|---|---|
| Stereo | 640×480 @ 30 | the sweet spot for depth quality against bandwidth |
| RGB | 1280×720 @ 30 | **do not lower this if you calibrate.** The board is detected in the colour image, and the detector needs about 15 pixels between corners. A 7.5 mm square at 400 mm lands on ~11 px at 640×360 and ~23 px at 1280×720 — the difference between a board that is found and one that is reported as absent while it is plainly in shot |
| Emitter | Laser | needed for depth on low-texture surfaces — a machined metal face has almost no texture |
| Auto exposure | on | until you have a controlled lighting setup |
| Post-processing | on | spatial + temporal + hole fill, already wired in the agent |

Turn IR1 and IR2 **off** unless you are debugging the stereo pair. They double
the bandwidth for no benefit to either the benchmark or the reconstruction.

---

## Step 5 — Confirm the depth is metric

This is the check that matters, and the panel cannot show it: the colourised
depth image you see **has no metric content left in it**. The agent keeps the
raw 16-bit frame separately, and that is what the 3D pipeline uses.

Confirm the agent read real intrinsics from the device:

```
depth intrinsics fx=... fy=... cx=... cy=... scale=0.001
```

If that line is missing, `scan3d_start` will refuse with *"camera intrinsics
unavailable"*. That refusal is deliberate — intrinsics must come from the
camera, never from a datasheet, because a datasheet value for "the D435i" is
wrong for any individual unit.

---

## Step 6 — Now go and record the first six runs

The camera is up. It is **not needed** for the benchmark dataset. Go to
**Robot & 3D scan → 5 · Record a benchmark run** and record:

| Runs | Elbow velocity | Configuration | Trajectory |
|---|---|---|---|
| 3 | 0.4 rad/s | mid_workspace | contour |
| 3 | 0.9 rad/s | mid_workspace | contour |

Before the first one: **set the payload, then zero the F/T sensor**, in that
order (Robot & 3D scan → 2 · Control).

---

## What blocks the 3D scan, and what to do about it

`scan3d` needs **T_tcp_cam** — where the camera sits relative to the tool
flange. Nothing in the project computes it yet; the panel accepts the matrix but
does not solve for it.

Accuracy here bounds everything downstream. A D435i at 300 mm has roughly
1–2 mm depth noise and that averages down over views. A **2° error in T_tcp_cam
does not average down at all** and puts the whole cloud 10 mm out at 300 mm
standoff.

### Interim: a measured estimate

Good enough to see whether the pipeline behaves, not good enough to trust a
number from it.

1. From the bracket CAD, read the camera's optical-centre offset from the tool
   flange in the TCP frame. The D435i's depth optical centre is behind the front
   glass on the **left** imager, not at the case centre — take it from the Intel
   datasheet drawing rather than the case outline.
2. Enter it as the translation column:
   ```json
   [[1,0,0, 0.050],
    [0,1,0, 0.000],
    [0,0,1, 0.040],
    [0,0,0, 1    ]]
   ```
3. If the camera is rotated relative to the flange, the 3×3 block is not the
   identity and you need the real rotation — that is the part a tape measure
   cannot give you.

### Sanity check it

Put a flat plate on the table, run a 6-view sweep, reconstruct. Then:

- Is the recovered plane **flat**? A bowed or doubled plane means the rotation
  is wrong.
- Does `tilt_deg` match the table's actual tilt (near zero)?
- Do the component extents match a ruler to a few mm?

If the plane comes out doubled — two parallel sheets a few mm apart — the
translation is wrong and the views are not landing on each other.

### Proper: AX = XB

Pose the arm so the camera sees a fixed calibration target from **at least ten
well-spread orientations**, record (TCP pose, target pose in camera) at each,
and solve the hand-eye equation. Ten poses with genuinely different rotations,
not ten small nudges — a cluster of similar poses is badly conditioned and
returns a confident wrong answer.

This is the next thing worth building. It is the single largest error source in
the 3D path, and it is not on the critical path for your first dataset.
