# The inspection console

`SONAIR_Console.html` — the operator-facing page. Four numbered steps down the
left, one job per screen, no JSON and no stack traces. Diagnostics stay in the
agent's log window where an engineer can read them.

Open it at `http://localhost:8000/SONAIR_Console.html` (not `file://`).

`Remote_control_Benchmark.html` and `Remote_control.html` are the pages this
replaced. They are kept as the record of how the cell was driven before, and
each now opens with a banner saying so. Do not run an experiment from them:
the hand-eye calibration, the multi-view scan, the modality registry and the
inertial logging exist only in the console, and two pages open at once means
two clients issuing motion to the same arm through the same connection.

**There is one page to drive: `SONAIR_Console.html`.**

---

## Why the camera would not start

The log said, over and over:

```
camera startup failed: Couldn't resolve requests — retrying in 3 s
```

RealSense Viewer worked, so the device was fine. Two things were wrong.

**The motion streams had been added to the video pipeline.**

```python
rscfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
rscfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
```

`bench_agent.D435iImuSource` already opens its **own** pipeline for accel and
gyro. The IMU sits behind a single sensor, so a second pipeline asking for it
fails the whole configuration — and takes depth and colour down with it, even
though nothing was wrong with either. The motion streams now have exactly one
owner and the video pipeline never asks for them.

**Nothing checked the requested mode against the device.**

`Couldn't resolve requests` is librealsense saying "no mode matches what you
asked for". It names neither the offending stream nor a working alternative.
`camera_negotiate.py` now asks the device what it supports, substitutes the
nearest workable mode, and says what it changed:

```
camera: colour 1280x720@60 is not supported on this device and connection;
        using 1280x720@30
```

If the start still fails, the message names the cause instead of repeating the
SDK's:

```
the camera rejected this stream combination  |  this camera is connected over
USB 2.1. On USB 2 a D435i offers only a handful of modes — use a USB 3 port
(blue) and the cable that came with the camera.  |  depth: asked for
848x480@60; device offers 848x480@30, 640x480@30, 424x240@60
```

The console's resolution and rate menus are now built from the device's own
profile list, so an unsupported combination cannot be picked in the first place.

---

## The inspection workflow

**Step 3 — Inspect.** Four stages, and the picture updates at each one so you
can see what the system found before the arm moves.

### 1. Find the part

Segments the component off the fixture **by height, not by colour**. A part and
its fixture are frequently the same metal under the same light, so a colour
segmentation separates them only by luck; what reliably distinguishes them is
that one stands 20 mm above the other.

Fits the fixture plane by RANSAC, keeps what stands proud of it, takes the
largest connected region, and measures it. On a synthetic 120 × 80 mm plate it
recovers 115 × 75 mm — a few millimetres under, because the depth edge is the
least reliable part of a stereo frame.

Reports size, height above the fixture, and measured area. With the hand-eye
transform set, the outline also comes back in robot coordinates.

### 2. Plan the scan

Two patterns:

- **Cover the whole face** — serpentine raster inside the measured outline
- **Follow the edge** — the outline resampled at a constant arc length

Waypoints stay inside the **measured outline**, not its bounding box. For
anything other than a rectangle those differ by a lot, and the difference is
entirely time spent scanning the fixture.

Reports point count, total travel and estimated time.

### 3. Look for problems

Two independent channels, reported separately because they fail differently.

| Channel | Finds | Blind to |
|---|---|---|
| **Surface** | dents, steps, burrs — height residual against a quadratic fitted to the part's own top | anything shallower than the depth noise, about 1–2 mm at 300 mm |
| **Visual** | marks, scratches, staining — local darkness against a blurred copy | nothing; it also fires on shadows, coolant, fingerprints and machining lines |

The baseline is a **fitted surface, not a blur**. A blurred baseline is dragged
toward whatever sits under the kernel, so a large dent partly hides itself; a
fit over the whole face is not.

A candidate found by both channels is ranked "Check first". That is all the
agreement means.

**These are candidates, not findings.** The page says so, the API says so, and
the wording survives being pasted into a report. A depth camera at 300 mm
resolves roughly 1–2 mm; the defects this project ultimately cares about are
smaller than its noise floor. The value is telling the arm where to look
closely, not deciding what is there.

### 4. Run it

Sends the planned waypoints, with a confirmation first.

### On the overlay

The outline is drawn in the depth image's pixel grid and scaled when the colour
frame is a different size. The planned path is in robot coordinates and is
projected back through the located outline's bounding box — which is an
approximation, and is why the picture is labelled a preview rather than a
measurement.

---

## What still blocks the robot-frame half

`scan3d` and the path planner need **T_tcp_cam** — where the camera sits
relative to the tool flange. Nothing computes it yet.

Finding the part and flagging candidates work without it, in image
coordinates. Planning a robot path does not, and the console says so rather
than planning something meaningless.

Accuracy there bounds everything downstream: a D435i at 300 mm has 1–2 mm depth
noise which averages down over views, but a 2° hand-eye error does not average
down at all and puts the whole cloud 10 mm out.

---

## Layout

| Step | What it is for |
|---|---|
| **1 Connect** | one address, one button, three device cards in plain words |
| **2 Robot** | 3D view, tool position, forces, health, jog, every joint field, electrical and IO |
| **3 Inspect** | the four-stage workflow above |
| **4 Record** | benchmark run capture |

The **STOP ROBOT** button is always visible in the rail. It is a software stop;
the physical e-stop is the safety device, and the page says that too.

Both jog pads are fully visible at any window width — the previous console cut
the second one off. Holding a pad repeats the command while held, because a
`speedl` decays after its `t` and a held joystick with no repeat produces one
twitch and then stops, which reads as a fault.
