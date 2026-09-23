# SONAIR console — the whole run, in order

Everything below is done from one page: `SONAIR_Console.html`, with
`multimodal_bridge.py` running on the same PC. The numbered steps in the left
rail are the order to do them in. Nothing here needs a terminal once the agent
is started.

---

## Before anything: what has to be installed

In the **same interpreter** the agent runs in:

```
pip install numpy opencv-python pyrealsense2 websockets
pip install pyserial          # only if an IMU is on a COM port
```

`python check_setup.py` reports which of these are present and what each one
unlocks. Missing `pyrealsense2` disables the camera and nothing else; missing
`opencv-python` disables hand-eye calibration and the inspection pipeline;
missing `numpy` disables 3D entirely. The robot, the IMU and recording work
without all three.

---

## 1 · Connect

Start the agent, then press **Connect**. The three lamps at the top right are
the only summary you need: robot, camera, motion sensor.

---

## 2 · Robot

Check the arm is in **Remote Control** on the pendant, and that the tool
position updates when you jog. If the lamp says the robot is connected but
nothing moves, the pendant is in Local.

---

## 3 · Camera

Four pictures: colour, depth, and **both infrared cameras**. The infrared pair
is not decoration — depth is computed from it, so when depth has holes those
two images are what says why.

- **Projector** — on for depth on machined or painted surfaces, off for a
  clean infrared image, alternating for both on consecutive frames. On a
  metal face this is the single biggest lever on how much depth you get.
- **Why is depth missing?** — reads the infrared pair and names one of three
  causes, each with a different fix: glare (drop the exposure), no texture
  (turn the projector up), too dark (raise exposure or gain).
- **Depth quality → Measure now** — the numbers that matter are *centre
  coverage* (how much of where the part sits returned a reading) and *depth
  noise*. Below about 70% coverage, expect holes in the 3D model.
- **Re-calibrate the camera** — the D435i can redo its own stereo calibration
  in about ten seconds against a flat, textured surface. Worth doing before a
  campaign on a camera that lives on a moving arm. The health score it returns
  is worth writing down next to the data.
- **Depth processing** — fewer filters is usually better. *Fill holes* invents
  depth that was never measured: it looks better on screen and measures worse,
  so it ships off.

---

## 4 · Sensors

### Getting data out of FusionHub

This is the step that most often fails, so it is built to be diagnosed rather
than guessed at.

1. In FusionHub, enable a network output. UDP to `127.0.0.1` if FusionHub runs
   on this PC. JSON or CSV — not the binary protocol.
2. Press **Find it for me**. The agent listens on every plausible port for six
   seconds and reports which ones received anything, from where, in what
   format, and whether it recognised any inertial fields. If it finds a usable
   port it fills the form in for you.
3. Press **Connect**.
4. If nothing arrives, press **Show what is arriving** — it prints the last
   packet verbatim with what was recognised in it. "Nothing arrived" and
   "something arrived that we could not read" are different faults with
   different fixes, and this is what tells them apart.

Other transports are there because FusionHub's available outputs depend on
version and licence: this PC connecting out over TCP, FusionHub connecting in,
its **WebSocket Sink**, an HTTP endpoint, a COM port, or **following a file
FusionHub is writing**.

In FusionHub 0.1.x the relevant graph nodes are **TCP Output** (under
CONNECTORS) and **WebSocket Sink**. Both accept `Any` data type, so either
will carry the LPMS source's `Imu` output without a converter in between.
That last one always works and produces exactly the same records, so a live
integration is never on the critical path.

### What you see

- **How it is lying** — a box showing the sensor's real orientation, with roll,
  pitch, yaw, turn rate and acceleration. A unit that reports its own
  quaternion is shown as measured; a unit that reports only accelerometer and
  gyroscope (the camera's built-in one) gets its orientation worked out here.
- The chips underneath are the health checks worth reading:
  - *turn rate read as degrees/s* — the units the source uses are **decided**
    from its own data, not assumed. If the sensor also reports orientation,
    the decision is made by comparing the two, which is conclusive.
  - *sensor vs our own estimate* — an independent orientation estimate run
    alongside the sensor's own. Agreement under a degree or two means both are
    working. A large number means one of them is not, and it has caught a
    wrongly configured output frame more than once.
  - *drift correction* — the gyroscope bias learned while the unit is still.
    A consumer part routinely shows 1-2 °/s; uncorrected, that is 60-120° of
    pure fiction over a minute.
- **Re-level** — hold the unit still and press it. Clears the learned bias and
  re-seeds the orientation from gravity.

### Every measurement channel

The table lists every channel this cell knows about, including the ones whose
hardware has not arrived. Channels marked **scored** count toward the
benchmark: cheap to ground-truth on real hardware *and* faithful to simulate.
Eddy current, ultrasound and thermography are listed as **inspection** — they
are evidence for the application case, and they must not be scored as
benchmark channels. That distinction is in the code, not just in this
document.

---

## 5 · Calibrate — where the camera sits on the tool

**Do this before anything 3D.** Every point the camera produces is placed in
the robot's frame through this one transform, so an error in it is a rigid
error that grows with standoff and rotates with the tool. At 300 mm standoff,
2° puts a point 10 mm out — in a *different direction* at every viewpoint,
which is exactly what stops several views from fusing into one surface.

1. Fix a chessboard where the camera can see it and the arm can move around
   it. Enter the **inner corner** counts — a board of 10×7 squares is 9×6
   inner. Getting this wrong is the commonest way to end up with a
   calibration that looks fine and is wrong.
2. Press **Start**.
3. Move the arm, check the live view says the board was found, press **Capture
   this pose**. Repeat about a dozen times. **Rotate the tool 30-60° about all
   three axes between poses and vary the distance** — the advice box tells you
   when the pose set is not varied enough to determine the answer.
4. Press **Work out where the camera is**.

The accuracy figure is not the solver's own residual. It is measured in the
frame the work happens in: the board has not moved, so the calibration is
asked to reconstruct it from every pose, and the spread of those
reconstructions is reported. Under 2 mm is good, under 5 mm is usable, above
that something is wrong — usually the square size in millimetres, a board that
moved, or the TCP set wrong on the pendant.

If the pose set was degenerate the console says **do not use this** even when
the residual reads 0.0 mm. That case is real: with every rotation about one
axis, the translation along it is unobservable, so any value fits, the board
reconstructs perfectly, and the answer is still centimetres out. Three
independent solvers are run and their disagreement is reported for the same
reason.

5. **Save and use it.** The name it saves under (`handeye-…`) is the
   calibration version — put it on every run recorded from now on. Runs either
   side of a recalibration cannot be compared, and this is the only record of
   which side a run came from.

---

## 6 · Inspect

Two routes to the same end: a scan path the robot can run.

### 3D scan — several views merged into a measured model

1. Press **Draw a box** and drag a box round the part in the picture.
2. **Use the box I drew** — gives the part's position and size in the robot's
   frame, and reports how much of the box actually returned depth.
3. **Work out the angles** — the standoff comes from the part's size and the
   lens so it fills the frame; the number of photographs comes from the
   overlap adjacent views must share; the tilt comes from the part's height
   against its footprint; a second ring is added when one elevation cannot see
   both the top and the walls. Viewpoints the arm cannot reach are dropped
   with the reason. Every one of those choices is shown, so you can override
   any of them in the boxes underneath.
4. **Start the scan**, then either **Take this view** at each position, or
   **Move and take them all** and let the arm do it. A view where too few
   points land on the part is refused rather than merged — a smeared or
   mismatched view adds noise at full weight, and afterwards nothing can tell
   which points came from it.
5. **Build the model.** A point confirmed by only one viewpoint is a point
   nothing has confirmed, so two are required by default. This is also why the
   view planning insists on real overlap.
6. **Plan the path on the model** — the path follows the measured surface, so
   the standoff is held even where the top is not flat. Cells with no
   measurement are skipped rather than interpolated: moving a probe to a
   standoff computed from a guessed height is how you hit the part.
7. **Save the model** writes a PLY that any 3D tool, and Isaac, will open.

Press **3D model** above the picture to look at what was built; height is
shown as colour.

### Find the part — one view, quicker

The original single-frame route, unchanged. Use it when the part is flat and
the shape is not in question.

---

## 7 · Record

One file per run, in the canonical schema. It now carries the robot state, the
inertial channels **with their derived orientation**, and every other
registered channel, so a sensor attached next month is recorded from the day
it is attached with no change to anything here.

Put the calibration version from step 5 in the box. The recorder refuses to
start without one.

**Check sensor timing** looks for one sharp tap across all inertial channels
and reports how far apart they saw it. Above about 5 ms, measure the offset
before recording — a temporal misalignment that is assumed away comes back
later as a position error blamed on the sim-to-real gap.
