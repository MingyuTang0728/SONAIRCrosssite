# SONAIR console — the whole run, in order

Everything below is done from one page: `SONAIR_Console.html`, with
`multimodal_bridge.py` running on the same PC. The numbered steps in the left
rail are the order to do them in. Nothing here needs a terminal once the agent
is started.

---

## Before anything: what has to be installed

In the **same interpreter** the agent runs in:

```
python install_deps.py
```

Run that with **the same Python you run the agent with**, and it installs
whatever is missing into that interpreter. Use it rather than typing a `pip`
line: on a machine with a system Python, a virtual environment and an IDE
interpreter, a bare `pip install` lands in whichever is first on PATH — the
install succeeds, the import still fails, and the instruction looks wrong.
Naming the interpreter explicitly fixes that and introduces a second trap,
because PowerShell parses a command beginning with a quoted path as a string
expression and refuses it with `unexpected token '-m'` unless you remember
the `&` call operator. A script has no path to quote.

The packages it installs: `websockets` (required), then `numpy`,
`opencv-python`, `pyzmq`, `pyrealsense2` and `pyserial` for 3D, calibration,
FusionHub's External Output, the camera, and a COM-port IMU respectively.

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

In FusionHub 0.1.x the relevant graph nodes are **External Output**, **TCP
Output** (under CONNECTORS) and **WebSocket Sink**. All accept `Any` data
type, so any of them will carry the LPMS source's `Imu` output without a
converter in between.

**External Output is a ZeroMQ publisher, not a raw socket.** Its properties
show one field, `TCP ENDPOINT`, reading something like `tcp://*:8901` — that
is ZeroMQ's address syntax: `*` means "bind every interface", the port is the
number after the colon, and FusionHub is the server. Choose the transport
**FusionHub's External Output** and paste that endpoint in verbatim; the
wildcard is rewritten to localhost for you.

**It publishes Protocol Buffers, not text.** There is no format option on
the node. Protobuf carries field numbers but no field names, and the `.proto`
is not published, so the channels are identified from the readings instead:
a unit-length 4-vector is an orientation, a 3-vector averaging 9.8 (or 1.0)
is gravity, a 3-vector averaging 20-90 is a magnetometer, and what is left is
the gyroscope. Gravity is the anchor — a sensor on a bench or on an arm
averages 1 g whatever else it is doing, and nothing else in an inertial
message sits at that magnitude. Only magnitudes are used, never directions,
so it holds however the unit is mounted.

The mapping is learned over the first twenty-odd packets, then LOCKED to the
field numbers it found, and reported on the link — from that point the field
numbers are exact and the physics is only how they were discovered. A wrong
mapping is visible immediately: orientation that does not follow the sensor.

Pointing a plain TCP transport at it is the trap: the TCP handshake succeeds,
the link reports connected, and what arrives is ZeroMQ's own protocol rather
than your data. **Show what is arriving** names that case — it reports the
format as `zmtp` and says which transport to switch to.
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
  - *sensor and our own estimate agree on which way is down* — an independent
    orientation estimate is run alongside the sensor's own and the two are
    differenced. Agreement under a degree or two means both are working; a
    large number means one of them is not, and it has caught a wrongly
    configured output frame more than once. Only the **gravity direction** is
    compared, and deliberately: our estimate has no compass, so its heading
    starts at zero and stays arbitrary, while the sensor fuses a magnetometer
    and reports a real one. Differencing the whole rotation reported that
    heading offset — routinely over a hundred degrees — as though it were an
    error, which read as a broken sensor and was nothing of the kind.
  - *the sensor's own clock jumped* — a timestamp that went backwards or
    leapt. The readings are still good; their spacing is taken from arrival
    time instead. One such jump used to freeze the update rate at 0 Hz while
    data was plainly arriving.
  - *drift correction* — the gyroscope bias learned while the unit is still.
    A consumer part routinely shows 1-2 °/s; uncorrected, that is 60-120° of
    pure fiction over a minute.
- **Re-level** — hold the unit still and press it. Clears the learned bias and
  re-seeds the orientation from gravity.
- **Every reading, live** — one card per measurement (Euler angles,
  accelerometer, gyroscope, magnetometer, quaternion, linear acceleration,
  environment), each with its own chart, all on screen together. They are read
  *against* each other: a spike in the accelerometer with no matching spike in
  the gyroscope is a knock; one that has a match is the arm turning; one in the
  magnetometer alone is something ferrous moving nearby. A card whose sensor
  does not provide that channel says so rather than showing an empty chart.
- **Move the robot** — the jog dock, bottom right of every page. The board, the
  charts and the arm are one job, so the control travels with you instead of
  living on the Robot page. It is the same jog path: one velocity, one
  host-side 20 Hz loop, one watchdog. Collapsing it also stops the arm.

### Getting the readings out

Two different things, and the difference matters.

- **Record every sample to a file** writes each reading to a CSV as it
  arrives, for as long as you leave it running, at the sensor's own rate. This
  is the one to use for an experiment. Files land in `imu_logs/` next to the
  agent, one row per sample with a fixed column set — columns a unit does not
  provide are present and empty, which is a statement, where a missing column
  is a question.
- **Save what is on screen** writes out only what is still in memory: a few
  thousand readings per sensor, around forty seconds at 100 Hz. Use it when
  something has just happened and you want *that*. **Download as a
  spreadsheet** then puts a second copy on the machine you are sitting at,
  which is not always the machine the agent runs on.

The charts are drawn about twenty times a second, which is a picture of the
sensor rather than all of it. The recorded file has every reading.

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

0. **Set the colour stream high enough first.** The board is detected in the
   colour image and the corner detector needs roughly 15 pixels between
   corners. A 7.5 mm square at 400 mm standoff lands on about 11 px at
   640×360 and about 23 px at 1280×720 — which is the difference between a
   board that is found and one reported as absent while it sits plainly in
   shot, in focus and well lit. 1280×720 is now the default; if you lowered
   it, put it back on the Camera page before calibrating. The live view
   reports the measured square pitch on every successful detection, so you can
   watch the margin fall as the arm backs away.
1. Fix a chessboard where the camera can see it and the arm can move around
   it. Enter the **inner corner** counts — a board of 10×7 squares is 9×6
   inner. Getting this wrong is the commonest way to end up with a
   calibration that looks fine and is wrong. If the count is wrong, the
   console searches the nearby sizes and offers the one it actually found as
   a button: press it and the counts are corrected for you.
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
