# Capturing the first dataset

Answers to the questions you asked, then the runbook.

---

## 1. Can the HTML read the IMU directly from FusionHub?

**No — and nothing will change that.** A browser has no UDP socket and no device
driver. FusionHub talks to the IMU through a driver and publishes over the
network; the page cannot reach either.

There are exactly two paths, and both are now implemented:

| Path | How it works | Use it when |
|---|---|---|
| **Host agent** (default) | FusionHub streams JSON over UDP to port 5005 on the workstation. `bench_agent.FusionHubBridge` listens, stamps against the master clock, and the bridge relays to the page. | Always. This is the path the benchmark recorder uses — only samples that pass through here get timestamped and recorded. |
| **Direct WebSocket** | If your FusionHub build can serve a WebSocket, the page connects to it itself. | Quick sanity check with no agent running. It bypasses the master clock, so it is **not** a recording path. |

Switch between them in **Robot & 3D scan → 3 · Inertial**.

Configure FusionHub to publish JSON over UDP to `127.0.0.1:5005`. The parser
accepts the field spellings FusionHub actually emits — `qw/qx/qy/qz` or
`w/x/y/z`, `gx/gy/gz` or `gyro_x/...`, `ax/ay/az` or `accel_x/...`.

If the live stream fights you, **record in FusionHub and use the replay path**:

```bash
python -m sonair_benchmark phase0 --fusionhub exported.csv --expected-hz 200 \
    --out phase0/ind0.json
```

The canonical record is identical either way, so nothing downstream changes when
you later switch to live.

> One thing that is not optional: FusionHub timestamps in **its own clock**. The
> parser returns that timestamp unconverted on purpose. Measure the offset with
> the tap test before you trust any cross-channel timing.

---

## 2. Can the camera display in the HTML, or is RealSense Viewer the only option?

**It displays in the HTML.** Three routes, in descending order of usefulness:

| Route | Streams | Needs |
|---|---|---|
| **Host agent** (what you have) | RGB, depth, IR1, IR2, and the metric raw depth used by 3D scanning | `pyrealsense2`, `opencv-python`, `numpy` on the workstation |
| **`getUserMedia`** | RGB only | Nothing. The D435i's colour sensor enumerates as an ordinary UVC webcam, so a browser can open it directly — but depth and IR do not, and there is no metric data |
| RealSense Viewer | everything | Separate app, cannot feed your page |

You are already on the first route. It was already in `multimodal_bridge.py`;
what was missing is that the **colourised depth image the UI shows has no metric
content left in it**. The bridge now also keeps the raw uint16 depth frame and
the camera's own intrinsics, which is what makes the 3D pipeline possible.

Practical requirements: a **USB 3.0 port and the cable that came with the
camera**. On USB 2.0 the D435i enumerates but silently drops to a reduced stream
set, and it reads as a bandwidth bug rather than a cable problem.

**Also: the D435i contains an IMU** (a BMI055). `bench_agent.D435iImuSource`
opens it on its own pipeline — separate from the image pipeline on purpose,
because the motion streams run at 200 Hz and 63 Hz against the images' 30 Hz,
and forcing them into one `wait_for_frames()` throttles the IMU to the frame
rate. It is consumer grade and not a substitute for the industrial unit, but it
costs nothing and works the moment the cable is in.

---

## 3. Is the "scan → reconstruct → locate → plan path" algorithm implemented?

**It was not. It is now** — `scan3d.py`, driven from **Robot & 3D scan → 4**.

What the console had before was a *planar* workflow: pick four corners on a
photo, fit a homography, raster inside the rectangle. That works for a flat
coupon and nothing else.

The new pipeline:

1. **Survey poses** — a ring of viewpoints around the work centre, each looking
   inward and down. Multiple angles are the point: a single top-down sweep
   cannot see vertical faces, and voxel averaging only reduces noise across
   genuinely different viewpoints.
2. **Capture** — at each pose, deproject the raw depth image into camera-frame
   points using the camera's own intrinsics.
3. **Fuse** — transform into the robot base frame through
   `T_base_tcp · T_tcp_cam` and accumulate into a voxel grid, keeping a running
   mean per voxel. Memory is bounded by the working volume, not by view count.
4. **Segment** — RANSAC out the dominant plane (the table), keep only what sits
   *above* it, then flood-fill for the largest connected blob.
5. **Locate** — PCA for the oriented bounding box. The third axis is forced to
   Z-up rather than taken from PCA: for a flat-ish part the two in-plane
   eigenvalues are close, PCA's third axis flips sign between runs, and a path
   built on a flipped axis runs backwards.
6. **Plan** — build a height field and raster over it at a constant standoff, so
   a stepped or curved top surface keeps the sensor at the same distance instead
   of the fixed Z a planar path would give. Cells with no measurement are
   **skipped, never interpolated** — moving a probe to a standoff computed from
   a guessed height is how you crash into the part.

Verified on a synthetic scene: a 120 × 80 mm block recovers as **123 × 83 mm**,
centre within 0.5 mm, path at the commanded standoff with 100% coverage.

### What it does not do

It builds a **height field of what the camera can see from above**. It is not
SLAM and it is not CAD registration. A part with undercuts, or a job needing
full 360° coverage, needs multi-side capture and a real surface reconstruction —
this pipeline would mislead you there rather than fail visibly.

### The thing that actually limits accuracy

Not the depth sensor — the **hand-eye calibration**. A D435i at 300 mm has
roughly 1–2 mm depth noise, and that averages down over views. A 2° error in
`T_tcp_cam` does not average down at all and puts the whole cloud 10 mm out at
300 mm standoff. The matrix in the panel is a placeholder; measure yours.

---

## 4. Why the GLB import stopped working

`THREE is not defined`. The page loaded three.js only from unpkg and jsdelivr.
When both are blocked — routine on a university network, and guaranteed if you
open the file as `file://` with no internet — three.js never arrives, `init3D`
throws on its first line, and the GLB import then silently does nothing.

Fixed three ways:

1. three.js, the loaders and the **DRACO decoder** are vendored in
   `vendor/three/` and tried **before** the CDNs. No internet needed.
2. `init3D` is not called at all when THREE is missing, so the failure cannot
   cascade into a silent import.
3. A failure paints an explanation on the stage instead of leaving an empty box.

The DRACO part matters separately: production UR meshes are usually
DRACO-compressed, the decoder was fetched from `gstatic.com`, and when that is
blocked a compressed GLB fails while an uncompressed one works — a baffling
symptom.

**Serve the page over HTTP, not `file://`:**

```bash
cd /path/to/SONAIRCrosssite
python -m http.server 8000
# then open http://localhost:8000/Remote_control_Benchmark.html
```

`file://` blocks `fetch` of the vendored scripts under most browsers' CORS rules.

---

## 5. Is the uploaded version of the HTML good?

Yes — keep it. It is more disciplined than the version it replaced, in three
ways worth naming:

- It **states what it is not**. "PLANAR VISION PILOT", "local_unverified",
  "Path points are camera optical-centre positions in the part frame, not
  executable TCP poses", "Corner selection is user-assisted localisation, not
  automatic 6D pose estimation". That is the right instinct for a benchmark.
- The CSV scorer **rejects on any invalid row rather than dropping rows**, and
  checks quaternion norms, monotonic time and finiteness. Silently dropping bad
  rows is how a scorer ends up reporting on a different dataset than it claims.
- `score_orientation.py` separates macro-average sequence RMSE from pooled frame
  P95, and refuses to compute a transfer gap when the condition sets differ.

Two gaps that are now filled: it had **no UR data beyond pose**, and its
"Next backend connections" note correctly identified that IMU telemetry needed a
bridge that did not exist yet. Both are built.

One observation on the scorer. `p95 = sorted(a)[ceil(.95*len(a))-1]` is the
nearest-rank definition, which is fine and defensible — but it differs from the
linear-interpolated percentile in `sonair_benchmark/metrics.py`. For sequences
of a few hundred samples the two agree to well under a tenth of a degree, so it
does not change any conclusion; it is worth stating which definition a published
number used.

---

## 6. Capturing the first dataset

### Before you plug anything in

Weigh the printed bracket and measure the IMU mounting offsets. Ten minutes, and
both numbers feed the error budget *and* the Isaac contract.

### Step 1 — Install the host dependencies

On the workstation wired to the UR5e:

```bash
pip install pyrealsense2 opencv-python numpy websockets
```

`numpy` is required for 3D scanning; without it that panel refuses to start
rather than producing wrong geometry.

### Step 2 — Start the agent

```bash
cd /path/to/SONAIRCrosssite
export UR_IP=192.168.0.20            # your controller
export BENCH_FUSIONHUB_PORT=5005
python multimodal_bridge.py
```

Expect:

```
UR service:   {'ok': True, 'host': '192.168.0.20', ...}
RTDE streaming 30 fields at 125 Hz (controller 5.x.x.x)
depth intrinsics fx=... fy=... cx=... cy=... scale=0.001
FusionHub listener on UDP 5005 as unit ind0
D435i IMU started (accel 63 Hz, gyro 200 Hz)
```

If you see `RTDE unavailable … falling back to port 30003`, RTDE is disabled or
another client holds it. The fallback still carries force, currents and
temperatures; it does not carry IO bits, program state or speed scaling.

### Step 3 — Serve and open the console

```bash
python -m http.server 8000
```

Open `http://localhost:8000/Remote_control_Benchmark.html` → **Robot & 3D scan**
→ Connect bridge → Start UR service.

You should see the force card reading, joints populating, and the inertial table
showing `ind0` at ~200 Hz.

> **Put the pendant in Remote Control.** External URScript is rejected in local
> mode, and the symptom is a connection timeout rather than a clear refusal.

### Step 4 — Set the payload, then zero the F/T sensor

In that order. Zeroing with the payload wrong subtracts gravity along with the
offset, and every later force reading is out by the weight of the tool.

### Step 5 — Phase 0 checks (no robot motion)

1. **Stationary log, one hour.** Clamp the IMU, surface isolated from foot
   traffic. Then:
   ```bash
   python -m sonair_benchmark phase0 --fusionhub stationary.csv \
       --expected-hz 200 --out phase0/ind0.json
   ```
   Watch the jitter figure. Above ~2 ms and a drifting rate will read downstream
   as a velocity-dependent gap that is not real.

2. **Six-position tumble test**, gravity as reference.

3. **Tap test.** Tap the carrier once, firmly → *Verify tap alignment*. The
   spread across channels is the temporal row of the error budget and gets
   quoted in every later result.

### Step 6 — Error budget

```bash
python -m sonair_benchmark budget --phase0 phase0/ind0.json \
    --tap-spread-ms 0.8 --tracker-mm 1.2 --frame-fit-mm 0.8 \
    --calib-version calib-1 --out calib/budget.json
```

### Step 7 — First real run

```bash
python -m sonair_benchmark plan --out campaign/plan.json
```

Then, per run: fill the manifest (elbow velocity, configuration, trajectory
type, repeat index, **calibration version**) → *Record run* → execute the
motion → *Stop*. One JSONL file per run lands in `bench_runs/`.

Start with three runs at 0.4 rad/s and three at 0.9 rad/s, mid-workspace,
contour. Six runs is enough to see whether the gap has the velocity dependence
the benchmark is built around, before committing four weeks to the full sweep.

### Step 8 — 3D scan (once the hand-eye transform is measured)

**Robot & 3D scan → 4**: set hand-eye → start session → generate viewpoints →
**review the TCP poses against your envelope** → run sweep → reconstruct → plan
→ *Send to Zig-Zag preview* → review the 3D overlay in Sensor console → execute.

The sweep moves the arm through every viewpoint. Each capture happens only after
a settle delay — a view taken mid-motion smears the depth image, and the smear is
fused in permanently.

### Step 9 — The simulated counterpart

*Export scan path for Isaac* writes the command file. `sonair_benchmark.isaac`
holds the contract for the four things Isaac must match, and
`write_isaac_stub()` emits a replay skeleton.

---

## 7. What is still missing, honestly

| Gap | Impact | What it needs |
|---|---|---|
| **Hand-eye calibration procedure** | Bounds all 3D accuracy | Pose the arm to a fixed target from ≥10 orientations and solve AX = XB. The panel accepts the matrix; nothing computes it yet |
| **Inverse kinematics** | Survey poses are sent as `movej` with a pose target, so the controller picks the configuration. It may pick one that hits something | A checked IK with joint limits and collision awareness |
| **Camera-to-part rotation about the optical axis** | The planner emits a fixed tool rotation | Specify the required tool orientation per surface normal |
| **RTDE input registers beyond the speed slider** | Only matters if a URScript program on the robot needs to read from the PC | Extend `RTDEInputChannel` |
| **Undercuts and 360° coverage** | Height field cannot represent them | Multi-side capture and a real surface reconstruction |

None of these blocks the first dataset. The first four block *trusting* the 3D
path for production inspection, which is a different bar.
