# Adding a modality

`sensor_hub.py` is the socket. A new sensor needs a descriptor and a
`read()` — not a change to the bridge, the recorder, the schema or the
console.

## The three-line version

```python
import sensor_hub

sensor_hub.HUB.declare(
    id="eddy0", label="Eddy current probe", modality="eddy_current",
    units="V (I/Q)", rate_hz=1000.0, frame="tcp", role="application",
    fields=["i", "q", "lift_off"])

sensor_hub.HUB.attach("eddy0", lambda: {"i": read_i(), "q": read_q()})
```

From that moment the channel appears in the console's channel table, is polled
on every recorded sample, and is written into every run file under
`sensors.eddy0`. Nothing else has to know it exists.

## What the descriptor is for

Two fields decide how the channel is treated, and they encode the benchmark's
selection rule rather than a preference:

**`modality`** maps to `SIMULATABLE`. A modality that cannot be simulated
faithfully cannot carry a sim-to-real gap number, whatever else it is good
for. Orientation, angular rate, acceleration and position can. Eddy current,
ultrasound, thermography and Raman cannot.

**`role`** is `"benchmark"` or `"application"`. Declaring a channel as
`benchmark` when its modality is not simulatable is reported as a conflict, in
the console, by name — because the distinction is the whole architecture of
the project and it must not depend on anyone remembering it.

`gt_cost` is the other half of the rule: a modality nobody can label cheaply
cannot anchor a campaign, and the table says so per modality.

## A channel with no hardware yet

Declare it anyway. `status` stays `declared`, the console shows it greyed out
with its expected rate and units, and the wiring, the schema and the screen
are all in place the day the sensor lands — which is the day you least want to
be writing integration code. The eddy current, ultrasonic and thermal channels
in `install_defaults()` are exactly this.

## Rules the hub enforces so you do not have to

- A driver that raises marks its own channel failed, with the message, and
  does not stop the others or end a recording. Over a four-week campaign a
  sensor *will* drop out, and the run in progress is still worth having.
- A stale channel is **omitted** from the recorded sample, not repeated. A
  recorder that keeps writing the last value of a dead sensor produces a file
  in which the dropout is invisible, and a dropout you cannot see is worse
  than a gap you can.
- `snapshot()` is what the recorder writes; `report()` is what the console
  draws. They are deliberately different — merging them would mean either
  recording health text into every sample or hiding staleness from the
  operator.

## Inertial sensors are a special case

They have their own path — `imu_link.py` for the transport and
`sonair_benchmark/attitude.py` for orientation — because orientation is a
scored modality and needs the estimator, the bias tracking and the
cross-check. Register a new inertial unit by starting a link for it
(`imu_link_start` with a new `unit` id); the attitude tracker, the recorder and
the console follow automatically.
