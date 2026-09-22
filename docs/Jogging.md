# Moving the robot

## Why it was jerky

Measured, not guessed. The old jog ran a `setInterval(100)` in the browser that
sent one `speedl(t=0.25)` per tick. On this console's own page, under its real
load — camera frames decoding and the 3D view rendering — that timer fires at:

```
min 111 | median 248 | p95 300 | max 319 ms
ticks later than 250 ms: 27 of 59
```

A `speedl` runs for `t` seconds and then the arm decelerates at `a`. Nearly
half the ticks arrived after the previous command had already expired, so the
arm braked to zero and re-accelerated, over and over. That is the stutter.

The browser timer was late because the main thread was saturated: two base64
JPEG streams were being decoded with `new Image()` and a data URL, which
decodes **on** the main thread, while three.js rendered at 60 fps.

**The design was wrong, not just the numbers.** Robot motion must never depend
on a browser's timing. No amount of tuning the interval fixes a page that can
stall for 300 ms whenever it feels like it.

## What it does now

The browser sends **intent** and nothing else:

```
jog_vel   here is the velocity I want, and how long to trust it
jog_stop  stop
```

`ur_jog.py` on the host owns the timing. A thread re-issues the command at a
steady **20 Hz with `t = 0.15 s`** — three times the overlap needed, so two
consecutive sends can be lost and the arm still never sees a gap.

Measured through the whole chain, browser to robot, while holding a pad for
three seconds under the same page load:

| | before | after |
|---|---|---|
| command interval | median 248 ms | median **50 ms** |
| intervals over 150 ms | 44 of 59 | **0** of 95 |
| drops to zero mid-move | many | **0** |

Three further things the host does that a browser cannot:

- **Ramps.** Direction changes are slew-limited (0.6 m/s per second linear,
  2.5 rad/s per second angular). A UR will happily step its commanded velocity;
  the mechanism will not, and the operator feels the difference as a knock.
- **Watchdog.** If no update arrives for 400 ms the arm ramps to zero and
  stops. It runs on the host's monotonic clock, never on a timestamp the
  browser supplies — a stalled browser is exactly the case where its clock
  cannot be trusted and exactly the case where the arm must stop.
- **One stop, then silence.** Streaming zeros at 20 Hz keeps interrupting the
  controller's program for no benefit.

Jog messages are also handled **inline** on the socket's own task rather than
through `asyncio.to_thread`. They are a lock and six floats; a thread-pool hop
per message would reintroduce the latency the whole change exists to remove.

The browser sends from `requestAnimationFrame`, not `setInterval` — but nothing
depends on it arriving on time any more, which is the point.

### The main thread was also fixed

- Frames now decode with `createImageBitmap`, which runs **off** the main
  thread. A frame is dropped rather than queued if one is already decoding: at
  30 fps a backlog only grows, and a late frame is worth less than the
  main-thread time spent on it.
- Only the stream you are looking at is decoded.
- The 3D view does not render while its page is hidden.

---

## Three ways to move it

### Smooth — for getting roughly there

Two on-screen pads, or the keyboard, or a gamepad. They all feed the same
velocity vector, so you can use whichever suits.

| Key | Does |
|---|---|
| &larr; &rarr; | left / right |
| &uarr; &darr; | forward / back |
| Q / E | down / up |
| A / D | rotate the tool |
| Shift | hold for quarter speed |
| Space | stop |

Tick **Keyboard control on** first. A gamepad is picked up automatically —
plug one in and press a button; left stick moves across the table, right stick
lifts and turns.

### Precise steps — for anything repeatable

Pick a step size (0.1, 1, 5, 10 or 50 mm) and press an axis button. Each press
is one `movel` to an absolute target, so it moves **exactly** that far and
stops. A browser stall cannot shorten or lengthen it — this mode is immune to
timing by construction, which is why it is the right choice for setting a
position you need to hit again.

Rotation steps are in degrees and capped at 5° per press: a rotation vector is
not three independent angles, so only small increments are meaningful.

### Table axes or tool axes

**Table axes** moves along the world X, Y and Z. **Tool axes** moves along the
tool's own directions, which is what you want when the tool is tilted and you
need to back straight off a surface.

---

## If it still does not move

The console says which of these it is, but in order of likelihood:

1. **The pendant is in Local mode.** External URScript is refused, and the
   symptom is a connection timeout rather than a clear refusal. Switch to
   Remote Control, top right of the pendant.
2. **Outside the working envelope.** The command is rejected before it is sent
   and the console names the axis.
3. **Protective stop.** Clear it on the pendant first.

The **STOP ROBOT** button clears the held keys and pads, halts the jog
controller without a ramp, and sends a stop. It is a software stop; the
physical e-stop is the safety device.
