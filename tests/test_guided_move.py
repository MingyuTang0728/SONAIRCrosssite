"""
Finding the three configurations, and driving to them only while held.

  * From a sensible start pose, three configurations are suggested whose
    every run keeps the tool inside the envelope -- and a session using them
    passes its own check.
  * From a pose already outside the envelope, nothing is suggested and the
    operator is told why.
  * The hold-to-move driver moves only while heartbeats arrive: letting go
    stops the arm, heartbeats that stop arriving stop it within the timeout,
    pressing again carries on, arrival is reported, and a move whose path
    leaves the envelope is refused before anything is sent.

Run:  python tests/test_guided_move.py
"""
from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import campaign_runner as cr                          # noqa: E402
import ur_control                                     # noqa: E402
import ur_kin                                         # noqa: E402

ENV = ur_control.Envelope()


class Arm:
    """movej that returns at once and runs in the background, like the robot."""

    def __init__(self, q):
        self.q = list(q)
        self.lock = threading.Lock()
        self.run = None
        self.moves = 0

    def joints(self):
        with self.lock:
            return list(self.q)

    def move(self, target, v, a):
        self.stop()
        self.moves += 1
        halt = threading.Event()

        def go():
            while not halt.is_set():
                with self.lock:
                    d = [t - c for t, c in zip(target, self.q)]
                    m = max(abs(x) for x in d)
                    if m < 1e-4:
                        self.q = list(target)
                        return
                    step = min(1.0, v * 0.01 / m)
                    self.q = [c + x * step for c, x in zip(self.q, d)]
                time.sleep(0.01)
        self.run = (halt, threading.Thread(target=go, daemon=True))
        self.run[1].start()
        return True, ""

    def stop(self):
        if self.run:
            self.run[0].set()
            self.run[1].join()
            self.run = None


def tcp(q, tool=(0, 0, 0.1)):
    T = ur_kin.fk(q)
    return list(T[:3, 3] + T[:3, :3] @ ur_kin.np.asarray(tool)) + [0, 0, 0]


def main():
    # 1. suggestions from a sensible start, and a session that passes with them
    q0 = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    res = cr.suggest(q0, tcp(q0), ENV.as_dict(), ENV.accepts_pose)
    assert res["ok"] and len(res["configs"]) == 3, res["problems"]
    st = {"configs": {c: {"q": v["q"], "direction": v["direction"]}
                      for c, v in res["configs"].items()},
          "done": {}, "rejected": {}, "sessions": {}}
    for s in (0, 1, 2):
        pv = cr.preview(s, st, ENV.accepts_pose, tcp(q0), q0)
        assert pv["ok"], pv["problems"]
    lows = {c: v["tool_low_cm"] for c, v in res["configs"].items()}
    print(f"  pass  three configurations suggested, every session passes its "
          f"check (lowest tool point {min(lows.values()):.0f} cm)")

    # 2. already outside: told so, nothing suggested
    q_out = [-1.6, -1.2, 1.3, -1.7, -1.57, 0.0]
    res = cr.suggest(q_out, tcp(q_out), ENV.as_dict(), ENV.accepts_pose)
    assert not res["ok"] and "outside the safe envelope" in res["problems"][0]
    print("  pass  from outside the envelope: nothing suggested, and why")

    # 3. the preview's tool offset turns with the flange
    q = list(q0)
    off = cr._tool_offset(q0, tcp(q0))
    q[2] += 1.0
    assert math.dist(cr._tool_at(q, off), tcp(q)[:3]) < 1e-9
    print("  pass  a tool offset is carried in flange axes, not base axes")

    # 4. hold to move
    arm = Arm(q0)
    target = list(q0)
    target[1] += math.radians(20)
    g = cr.HoldToMove(arm.move, arm.stop, arm.joints,
                      path_ok=lambda a, b: cr.transit_ok(a, b, ENV.accepts_pose))
    assert g.press(target, "extended")["ok"]
    for _ in range(6):
        time.sleep(0.15)
        g.beat()
    moved = g.remaining_deg()
    assert 5 < moved < 20, moved
    # let go
    g.release()
    a = g.remaining_deg()
    time.sleep(0.3)
    assert abs(g.remaining_deg() - a) < 0.05
    print(f"  pass  moves while held, stops on release ({20 - a:.1f} deg done)")

    # heartbeats stop without a release (network lost, browser frozen)
    g.press(target, "extended")
    time.sleep(0.1)
    t0 = time.monotonic()
    b0 = g.remaining_deg()
    while g.stops < 2 and time.monotonic() - t0 < 2:
        time.sleep(0.02)
    waited = time.monotonic() - t0
    assert g.stops == 2 and waited < cr.HOLD_TIMEOUT_S + 0.2, waited
    c = g.remaining_deg()
    time.sleep(0.3)
    assert abs(g.remaining_deg() - c) < 0.05 and c < b0
    print(f"  pass  heartbeats lost: the watchdog stopped the arm after "
          f"{waited + 0.1:.2f} s")

    # the whole process stalls (a driver holding the interpreter) while the
    # button is held: the heartbeats queued meanwhile must not be judged
    # missing the instant the watchdog wakes -- that stopped the real arm
    # after every stall, so it barely moved
    g.press(target, "extended")
    beating = threading.Event()
    beating.set()

    def beats():
        while beating.is_set():
            g.beat()
            time.sleep(0.15)
    threading.Thread(target=beats, daemon=True).start()
    time.sleep(0.4)
    before = g.stops
    old = sys.getswitchinterval()
    sys.setswitchinterval(5.0)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 1.2:       # hold the interpreter 1.2 s
        pass
    sys.setswitchinterval(old)
    time.sleep(0.6)
    assert g.stops == before, "a stall of the agent was taken for a release"
    beating.clear()
    time.sleep(0.8)
    assert g.stops == before + 1, "heartbeats stopped but the arm did not"
    print("  pass  a 1.2 s stall of the agent is not taken for a release; "
          "heartbeats that really stop still stop the arm")

    # press again and hold to arrival
    g.press(target, "extended")
    t0 = time.monotonic()
    r = {}
    while time.monotonic() - t0 < 10:
        time.sleep(0.15)
        r = g.beat()
        if r["arrived"]:
            break
    assert r["arrived"], r
    print("  pass  pressing again carries on, and arrival is reported")

    # a target whose path leaves the envelope: refused, nothing sent
    n = arm.moves
    low = list(arm.joints())
    low[1] += math.radians(70)
    low[2] += math.radians(40)
    r = g.press(low, "near_singular")
    assert not r["ok"] and "leaves the safe envelope" in r["error"], r
    assert arm.moves == n
    print("  pass  a move whose path leaves the envelope is refused unsent: "
          + r["error"][:70] + "...")
    print("all passed")


if __name__ == "__main__":
    main()
