"""
The robot link must survive the agent freezing.

The controller closes an RTDE client that stops draining its buffer. On the
cell the agent process froze for seconds at a time -- all threads at once,
because a C extension held the interpreter lock -- and each time the
controller closed RTDE ("RTDE connection closed by controller") and the cell
dropped onto the fallback interface. This test freezes the agent for ~3 s
against a stand-in controller that drops slow clients the same way, and
requires the link to come through it on RTDE with nothing dropped.

Run:  python tests/test_robot_link.py
"""
import subprocess, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import ur_telemetry as T


def _freeze(seconds_hint=3.0):
    big = [(i * 7919) % 1000003 for i in range(int(4_000_000 * seconds_hint))]
    t0 = time.perf_counter(); big.sort()      # list.sort never yields the GIL
    return time.perf_counter() - t0


def main():
    srv = subprocess.Popen([sys.executable, str(HERE / "fake_ur.py")],
                           stdout=subprocess.PIPE, text=True)
    try:
        T.RTDE_PORT = int(srv.stdout.readline()); T.PRIMARY_PORT = 1
        t = T.URTelemetry("127.0.0.1")
        t.start(); time.sleep(2.5)
        assert t.health.mode == "process", t.health.mode
        assert t.health.source == "rtde", t.health.source
        n0 = t.health.packets
        held = _freeze()
        time.sleep(2.0)
        ok = (t.health.source == "rtde" and t.health.rtde_drops == 0
              and t.health.packets > n0 + 125 * held)
        print(f"  froze the agent for {held:.1f}s: source={t.health.source} "
              f"drops={t.health.rtde_drops} packets {n0}->{t.health.packets} "
              f"stalls seen={t.health.host_stalls}")
        assert t.health.host_stalls >= 1, "the stall watch missed the freeze"
        assert ok, "the robot link did not survive the agent freezing"
        st = t.state()
        assert st.get("_mono"), "no receive time on the state"
        t.stop()
        print("  pass  test_robot_link_survives_agent_freeze")
    finally:
        srv.kill()


def drop_recovers_fast():
    """
    A controller that drops a WORKING stream once must see the agent back
    within a second. The old backoff doubled to ten seconds whatever had
    happened, which is where the 13-14 s holes in the arc scan came from.
    """
    import os
    env = dict(os.environ, FAKE_DROP_FIRST_AFTER="3")
    srv = subprocess.Popen([sys.executable, str(HERE / "fake_ur.py")],
                           stdout=subprocess.PIPE, text=True, env=env)
    try:
        T.RTDE_PORT = int(srv.stdout.readline()); T.PRIMARY_PORT = 1
        t = T.URTelemetry("127.0.0.1")
        t.start()
        # The age of the newest robot state, sampled throughout. Measuring
        # the difference between successive states instead misses a link
        # that never comes back at all -- a first version of this test passed
        # a reader that had gone silent for good.
        worst, t_end = 0.0, time.time() + 7.0
        time.sleep(0.5)
        while time.time() < t_end:
            m = t.state().get("_mono")
            if m:
                worst = max(worst, time.perf_counter() - m)
            time.sleep(0.005)
        print(f"  controller dropped the stream once: worst gap in robot "
              f"state {worst:.2f}s, drops recorded {t.health.rtde_drops}")
        assert t.health.rtde_drops >= 1, "the fake never dropped"
        assert worst < 1.0, "a single drop cost more than a second of data"
        assert t.health.source == "rtde"
        t.stop()
        print("  pass  test_single_drop_recovers_within_a_second")
    finally:
        srv.kill()


if __name__ == "__main__":
    main()
    drop_recovers_fast()
