"""
The simulated cell and the live twin.

  1. RTDE text messages are read in both layouts controllers use (PolyScope
     5.11 and 5.2x), and a message arriving in the middle of the handshake no
     longer puts every later reply out of step -- the fault that cost the
     RTDE link against URSim 5.26 and dropped it onto port 30003;
  2. sim_cell, in front of a stand-in controller, is read by the agent's own
     robot link as a robot -- and announces itself, so the link knows it is
     simulated;
  3. with the MuJoCo plant, the measured joints are the plant's (they trail
     the command) and a simulated IMU arrives over UDP;
  4. a run recorded while simulated is labelled side "sim", says so in its
     notes, and is written apart from the real runs; so are the logs;
  5. the twin follows the commanded joints with the benchmark's model and
     reports its difference from the real arm; without MuJoCo it says why.

Run:  python tests/test_sim_cell.py     (3 and 5 need mujoco + MENAGERIE)
"""
from __future__ import annotations

import json
import math
import os
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import ur_telemetry as urt                            # noqa: E402
import sim_cell                                       # noqa: E402
import fake_ur                                        # noqa: E402

TMP = Path(tempfile.mkdtemp())


def free_port(ip="127.0.0.1"):
    s = socket.socket(); s.bind((ip, 0)); p = s.getsockname()[1]; s.close()
    return p


def menagerie():
    for d in (os.environ.get("MENAGERIE"), HERE.parent / "mujoco_menagerie"):
        if d and (Path(d) / "universal_robots_ur5e" / "scene.xml").exists():
            return str(d)
    return None


def test_text_messages():
    m, src = b"SafetySetup has not been confirmed yet", b"URControl"
    assert urt.parse_text_message(b"\x02" + m) == m.decode()
    assert urt.parse_text_message(bytes([len(m)]) + m + bytes([len(src)]) + src
                                  + b"\x02") == m.decode()
    print("  pass  text messages read in both the 5.11 and the 5.2x layout")


class ChattyRTDE(fake_ur.FakeRTDE):
    """A controller that says something in the middle of the handshake."""

    class _Sock:
        def __init__(self, s):
            self._s = s

        def __getattr__(self, k):
            return getattr(self._s, k)

        def sendall(self, data):
            self._s.sendall(data)
            if data[2:3] == b"V":
                self._s.sendall(sim_cell._text_message(
                    "SafetySetup has not been confirmed yet"))

    def _serve(self, c):
        return super()._serve(self._Sock(c))


def test_handshake_survives_a_message():
    fake = ChattyRTDE()
    cl = urt.RTDEClient("127.0.0.1", fake.port)
    cl.connect()
    granted = cl.setup_outputs(urt.OUTPUT_RECIPE[:6])
    cl.start()
    assert granted and cl.read() is not None
    assert "SafetySetup" in cl.last_message
    cl.close()
    print("  pass  a controller message mid-handshake is skipped and kept: "
          f"\"{cl.last_message}\"")


def cell(plant_kind="ursim"):
    os.environ["FAKE_Q"] = "0,-1.57,1.57,-1.57,-1.57,0"
    fake = fake_ur.FakeRTDE()
    port = free_port("127.0.0.2")
    imu_port = free_port()
    plant = sim_cell.Plant(plant_kind, menagerie(), imu_noise=False,
                           carrier_file=str(TMP / "none.json"),
                           imu_cal_file=str(TMP / "none.json"))
    c = sim_cell.SimCell("127.0.0.1", "127.0.0.2", plant,
                         imu_to=("127.0.0.1", imu_port), rtde_port=port,
                         ursim_rtde_port=fake.port, relay_ports=()).start()
    assert c.ready.wait(10)
    return c, port, imu_port


def test_agent_reads_the_cell():
    c, port, _ = cell("ursim")
    old = urt.RTDE_PORT
    urt.RTDE_PORT = port
    try:
        tel = urt.URTelemetry("127.0.0.2", use_process=False)
        tel.start()
        end = time.time() + 8
        while time.time() < end and not tel.state().get("actual_q"):
            time.sleep(0.1)
        h = tel.status()["health"]
        st = tel.state()
        tel.stop()
    finally:
        urt.RTDE_PORT = old
        c.stop()
    assert h["source"] == "rtde" and h["simulated"], h
    assert "SONAIR simulated cell" in h["controller_message"]
    assert abs(st["actual_q"][1] + 1.57) < 1e-6 and st.get("target_q")
    print(f"  pass  the agent's robot link reads the cell over RTDE and knows it "
          f"is simulated (\"{h['controller_message'][:50]}...\")")


def test_mujoco_plant_and_imu():
    if not menagerie():
        print("  skipped  MuJoCo plant (set MENAGERIE to a menagerie clone)")
        return
    c, port, imu_port = cell("mujoco")
    got = []
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.bind(("127.0.0.1", imu_port)); u.settimeout(2)
    for _ in range(50):
        try:
            got.append(json.loads(u.recv(4096)))
        except socket.timeout:
            break
    c.stop()
    assert len(got) >= 30, len(got)
    acc = math.sqrt(sum(got[-1][k] ** 2 for k in ("accel_x", "accel_y", "accel_z")))
    assert abs(acc - 9.81) < 0.3, acc
    assert c.plant.arm is not None and c.packets > 50
    print(f"  pass  MuJoCo plant behind the cell; simulated IMU over UDP "
          f"({len(got)} readings, |a| = {acc:.2f} m/s^2 at rest)")


def test_simulated_runs_are_kept_apart():
    import bench_agent
    bench_agent.RECORDER.out_dir = TMP / "runs"
    bench_agent.RECORDER.state_fn = lambda: {
        "q": [0.0] * 6, "tcp": [0.4, 0, 0.3, 0, 3.14, 0], "target_q": [0.0] * 6,
        "qd": [0.0] * 6, "robot_age_s": 0.004}
    bench_agent.RECORDER.packet_source = None
    bench_agent.SIMULATED = lambda: True
    try:
        res = bench_agent.RECORDER.start(
            run_id="v0p50_mid_workspace_contour_r00", joint_vel=0.5,
            arm_config="mid_workspace", traj_type="contour", repeat_idx=0,
            calib_version="t", notes="rehearsal")
        assert res["ok"], res
        time.sleep(0.3)
        stop = bench_agent.RECORDER.stop()
        assert bench_agent._sim_prefix() == "simcell_"
    finally:
        bench_agent.SIMULATED = lambda: False
    from sonair_benchmark.schema import read_run
    p = Path(stop["path"])
    run = read_run(p)
    assert p.parent.name == "simcell", p
    assert run.manifest.side == "sim" and "SIMULATED CELL" in run.manifest.notes
    assert not list((TMP / "runs").glob("*.jsonl")), "a simulated run reached the real folder"
    print("  pass  a run on the simulated cell is labelled side=sim, says so, and "
          "is written to runs/simcell/, never beside the real runs")


class Feed:
    """A telemetry stand-in that publishes what it is given."""

    def __init__(self):
        self.sinks = []

    def subscribe(self, fn):
        self.sinks.append(fn)

    def unsubscribe(self, fn):
        self.sinks.remove(fn)

    def push(self, st):
        for fn in list(self.sinks):
            fn(st)


def test_twin():
    import twin
    t = twin.Twin()
    if not menagerie():
        os.environ.pop("MENAGERIE", None)
        res = t.start(Feed())
        if not res["ok"]:
            assert res["error"]
            print(f"  skipped  twin (and said why: {res['error'][:60]}...)")
            return
    os.environ["MENAGERIE"] = menagerie()
    feed = Feed()
    assert t.start(feed)["ok"], t.why
    q0 = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    ts = 100.0
    # an ideal arm: the real joints are exactly the command, which sweeps the
    # elbow at 0.5 rad/s for a second
    for k in range(250):
        q = list(q0)
        q[2] += 0.5 * min(k, 125) * 0.008
        feed.push({"timestamp": ts + k * 0.008, "target_q": q, "actual_q": q})
    snap = t.snapshot()
    t.stop()
    assert snap["enabled"] and len(snap["q"]) == 6
    assert snap["peak_deg"][2] > 0.5, snap["peak_deg"]      # S0 trails the command
    assert not feed.sinks, "the twin stayed subscribed after stopping"
    print(f"  pass  twin runs the benchmark's model on the commanded joints: "
          f"elbow trails by up to {snap['peak_deg'][2]:.1f} deg on a 0.5 rad/s "
          f"sweep ({snap['model'][:40]}...)")


def main():
    test_text_messages()
    test_handshake_survives_a_message()
    test_agent_reads_the_cell()
    test_mujoco_plant_and_imu()
    test_simulated_runs_are_kept_apart()
    test_twin()
    print("all passed")


if __name__ == "__main__":
    main()
