"""
The recorder writes one row per robot packet -- not the latest packet when a
host timer happens to look.

The first two campaign sessions were recorded by polling the shared robot
state at 125 Hz. Packets arrive in bursts (the RTDE reader hands them over in
batches), so 41.6% of the rows held a distinct state: about 49 Hz of real data
under a 125 Hz label. This drives the recorder with a real URTelemetry
publishing 125 Hz packets in 40 ms bursts and checks:

  1. every packet is one row, none repeated, none missed;
  2. each row carries the controller's own timestamp (aux.controller_t),
     and the row times keep the true 8 ms spacing despite the bursts;
  3. the commanded channels the simulator is fed (target_q ...) are present;
  4. a gap in the stream is counted, not hidden;
  5. with no telemetry stream the recorder falls back to polling;
  6. the CSV writer keeps time columns to the microsecond.

Run:  python tests/test_recorder_packets.py
"""
from __future__ import annotations

import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import bench_agent                                    # noqa: E402
from sonair_benchmark.schema import read_run          # noqa: E402
from ur_telemetry import URTelemetry                  # noqa: E402

TMP = Path(tempfile.mkdtemp())
DT = 0.008


def stream(tel, n, burst=5, gap_at=None, gap_s=0.0):
    """n packets on the controller's 8 ms grid, delivered `burst` at a time."""
    t0 = time.perf_counter()
    k = 0
    while k < n:
        due = t0 + (k + burst) * DT + (gap_s if gap_at is not None and k >= gap_at else 0)
        while time.perf_counter() < due:
            time.sleep(0.001)
        for j in range(k, min(n, k + burst)):
            q = [0.0, -1.57, 0.25 * j * DT, -1.57, -1.57, 0.0]
            off = gap_s if gap_at is not None and j >= gap_at else 0.0
            tel._publish({"timestamp": 5000.0 + j * DT, "actual_q": q,
                          "actual_qd": [0, 0, 0.25, 0, 0, 0],
                          "actual_TCP_pose": [0.4, 0, 0.3, 0, 3.14, 0],
                          "target_q": q, "target_qd": [0, 0, 0.25, 0, 0, 0],
                          "target_moment": [0] * 6, "speed_scaling": 1.0},
                         "rtde", t_recv=t0 + j * DT + off)
        k += burst


def record(run_id, tel, **kw):
    rec = bench_agent.RECORDER
    rec.out_dir = TMP
    rec.packet_source = (lambda: tel) if tel is not None else None
    res = rec.start(run_id=run_id, joint_vel=0.25, arm_config="mid_workspace",
                    traj_type="contour", repeat_idx=0, calib_version="t",
                    rate_hz=125.0)
    assert res.get("ok"), res
    if tel is not None:
        stream(tel, **kw)
    else:
        time.sleep(1.0)
    stop = rec.stop()
    rec.packet_source = None
    return stop, read_run(stop["path"])


def main():
    bench_agent.RECORDER.state_fn = lambda: {
        "q": [0.0] * 6, "tcp": [0.4, 0, 0.3, 0, 3.14, 0],
        "target_q": [0.0] * 6, "qd": [0.0] * 6, "robot_age_s": 0.004}
    tel = URTelemetry("0.0.0.0", use_process=False)

    stop, run = record("pk_clean", tel, n=500)
    s = run.samples
    ct = [r["aux"]["controller_t"] for r in s]
    assert stop["mode"] == "packet", stop
    assert len(s) == 500, len(s)
    assert len(set(ct)) == 500, "a packet was written twice"
    assert ct == sorted(ct)
    assert all(abs((b - a) - DT) < 1e-6 for a, b in zip(ct, ct[1:]))
    dt = [b["t"] - a["t"] for a, b in zip(s, s[1:])]
    assert max(abs(d - DT) for d in dt) < 1e-4, max(dt)
    assert all("target_q" in r and "target_moment" in r for r in s)
    assert stop["skipped_intervals"] == 0
    print(f"  pass  {len(s)} packets in 40 ms bursts -> {len(s)} rows, every "
          f"one distinct, controller time kept, row spacing 8 ms "
          f"(worst {max(abs(d - DT) for d in dt) * 1e6:.0f} us off)")

    stop, run = record("pk_gap", tel, n=300, gap_at=150, gap_s=0.6)
    assert len(run.samples) == 300
    assert stop["skipped_intervals"] == 1 and stop["worst_gap_s"] >= 0.6, stop
    print(f"  pass  a {stop['worst_gap_s']:.2f} s hole in the stream is counted "
          f"in the run summary")

    assert not tel._sinks, "the recorder stayed subscribed after stopping"

    stop, run = record("pk_poll", None)
    assert stop["mode"] == "poll" and len(run.samples) > 50, stop
    print(f"  pass  no telemetry stream: falls back to polling "
          f"({len(run.samples)} rows in 1 s)")

    cells = [bench_agent._cell(v) for v in (1791312345.67, 10234.567891,
                                            1523.456789, 0.000123456789)]
    assert cells[0] == "1791312345.670000" and cells[1] == "10234.567891", cells
    assert cells[2] == "1523.45679" and float(cells[3]) == 0.000123456789, cells
    print("  pass  CSV keeps epoch and controller times to the microsecond")
    print("all passed")


if __name__ == "__main__":
    main()
