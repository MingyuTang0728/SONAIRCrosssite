"""
imu_probe.py — capture what the IMU source is actually sending, and decode it.

Run this when the console says it is connected and the readings stay empty.
It uses the same transports and the same decoder as the agent, but it prints
everything instead of only the result, and it SAVES THE RAW PACKETS so the
exact bytes can be looked at later. A screenshot of a hex dump loses bytes;
a file does not.

    python imu_probe.py --zmq tcp://*:8901
    python imu_probe.py --udp 5005
    python imu_probe.py --ws ws://127.0.0.1:8080
    python imu_probe.py --tcp 127.0.0.1:8901
    python imu_probe.py --find            (listen on every likely UDP port)

It writes imu_capture.bin (raw packets, length-prefixed) and imu_capture.txt
(the readable report). Send me imu_capture.txt and I can see exactly what
your unit emits.

Nothing here touches the robot, and it can run while the agent is running —
except for the UDP transports, where only one program can hold a port. Stop
the agent first for those.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import imu_link as L


def describe(raw: bytes, out) -> dict:
    """Everything that can be said about one packet."""
    info = L.sniff(raw)
    out(f"  {len(raw)} bytes, looks like {info.get('format')}")
    if info.get("text"):
        out(f"  text: {info.get('preview','')[:300]}")
    else:
        out("  hex : " + " ".join(f"{b:02x}" for b in raw[:96]))
        if len(raw) > 96:
            out("        … " + " ".join(f"{b:02x}" for b in raw[-16:]))
    if info.get("vectors"):
        out("  decoded vectors (magnitude is what identifies each channel):")
        for v in info["vectors"]:
            out(f"    field {v['field']:<8} n={v['n']}  "
                f"{[round(x, 5) for x in v['values']]}   |v| = {v['magnitude']}")
    if info.get("protobuf"):
        out("  protobuf fields:")
        for e in info["protobuf"]:
            out(f"    {e['field']:<10} {e['type']:<7} {e['value']}")
    if info.get("fields"):
        out(f"  recognised: {', '.join(info['fields'])}")
    if info.get("advice"):
        out(f"  note: {info['advice']}")

    # Where does a STRICT parse stop? That is the question behind a packet
    # that half-decodes, and it is not visible from the result alone.
    try:
        n = len(L.protobuf_walk(raw))
        out(f"  strict parse: OK, {n} fields — the whole packet is understood")
    except Exception as e:                          # noqa: BLE001
        out(f"  strict parse: stops with '{e}' — the rest is read leniently")
    return info


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zmq", metavar="ENDPOINT",
                    help="FusionHub External Output, e.g. tcp://*:8901")
    ap.add_argument("--topic", default="", help="ZeroMQ topic filter")
    ap.add_argument("--udp", type=int, metavar="PORT")
    ap.add_argument("--tcp", metavar="HOST:PORT")
    ap.add_argument("--ws", metavar="URL")
    ap.add_argument("--file", metavar="PATH", help="follow a file being written")
    ap.add_argument("--find", action="store_true",
                    help="listen on every likely UDP port and report")
    ap.add_argument("-n", "--packets", type=int, default=40,
                    help="how many packets to capture (default 40)")
    ap.add_argument("-s", "--seconds", type=float, default=15.0)
    a = ap.parse_args()

    report = []

    def out(line=""):
        print(line)
        report.append(str(line))

    out("=" * 70)
    out("SONAIR inertial probe")
    out("=" * 70)

    if a.find:
        out("Listening on every likely UDP port for 8 seconds…")
        res = L.discover_udp(seconds=8.0)
        out(f"  usable ports : {res['usable_ports'] or 'none'}")
        for port, e in (res.get("found") or {}).items():
            out(f"  UDP {port} from {e['from']}: {e['packets']} packets, "
                f"{(e['sniff'] or {}).get('format')}")
        out(f"  {res['advice']}")
        Path("imu_capture.txt").write_text("\n".join(report), encoding="utf-8")
        return 0

    if a.zmq:
        kind, cfg = "zmq-sub", {"endpoint": a.zmq, "topic": a.topic}
    elif a.udp:
        kind, cfg = "udp-listen", {"port": a.udp}
    elif a.tcp:
        host, _, port = a.tcp.partition(":")
        kind, cfg = "tcp-client", {"host": host, "port": int(port or 8901)}
    elif a.ws:
        kind, cfg = "websocket-client", {"url": a.ws}
    elif a.file:
        kind, cfg = "file-tail", {"path": a.file, "from_start": True}
    else:
        ap.print_help()
        return 2

    out(f"transport : {kind}  {cfg}")
    raws: list[bytes] = []
    recs: list[tuple] = []

    link = L.make_link(kind, "probe", gyro_units="auto",
                       on_sample=lambda t, r: recs.append((t, r)), **cfg)
    res = link.start()
    if not res.get("ok"):
        out(f"\nCould not start: {res.get('error')}")
        Path("imu_capture.txt").write_text("\n".join(report), encoding="utf-8")
        return 1

    out(f"capturing up to {a.packets} packets, or {a.seconds:.0f} s…\n")
    t0 = time.monotonic()
    last = b""
    while (time.monotonic() - t0 < a.seconds) and len(raws) < a.packets:
        time.sleep(0.02)
        cur = link.last_raw
        if cur and cur is not last:
            last = cur
            raws.append(bytes(cur))
    link.stop()

    if not raws:
        out("NOTHING ARRIVED.")
        out("  The transport started, so the address was accepted, and no data")
        out("  followed. On the FusionHub side: is the graph running (Activate),")
        out("  and is the source node producing? Its Console shows 'No IMU data'")
        out("  when the sensor has dropped out.")
        Path("imu_capture.txt").write_text("\n".join(report), encoding="utf-8")
        return 1

    out(f"captured {len(raws)} packets\n")
    out("-" * 70)
    out("FIRST PACKET")
    out("-" * 70)
    describe(raws[0], out)

    if len(raws) > 1:
        out()
        out("-" * 70)
        out("A LATER PACKET (to show which numbers move)")
        out("-" * 70)
        describe(raws[len(raws) // 2], out)

    out()
    out("-" * 70)
    out("WHAT THE AGENT WOULD MAKE OF IT")
    out("-" * 70)
    h = link.health()
    out(f"  packets decoded   : {h.get('protobuf_decoded', len(recs))}")
    out(f"  samples delivered : {h['samples']}     rejected: {h['bad']}")
    out(f"  rate              : {h['rate_hz']} Hz")
    out(f"  gyroscope units   : {h.get('gyro_units')} "
        f"({h.get('gyro_units_basis') or 'not yet decided'})")
    mapping = h.get("protobuf_mapping")
    if mapping:
        out(f"  channel mapping   : {mapping}")
        out(f"  timestamp field   : {h.get('protobuf_time_field')} "
            f"in {h.get('protobuf_time_unit')}")
    if h.get("protobuf_partial"):
        out(f"  {h['protobuf_partial']}")
    if recs:
        t, r = recs[-1]
        out("  last reading:")
        for k in ("quat", "gyro", "accel", "mag"):
            if k in r:
                out(f"    {k:<6} {[round(v, 5) for v in r[k]]}")
        out(f"    source time {t}")
    else:
        out("  NO READINGS WERE PRODUCED — packets arrive and decode to nothing.")
        out("  The vectors printed above are what the decoder can see; if they")
        out("  look like real measurements, the field mapping is the problem.")

    # raw packets, length-prefixed, so the exact bytes survive
    with open("imu_capture.bin", "wb") as fh:
        for r in raws:
            fh.write(struct.pack("<I", len(r)))
            fh.write(r)
    Path("imu_capture.txt").write_text("\n".join(report), encoding="utf-8")
    out()
    out(f"Saved {len(raws)} raw packets to imu_capture.bin")
    out("Saved this report to imu_capture.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
