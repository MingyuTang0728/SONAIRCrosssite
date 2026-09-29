"""
A stand-in UR controller for the RTDE port, faithful in the one way that
matters here: it streams at the configured rate and CLOSES A CLIENT THAT DOES
NOT KEEP UP, the way the real controller does when its per-client output
buffer fills. Used to prove that the agent keeps the link through a stall of
its own process.
"""
import socket, struct, threading, time

TYPES = {"DOUBLE": ("d", 8), "UINT32": ("I", 4), "UINT64": ("Q", 8),
         "INT32": ("i", 4), "UINT8": ("B", 1), "BOOL": ("?", 1),
         "VECTOR3D": ("3d", 24), "VECTOR6D": ("6d", 48),
         "VECTOR6INT32": ("6i", 24), "VECTOR6UINT32": ("6I", 24)}


def known_type(name):
    if name in ("robot_mode", "safety_mode", "safety_status", "tool_output_voltage"):
        return "INT32"
    if name in ("runtime_state", "robot_status_bits", "safety_status_bits"):
        return "UINT32"
    if name.endswith("_bits"):
        return "UINT64"
    if name == "joint_mode":
        return "VECTOR6INT32"
    if name == "actual_tool_accelerometer":
        return "VECTOR3D"
    if name.startswith(("actual_q", "actual_qd", "actual_current", "joint_",
                        "target_", "actual_TCP", "actual_joint_voltage")):
        return "VECTOR6D"
    return "DOUBLE"


class FakeRTDE:
    def __init__(self, port=0, max_backlog_s=0.4, hz=125.0):
        self.srv = socket.socket(); self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port)); self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.max_backlog_s, self.hz = max_backlog_s, hz
        self.closed_slow = 0
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            c, _ = self.srv.accept()
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _pkt(self, cmd, body=b""):
        return struct.pack(">HB", 3 + len(body), cmd) + body

    def _recv(self, c):
        h = b""
        while len(h) < 3:
            h += c.recv(3 - len(h))
        n, cmd = struct.unpack(">HB", h)
        b = b""
        while len(b) < n - 3:
            b += c.recv(n - 3 - len(b))
        return cmd, b

    def _serve(self, c):
        recipe = []
        try:
            while True:
                cmd, b = self._recv(c)
                if cmd == 86:
                    c.sendall(self._pkt(86, b"\x01"))
                elif cmd == 118:
                    c.sendall(self._pkt(118, struct.pack(">IIII", 5, 11, 1, 0)))
                elif cmd == 79:
                    names = b[8:].decode().split(",")
                    recipe = [(n, known_type(n)) for n in names]
                    c.sendall(self._pkt(79, b"\x01" + ",".join(t for _, t in recipe).encode()))
                elif cmd == 83:
                    c.sendall(self._pkt(83, b"\x01"))
                    break
        except Exception:
            c.close(); return
        fmt = ">" + "".join(TYPES[t][0] for _, t in recipe)
        vals = []
        # FAKE_Q=q1,..,q6 holds the arm at a real pose (a powered, running
        # robot), for driving the console against it; unset, everything is 0.
        import os
        q = [float(v) for v in os.environ.get("FAKE_Q", "").split(",") if v]
        tcp = None
        if len(q) == 6:
            try:
                import sys
                sys.path.insert(0, str(__import__("pathlib").Path(__file__)
                                       .resolve().parent.parent))
                import ur_kin
                tcp = ur_kin.fk_pose(q)
            except Exception:       # noqa: BLE001
                tcp = None
        for name, t in recipe:
            n = {"3d": 3, "6d": 6, "6i": 6, "6I": 6}.get(TYPES[t][0], 1)
            v = [0] * n
            if len(q) == 6 and name in ("actual_q", "target_q"):
                v = list(q)
            elif tcp and name in ("actual_TCP_pose", "target_TCP_pose"):
                v = list(tcp)
            elif len(q) == 6 and name == "robot_mode":
                v = [7]
            elif len(q) == 6 and name == "safety_mode":
                v = [1]
            elif len(q) == 6 and name == "speed_scaling":
                v = [1.0]
            vals += v
        # A small kernel buffer and a hard cap on what it will queue for one
        # client: exactly the "slow client" the real controller drops.
        c.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        c.setblocking(False)
        pending, period, t0, i = b"", 1.0 / self.hz, time.perf_counter(), 0
        import os
        drop_after = float(os.environ.get("FAKE_DROP_FIRST_AFTER", "0") or 0)
        self.served = getattr(self, "served", 0) + 1
        first = self.served == 1
        cap = int(self.max_backlog_s * self.hz * (len(struct.pack(fmt, *vals)) + 4))
        while True:
            i += 1
            vals[0] = i * period           # 'timestamp'
            pending += self._pkt(85, b"\x01" + struct.pack(fmt, *vals))
            try:
                sent = c.send(pending)
                pending = pending[sent:]
            except BlockingIOError:
                pass
            except OSError:
                return
            if first and drop_after and time.perf_counter() - t0 > drop_after:
                c.close()               # a one-off drop, as a controller does
                return
            if len(pending) > cap:
                self.closed_slow += 1
                c.close()
                return
            nxt = t0 + i * period
            time.sleep(max(0.0, nxt - time.perf_counter()))


if __name__ == "__main__":
    import sys
    s = FakeRTDE(port=int(sys.argv[1]) if len(sys.argv) > 1 else 0)
    print(s.port, flush=True)
    last = -1
    while True:
        time.sleep(0.2)
        if s.closed_slow != last:
            last = s.closed_slow
            print(f"CLOSED_SLOW {last}", flush=True)
