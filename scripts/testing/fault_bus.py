"""
fault_bus.py - UDP link from the fault detector (live_detect_v2_fixed.py) to the ROS2 controller (main.py).

Messages are JSON on 127.0.0.1:14600:
    {"mode": "ADAPTIVE" | "YAW-FREE", "motor": 1..4, "rho": 0..1 or null, "t": wall time}
The sender sends immediately when the mode/motor/rho changes, and repeats every 0.5 s as a heartbeat.
"""
import json
import socket
import time

NOMINAL, ADAPTIVE, YAW_FREE = "NOMINAL", "ADAPTIVE", "YAW-FREE"
PORT = 14600


class FaultSender:
    def __init__(self, host="127.0.0.1", port=PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)
        self.key, self.t_sent = None, 0.0

    def send(self, mode, motor=None, rho=None, force=False):
        key = (mode, motor, None if rho is None else round(float(rho), 2))
        now = time.time()
        if not force and key == self.key and now - self.t_sent < 0.5:
            return
        self.key, self.t_sent = key, now
        msg = dict(mode=mode, motor=None if motor is None else int(motor),
                   rho=None if rho is None else float(rho), t=now)
        self.sock.sendto(json.dumps(msg).encode(), self.addr)


class FaultReceiver:
    def __init__(self, port=PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.state = dict(mode=NOMINAL, motor=None, rho=None)

    def poll(self):
        """Read every pending message; return the newest state (non-blocking)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                self.state = json.loads(data.decode())
            except ValueError:
                pass
        return self.state
