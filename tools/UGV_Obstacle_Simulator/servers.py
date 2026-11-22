"""TCP services on 127.0.0.1 that stand in for the weeder: LiDAR, GNSS, Vehicle Control."""
from __future__ import annotations

import copy
import os
import socket
import threading
import time

try:                                   # imported as part of a package (tests)
    from . import sim_core as sc
    from .protocols import ScanRequestParser, VCParser, build_scan_telegram
except ImportError:                    # run as plain scripts from this folder
    import sim_core as sc
    from protocols import ScanRequestParser, VCParser, build_scan_telegram

WDOG_TIMEOUT = 0.3          # s without a wdog toggle -> motion inhibited
GNSS_PERIOD = 0.4           # s between GNSS packets (matches the original simulator)
GNSS_PACKET = 112


class PortInUseError(Exception):
    def __init__(self, port: int):
        super().__init__(f"Port {port} is already in use")
        self.port = port


class SimState:
    def __init__(self):
        self.lock = threading.RLock()
        v, boxes = sc.PRESETS["1"]()
        self.vehicle = v
        self.world = sc.World(walls=True)
        self.world.boxes = boxes
        self.cmd_steer = 0.0
        self.cmd_speed = 0.0
        self.last_wdog: int | None = None
        self.last_wdog_toggle_time = -1e9
        self.vc_clients = 0
        self.lidar_clients = 0
        self.gnss_clients = 0
        self.collision = False
        self.last_scan: list[int] = [0] * sc.N_RAYS
        self.scan_count = 0

    @property
    def vc_connected(self) -> bool:
        return self.vc_clients > 0

    def wdog_ok(self, now: float) -> bool:
        return self.vc_connected and (now - self.last_wdog_toggle_time) <= WDOG_TIMEOUT

    def snapshot(self):
        with self.lock:
            return copy.copy(self.vehicle), copy.deepcopy(self.world)


class ServerHandle:
    def __init__(self):
        self.threads: list[threading.Thread] = []
        self.listeners: list[socket.socket] = []
        self.clients: set[socket.socket] = set()
        self.stopping = threading.Event()
        self._lock = threading.Lock()

    def track(self, c: socket.socket):
        with self._lock:
            self.clients.add(c)

    def untrack(self, c: socket.socket):
        with self._lock:
            self.clients.discard(c)

    def stop(self):
        self.stopping.set()
        for s in self.listeners:
            try:
                s.close()
            except OSError:
                pass
        with self._lock:
            for c in list(self.clients):
                try:
                    c.close()
                except OSError:
                    pass
        for t in self.threads:
            t.join(timeout=1.0)


# ---- helpers ----------------------------------------------------------------
def _bind(port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    try:
        s.bind(("127.0.0.1", port))
    except OSError:
        s.close()
        raise PortInUseError(port)
    s.listen(4)
    s.settimeout(0.2)
    return s


def _authenticate(c: socket.socket) -> bytes | None:
    """Read '<zID>\\n', reply 'OK\\n'. Returns any bytes received after the newline."""
    buf = b""
    while b"\n" not in buf:
        d = c.recv(1024)
        if not d:
            return None
        buf += d
        if len(buf) > 1024:
            return None
    c.sendall(b"OK\n")
    return buf.split(b"\n", 1)[1]


def _serve(handle: ServerHandle, listener: socket.socket, client_fn, name: str):
    while not handle.stopping.is_set():
        try:
            c, _ = listener.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        c.settimeout(None)
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        handle.track(c)

        def run(c=c):
            try:
                client_fn(c)
            except OSError:
                pass  # client dropped; never take the simulator down
            finally:
                handle.untrack(c)
                try:
                    c.close()
                except OSError:
                    pass

        threading.Thread(target=run, name=f"{name}-client", daemon=True).start()


# ---- services ---------------------------------------------------------------
def _lidar_client(state: SimState):
    def fn(c: socket.socket):
        rest = _authenticate(c)
        if rest is None:
            return
        with state.lock:
            state.lidar_clients += 1
        try:
            parser = ScanRequestParser()
            pending = parser.feed(rest)
            while True:
                for _ in range(pending):
                    v, w = state.snapshot()
                    ranges = sc.scan(v, w)
                    with state.lock:
                        state.scan_count += 1
                        state.last_scan = ranges
                        n = state.scan_count
                    c.sendall(build_scan_telegram(ranges, n))
                d = c.recv(4096)
                if not d:
                    return
                pending = parser.feed(d)
        finally:
            with state.lock:
                state.lidar_clients -= 1
    return fn


def _gnss_client(state: SimState, packets: list[bytes], stopping: threading.Event):
    def fn(c: socket.socket):
        with state.lock:
            state.gnss_clients += 1
        try:
            i = 0
            while not stopping.is_set():
                c.sendall(packets[i % len(packets)])
                i += 1
                time.sleep(GNSS_PERIOD)
        finally:
            with state.lock:
                state.gnss_clients -= 1
    return fn


def _vc_client(state: SimState):
    def apply(frames):
        now = time.monotonic()
        with state.lock:
            for steer, speed, wdog in frames:
                state.cmd_steer, state.cmd_speed = steer, speed
                if wdog != state.last_wdog:
                    state.last_wdog = wdog
                    state.last_wdog_toggle_time = now

    def fn(c: socket.socket):
        rest = _authenticate(c)
        if rest is None:
            return
        with state.lock:
            state.vc_clients += 1
        try:
            parser = VCParser()
            apply(parser.feed(rest))
            while True:
                d = c.recv(4096)
                if not d:
                    return
                apply(parser.feed(d))
        finally:
            with state.lock:
                state.vc_clients -= 1
                if state.vc_clients == 0:
                    state.cmd_speed = 0.0
                    state.cmd_steer = 0.0
                    state.last_wdog = None
                    state.last_wdog_toggle_time = -1e9
    return fn


def load_gnss_packets(path: str) -> list[bytes]:
    if not os.path.isfile(path):
        return []
    with open(path, "rb") as f:
        data = f.read()
    return [data[i:i + GNSS_PACKET] for i in range(0, len(data) - GNSS_PACKET + 1, GNSS_PACKET)]


def start_servers(state: SimState, gps_path: str, ports=(23000, 24000, 25000)) -> ServerHandle:
    """Bind all three ports (all-or-nothing) and start serving. Raises PortInUseError."""
    handle = ServerHandle()
    try:
        for p in ports:
            handle.listeners.append(_bind(p))
    except PortInUseError:
        for s in handle.listeners:
            s.close()
        raise
    packets = load_gnss_packets(gps_path)
    lidar_l, gnss_l, vc_l = handle.listeners
    services = [(lidar_l, _lidar_client(state), "lidar"), (vc_l, _vc_client(state), "vc")]
    if packets:
        services.append((gnss_l, _gnss_client(state, packets, handle.stopping), "gnss"))
    else:
        print(f"WARNING: GNSS file not found at {gps_path}; GNSS port accepts but sends nothing")
        services.append((gnss_l, lambda c: c.recv(1), "gnss"))
    for listener, fn, name in services:
        t = threading.Thread(target=_serve, args=(handle, listener, fn, name), name=name, daemon=True)
        t.start()
        handle.threads.append(t)
    return handle
