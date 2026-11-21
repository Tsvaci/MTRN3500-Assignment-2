"""Drive the exemplar (scripted-controller test exe) against the UGV Obstacle Simulator, headless.
Runs the simulator's own servers + physics (same as its GUI tick) and logs every scenario.
Usage: python run_obstacle_sim_scenarios.py <sim_dir> <exe> <repo> <out_dir>
"""
import json, math, os, subprocess, sys, threading, time

SIM, EXE, REPO, OUT = sys.argv[1:5]
sys.path.insert(0, SIM)
import sim_core as sc
from servers import SimState, start_servers

os.makedirs(OUT, exist_ok=True)
INPUT = os.path.join(OUT, "input.txt")
def ctl(rt=0.0, lt=0.0, rx=0.0, b=0, conn=1):
    with open(INPUT + ".tmp", "w") as f:
        f.write(f"{conn} {rt} {lt} {rx} {b}")
    os.replace(INPUT + ".tmp", INPUT)
ctl()

# ---- geometry: gap between the vehicle body and the nearest solid ----------
def seg_pt(px, py, a, b):
    (x1, y1), (x2, y2) = a, b
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - x1 - t * dx, py - y1 - t * dy)

def poly_gap(P, Q):
    if sc._polys_overlap(P, Q):
        return 0.0
    d = 1e9
    for A, B in ((P, Q), (Q, P)):
        for p in A:
            for i in range(len(B)):
                d = min(d, seg_pt(p[0], p[1], B[i], B[(i + 1) % len(B)]))
    return d

def gap(v, w, boxes_only=False):
    body = v.corners()
    solids = w.boxes if boxes_only else w.solids()
    return min([poly_gap(body, b.corners()) for b in solids] or [99.0])

def nearest_ahead(v, w):
    r = sc.scan(v, w, noise=False)
    best = None
    for i, mm in enumerate(r):
        if mm == 0:
            continue
        d = mm / 1000
        lat, fwd = d * math.cos(math.radians(i * .5)), d * math.sin(math.radians(i * .5))
        if abs(lat) <= sc.BODY_WID / 2 and fwd > 0 and (best is None or fwd < best):
            best = fwd
    return best

# ---- simulator ----------------------------------------------------------------
state = SimState()
handle = start_servers(state, os.path.join(SIM, "GPSData.dat"))
log, scen = [], {"name": "startup"}
stop = threading.Event()

def physics():
    last, nlog = time.monotonic(), 0
    while not stop.is_set():
        now = time.monotonic()
        dt = min(0.05, now - last); last = now
        with state.lock:
            st = state
            spd = st.cmd_speed if st.wdog_ok(now) else 0.0
            if sc.step(st.vehicle, st.world, st.cmd_steer, spd, dt):
                st.collision = True
            nlog += 1
            if nlog % 3 == 0:
                v = st.vehicle
                log.append(dict(t=now, s=scen["name"], x=v.x, y=v.y, h=math.degrees(v.heading), v=v.speed,
                                cs=st.cmd_steer, cv=st.cmd_speed, wd=st.wdog_ok(now), col=st.collision,
                                gap=gap(v, st.world), gapbox=gap(v, st.world, True),
                                ahead=nearest_ahead(v, st.world)))
        time.sleep(0.02)
threading.Thread(target=physics, daemon=True).start()

disp = subprocess.Popen([sys.executable, os.path.join(REPO, "tools", "mock_display.py"), os.path.join(OUT, "display.json")],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
env = dict(os.environ, MTRN_TEST_INPUT=INPUT)
exe = subprocess.Popen([EXE], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE,
                       env=env, cwd=os.path.dirname(EXE), text=True, errors="replace")
console = []
def drain():
    for line in exe.stdout:
        console.append((time.monotonic(), scen["name"], line.rstrip()))
threading.Thread(target=drain, daemon=True).start()

t0 = time.monotonic()
while not any("all threads running" in l for _, _, l in console):
    if time.monotonic() - t0 > 30 or exe.poll() is not None:
        print("\n".join(l for _, _, l in console)); sys.exit("exe did not start")
    time.sleep(0.2)
time.sleep(1.5)

def setup(preset=None, boxes=None, walls=True, pose=None):
    with state.lock:
        if preset:
            v, b = sc.PRESETS[preset]()
        else:
            v, b = sc._start(), []
        state.vehicle = pose or v
        state.world.boxes = boxes if boxes is not None else b
        state.world.walls = walls
        state.collision = False

def run(name, dur, preset=None, boxes=None, walls=True, pose=None, inputs=(), events=()):
    """inputs: list of (t, kwargs for ctl). events: list of (t, fn)."""
    ctl(); time.sleep(0.8)
    scen["name"] = "idle"
    setup(preset, boxes, walls, pose)
    time.sleep(0.6)                      # let crash avoidance see the new scene
    scen["name"] = name
    ts = time.monotonic()
    pend = sorted([(t, "i", k) for t, k in inputs] + [(t, "e", f) for t, f in events], key=lambda z: z[0])
    while time.monotonic() - ts < dur:
        el = time.monotonic() - ts
        while pend and pend[0][0] <= el:
            _, kind, x = pend.pop(0)
            ctl(**x) if kind == "i" else x()
        time.sleep(0.01)
    scen["name"] = "idle"; ctl()
    print("done", name, flush=True)

FULL = dict(rt=1.0)
def add_box_ahead(dist, w=0.6, h=0.3):
    def f():
        with state.lock:
            v = state.vehicle
            ox, oy, hd = v.lidar_pose()
            cx, cy = ox + (dist + h / 2) * math.cos(hd), oy + (dist + h / 2) * math.sin(hd)
            state.world.boxes.append(sc.Box(cx, cy, w, h))   # axis-aligned; vehicle faces +y
    return f

L0 = sc.LIDAR_Y0; X0 = sc.START_X
run("A1 wall 1.5 m ahead, full forward", 6, "1", inputs=[(0, FULL)])
# A2 needs the A1 end pose: run reverse right after A1 without resetting
scen["name"] = "A2 reverse away from the wall"; ctl(lt=1.0); time.sleep(2.0); ctl(); scen["name"] = "idle"; time.sleep(0.5)
run("A3 wall ahead, half throttle", 7, "1", inputs=[(0, dict(rt=0.5))])
run("B1 box ahead-left, drive straight", 5, "2", inputs=[(0, FULL)])
run("B2 box ahead-left, hard LEFT into it", 5, "2", inputs=[(0, dict(rt=1.0, rx=-1.0))])
run("B3 box ahead-left, hard RIGHT away", 4, "2", inputs=[(0, dict(rt=1.0, rx=1.0))])
run("C1 box ahead-right, drive straight", 5, "3", inputs=[(0, FULL)])
run("C2 box ahead-right, hard RIGHT into it", 5, "3", inputs=[(0, dict(rt=1.0, rx=1.0))])
run("C3 box ahead-right, hard LEFT away", 4, "3", inputs=[(0, dict(rt=1.0, rx=-1.0))])
run("D1 1 m gap 2 m ahead, straight then far wall", 12, "4", inputs=[(0, FULL)])
run("E1 walls off (no-echo zeros), open space", 4, None, boxes=[], walls=False, inputs=[(0, FULL)])
run("F1 box appears 1.5 m ahead at full speed", 5, None, boxes=[], inputs=[(0, FULL)], events=[(2.0, add_box_ahead(1.5))])
run("F2 box appears 0.6 m ahead at full speed", 5, None, boxes=[], inputs=[(0, FULL)], events=[(2.0, add_box_ahead(0.6))])
run("G1 wall ahead, 20 deg left (oblique approach)", 7, "1", inputs=[(0, dict(rt=1.0, rx=-0.5))])
# H1: box beside the body (behind the LiDAR, which can't see it), turn into it
side = sc.Box(X0 - 0.35 - 0.10 - 0.2, sc.START_Y + 0.4, 0.4, 0.4)   # 10 cm off the left side, mid-body
run("H1 box beside the body (LiDAR blind spot), hard left", 4, None, boxes=[side], inputs=[(0, dict(rt=1.0, rx=-1.0))])

ctl(b=1)
try:
    exe.wait(15)
except subprocess.TimeoutExpired:
    exe.kill()
stop.set(); handle.stop(); disp.kill()
json.dump(dict(log=log, console=console, exit=exe.returncode,
               consts=dict(BODY_LEN=sc.BODY_LEN, BODY_WID=sc.BODY_WID, WHEELBASE=sc.WHEELBASE, LIDAR_Y0=L0)),
          open(os.path.join(OUT, "run.json"), "w"))
print("exit", exe.returncode)
