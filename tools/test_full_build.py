"""
Full-build test against the stand-in servers (no simulator, no MATLAB, no Xbox controller).
Uses the test-only exe from build_test_exe.py, whose Controller reads its inputs from a file.

Run 1 (main run): GNSS decode + CRC, display data in mm, driving, crash-avoidance scenarios,
                  watchdog toggling, controller-disconnect, shutdown with the B button.
Run 2: display server lost -> critical shutdown, exit code 2.
Run 3: GNSS connection dropped -> GNSS (non-critical) is restarted and keeps decoding.
Run 4: 'q' key -> routine shutdown, exit code 0 (the key is typed into the exe's own console).

Usage (repo root):  python tools/test_full_build.py <test exe>
"""
import ctypes, json, math, os, socket, struct, subprocess, sys, tempfile, threading, time
from ctypes import wintypes

HERE = os.path.dirname(os.path.abspath(__file__))
EXE = sys.argv[1]
GPS_FILE = os.path.join(HERE, "..", "..", "WeederSimulator", "GPSData.dat")
TMP = tempfile.mkdtemp(prefix="mtrn_fb_")
SCENE = os.path.join(TMP, "scene.json")
VC_LOG = os.path.join(TMP, "vc.jsonl")
DISPLAY_OUT = os.path.join(TMP, "display.json")
INPUT = os.path.join(TMP, "input.txt")

results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {detail}", flush=True)


for port in (23000, 24000, 25000, 28000):
    with socket.socket() as probe:
        if probe.connect_ex(("127.0.0.1", port)) == 0:
            sys.exit(f"Port {port} is in use (simulator or MATLAB display running?). Close it and re-run.")


def write(path, text):
    with open(path + ".tmp", "w") as f:
        f.write(text)
    os.replace(path + ".tmp", path)


def scene(*obstacles):
    write(SCENE, json.dumps({"obstacles": [{"x": x, "y": y, "r": r} for x, y, r in obstacles]}))


def controller(connected=1, rt=0.0, lt=0.0, rx=0.0, b=0):
    write(INPUT, f"{connected} {rt} {lt} {rx} {b}")


def start_servers(gnss_drop_after=0):
    open(VC_LOG, "w").close()
    args = [sys.executable, "-I", os.path.join(HERE, "mock_weeder.py"), SCENE, VC_LOG, GPS_FILE]
    if gnss_drop_after:
        args.append(str(gnss_drop_after))
    weeder = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    display = subprocess.Popen([sys.executable, "-I", os.path.join(HERE, "mock_display.py"), DISPLAY_OUT],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    return weeder, display


def start_exe():
    env = dict(os.environ, MTRN_TEST_INPUT=INPUT)
    p = subprocess.Popen([EXE], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE,
                         text=True, env=env)
    lines = []

    def drain():                       # keep reading, or the exe blocks on a full pipe
        for line in p.stdout:
            lines.append(line.rstrip("\n"))
    threading.Thread(target=drain, daemon=True).start()
    return p, lines


def vc_since(t0):
    out = []
    with open(VC_LOG) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if rec["t"] >= t0:
                out.append(rec)
    return out


def drive(desc, obstacles, settle=0.6, **ctrl):
    """Set the scene and controller, wait, and return the last command the vehicle received."""
    scene(*obstacles)
    controller(**ctrl)
    time.sleep(settle)
    t0 = time.time()
    time.sleep(0.4)
    cmds = vc_since(t0)
    if not cmds:
        check(desc, False, "no VC commands received")
        return None
    return cmds[-1]


def expected_straight_speed(surface_m, asked=1.0):
    usable = surface_m - asked * 0.10 - 0.30
    return 0.0 if usable <= 0 else min(1.0, math.sqrt(usable / 0.70))


def kill(*procs):
    for pr in procs:
        try:
            pr.kill()
        except Exception:
            pass


# ------------------------------------------------------------------ Run 1
print("\n=== Run 1: main run")
scene()
controller()
weeder, display = start_servers()
p, lines = start_exe()
time.sleep(4.0)

# Driving and crash avoidance. Obstacles are circles of radius 0.1 m; "surface" = nearest face.
c = drive("clear road", [], rt=1.0)
check("clear road, full throttle -> 1.0 m/s, straight", c and abs(c["speed"] - 1.0) < 0.01 and c["steer"] == 0, str(c))

for surface in (2.0, 0.8, 0.5, 0.35):
    c = drive(f"ahead {surface}", [(surface + 0.1, 0.0, 0.1)], rt=1.0)
    exp = expected_straight_speed(surface)
    check(f"obstacle {surface} m ahead -> speed {exp:.2f}", c and abs(c["speed"] - exp) < 0.03, str(c))

c = drive("reverse", [(0.45, 0.0, 0.1)], lt=1.0)
check("obstacle 0.35 m ahead, reversing -> -1.0 m/s allowed", c and abs(c["speed"] + 1.0) < 0.01, str(c))

c = drive("half throttle", [], rt=0.5)
check("half throttle -> 0.5 m/s", c and abs(c["speed"] - 0.5) < 0.01, str(c))

c = drive("deadzone", [], rt=0.05, rx=0.05)
check("tiny trigger/stick inside deadzone -> 0, 0", c and c["speed"] == 0 and c["steer"] == 0, str(c))

c = drive("left clear", [], rt=1.0, rx=-1.0)
check("stick full left, clear -> steer +40 (left)", c and abs(c["steer"] - 40) < 0.01 and c["speed"] == 1.0, str(c))

side = [(0.5, 0.75, 0.1)]       # beside the path on the left, not in the straight corridor
c = drive("straight past side obstacle", side, rt=1.0)
check("obstacle front-left, driving straight -> unaffected", c and c["speed"] == 1.0 and c["steer"] == 0, str(c))
c = drive("turn into side obstacle", side, rt=1.0, rx=-1.0)
check("obstacle front-left, turning left -> turn eased (0 < steer < 40), still moving",
      c and 0 <= c["steer"] < 40 and c["speed"] > 0.5, str(c))
c = drive("turn away from side obstacle", side, rt=1.0, rx=1.0)
check("obstacle front-left, turning right -> unaffected (-40, 1.0)", c and c["steer"] == -40 and c["speed"] == 1.0, str(c))

right = [(0.5, -0.75, 0.1)]
c = drive("turn into right obstacle", right, rt=1.0, rx=1.0)
check("obstacle front-right, turning right -> turn eased (-40 < steer <= 0)", c and -40 < c["steer"] <= 0 and c["speed"] > 0.5, str(c))

c = drive("beside left", [(0.1, 0.55, 0.1)], rt=1.0, rx=-1.0)
check("obstacle right beside the front-left corner, turning left -> eased or slowed",
      c and (c["steer"] < 40 or c["speed"] < 0.5), str(c))

c = drive("blocked ahead, steer left", [(0.6, 0.0, 0.1)], rt=1.0, rx=-1.0)
check("blocked 0.5 m ahead, turning left -> steering free, speed cut", c and c["steer"] == 40 and c["speed"] < 0.3, str(c))

c = drive("one stray point", [(0.4, 0.0, 0.002)], rt=1.0)
check("single stray reading 0.4 m ahead -> ignored", c and c["speed"] == 1.0, str(c))

c = drive("thin pole", [(0.51, 0.0, 0.01)], rt=1.0)
check("2 cm pole 0.5 m ahead -> slowed", c and c["speed"] < 0.5, str(c))

c = drive("disconnected", [], connected=0, rt=1.0, rx=-1.0)
check("controller disconnected -> 0, 0", c and c["speed"] == 0 and c["steer"] == 0, str(c))
check("disconnect message printed", any("DISCONNECTED" in l for l in lines))

# Watchdog
cmds = vc_since(time.time() - 1.0)
wd = [r["wdog"] for r in cmds]
check("watchdog alternates 0/1 on every message", len(wd) > 10 and all(a != b for a, b in zip(wd, wd[1:])),
      f"{len(wd)} msgs in 1 s")

# Display
scene((2.1, 0.0, 0.1))
time.sleep(1.0)
try:
    d = json.load(open(DISPLAY_OUT))
    x180, y180 = d["x"][180], d["y"][180]
    x0, y0 = d["x"][0], d["y"][0]
    check("display receives 361+361 doubles", d["frames"] > 5, f"{d['frames']} frames")
    check("display in mm, Y = forward: obstacle 2.0 m ahead -> X[180] ~ 0, Y[180] ~ 2000",
          abs(y180 - 2000) < 5 and abs(x180) < 5, f"X[180]={x180:.1f} Y[180]={y180:.1f}")
    check("display X = right: beam 0 (vehicle's right) at 8 m -> X[0] ~ +8000, Y[0] ~ 0",
          abs(x0 - 8000) < 5 and abs(y0) < 5, f"X[0]={x0:.1f} Y[0]={y0:.1f}")
except Exception as e:
    check("display output readable", False, str(e))

# GNSS
data = open(GPS_FILE, "rb").read()
expected = [struct.unpack_from("<3d", data, i + 28 + 16) for i in range(0, 112 * 30, 112)]
ok_lines = [l for l in lines if l.startswith("GNSS: Northing")]
bad_lines = [l for l in lines if "CRC MISMATCH" in l]
check("GNSS records decoded with CRC OK", len(ok_lines) > 20, f"{len(ok_lines)} OK lines")
check("GNSS corrupted records rejected (every 7th)", len(bad_lines) > 2, f"{len(bad_lines)} mismatches")
if ok_lines:
    print("   first GNSS lines:", *ok_lines[:2], sep="\n     ")
    n1 = float(ok_lines[1].split()[2])
    check("GNSS values match GPSData.dat record 2", abs(n1 - expected[1][0]) < 0.001,
          f"printed {n1:.3f} vs file {expected[1][0]:.3f}")
if bad_lines:
    print("   ", bad_lines[0])

# Shutdown by B button
controller(b=1)
try:
    rc = p.wait(timeout=10)
except subprocess.TimeoutExpired:
    rc = None
check("B button -> all threads stop, exit code 0", rc == 0, f"exit {rc}")
check("B press reported", any("B pressed" in l for l in lines))
stopped = [l for l in lines if "thread stopped." in l]
check("TMM confirms 6 threads stopped", len(stopped) == 6, f"{len(stopped)}")
last_cmd = vc_since(0)[-1] if vc_since(0) else None
check("last command sent before closing is a stop", last_cmd and last_cmd["speed"] == 0, str(last_cmd))
kill(weeder, display)
run1_lines = lines

# ------------------------------------------------------------------ Run 2
print("\n=== Run 2: display lost")
time.sleep(1.0)
controller()
weeder, display = start_servers()
p, lines = start_exe()
time.sleep(4.0)
kill(display)
try:
    rc = p.wait(timeout=15)
except subprocess.TimeoutExpired:
    rc = None
check("display server killed -> full shutdown, exit code 2", rc == 2, f"exit {rc}")
check("TMM names Display as the critical failure", any("CRITICAL module Display" in l for l in lines))
kill(weeder, p)

# ------------------------------------------------------------------ Run 3
print("\n=== Run 3: GNSS connection dropped every 15 records")
time.sleep(1.0)
weeder, display = start_servers(gnss_drop_after=15)
p, lines = start_exe()
time.sleep(12.0)
check("GNSS restarted after its connection dropped", sum("GNSS: thread started." in l for l in lines) >= 2,
      f"{sum('GNSS: thread started.' in l for l in lines)} GNSS starts")
check("program kept running (no critical shutdown)", p.poll() is None)

# ------------------------------------------------------------------ Run 4 ('q' key)
print("\n=== Run 4: q key")
kill(p, weeder, display)
time.sleep(1.0)
k32 = ctypes.windll.kernel32


class KEY_EVENT_RECORD(ctypes.Structure):
    _fields_ = [("bKeyDown", wintypes.BOOL), ("wRepeatCount", wintypes.WORD),
                ("wVirtualKeyCode", wintypes.WORD), ("wVirtualScanCode", wintypes.WORD),
                ("UnicodeChar", wintypes.WCHAR), ("dwControlKeyState", wintypes.DWORD)]


class INPUT_RECORD(ctypes.Structure):
    _fields_ = [("EventType", wintypes.WORD), ("KeyEvent", KEY_EVENT_RECORD)]


def inject_key(pid, ch):
    """Types a key into another process's console (the exe runs in its own console window)."""
    k32.FreeConsole()
    if not k32.AttachConsole(pid):
        return False
    h = k32.CreateFileW("CONIN$", 0x80000000 | 0x40000000, 3, None, 3, 0, None)
    recs = (INPUT_RECORD * 2)()
    for i, down in enumerate((1, 0)):
        recs[i].EventType = 1
        recs[i].KeyEvent.bKeyDown = down
        recs[i].KeyEvent.wRepeatCount = 1
        recs[i].KeyEvent.wVirtualKeyCode = ord(ch.upper())
        recs[i].KeyEvent.UnicodeChar = ch
    written = wintypes.DWORD(0)
    ok = k32.WriteConsoleInputW(h, recs, 2, ctypes.byref(written))
    k32.CloseHandle(h)
    k32.FreeConsole()
    return bool(ok)


controller()
weeder, display = start_servers()
out_path = os.path.join(TMP, "run4_output.txt")
with open(out_path, "w") as out:
    p = subprocess.Popen([EXE], stdout=out, stderr=subprocess.STDOUT, env=dict(os.environ, MTRN_TEST_INPUT=INPUT),
                         creationflags=subprocess.CREATE_NEW_CONSOLE)
    time.sleep(4.0)
    t_key = time.time()
    injected = inject_key(p.pid, "q")
    try:
        rc = p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        rc = None
        p.kill()
    t_exit = time.time() - t_key
text = open(out_path, encoding="utf-8", errors="replace").read()
check("q key -> routine shutdown, exit code 0", injected and rc == 0 and "'q' pressed" in text,
      f"exit {rc} after {t_exit:.1f} s")
check("q key: TMM confirms 6 threads stopped", text.count("thread stopped.") == 6)
kill(weeder, display)

print()
passed = sum(ok for _, ok in results)
print(f"{passed}/{len(results)} checks passed")
with open(os.path.join(TMP, "run1_output.txt"), "w") as f:
    f.write("\n".join(run1_lines))
print("run 1 console output saved to", os.path.join(TMP, "run1_output.txt"))
sys.exit(0 if passed == len(results) else 1)
