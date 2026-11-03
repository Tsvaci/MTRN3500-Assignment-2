"""
Moving-obstacle check (no simulator involved).
Starts mock_lidar.py --moving, where a post 1.0 m away sweeps from the vehicle's right (10 deg)
to its left (170 deg) and back, 2 degrees per scan. Runs the exe for a few seconds and, for every
scan it prints, finds the nearest point and compares it with where the post really was.

Passes if every printed scan shows the post where it should be, which proves that
  - the printout is live (it follows the moving post), and
  - left/right is right: post on the vehicle's right -> y negative, on its left -> y positive.
Run from the repo root after building:  python tools/test_moving_obstacle.py
"""
import subprocess, time, sys, re, threading, os, math, socket

EXE = r"Executables\MTRN3500 Assignment 2.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
MOCK = os.path.join(HERE, "mock_lidar.py")
RUN_SECONDS = 8
TOLERANCE_M = 0.03

def moving_post_deg(k):
    """Same formula as mock_lidar.py (importing that file would start its server)."""
    phase = (2 * k) % 320
    return 10.0 + phase if phase <= 160 else 330.0 - phase

with socket.socket() as probe:
    if probe.connect_ex(("127.0.0.1", 23000)) == 0:
        sys.exit("Port 23000 is already in use (is the simulator running?). Close it and re-run.")

srv = subprocess.Popen([sys.executable, MOCK, "--moving"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.5)
p = subprocess.Popen([EXE], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, stdin=subprocess.DEVNULL)
lines = []
def drain():
    for line in p.stdout:          # must keep reading, or the exe blocks on a full pipe
        lines.append(line.rstrip("\n"))
threading.Thread(target=drain, daemon=True).start()
time.sleep(RUN_SECONDS)
srv.kill()                         # LiDAR lost -> exe shuts itself down
p.wait(timeout=20)

# Split the output into printed scans: {scan number: [(x, y) * 361]}
scans, current = {}, None
for l in lines:
    m = re.match(r"LiDAR scan #(\d+)", l)
    if m:
        current = []
        scans[int(m.group(1))] = current
    elif current is not None:
        for pm in re.finditer(r"(\d+): \(\s*(-?[\d.]+),\s*(-?[\d.]+)\)", l):
            current.append((float(pm.group(2)), float(pm.group(3))))

failures = 0
print(f"{'scan':>5} {'post at':>8} {'expected (x, y)':>18} {'printed nearest (x, y)':>24}  side")
for k, pts in sorted(scans.items()):
    if len(pts) != 361:
        print(f"{k:>5}  only {len(pts)} points printed"); failures += 1; continue
    ang = moving_post_deg(k)
    ex, ey = math.sin(math.radians(ang)), -math.cos(math.radians(ang))
    nonzero = [(math.hypot(x, y), x, y) for x, y in pts if (x, y) != (0.0, 0.0)]
    _, nx, ny = min(nonzero)
    # The nearest point is one edge of the 6-degree-wide post, so compare it with the post's span.
    span = [(math.sin(math.radians(a)), -math.cos(math.radians(a)))
            for a in (ang - 3.0, ang - 1.5, ang, ang + 1.5, ang + 3.0)]
    ok = min(math.hypot(nx - sx, ny - sy) for sx, sy in span) <= TOLERANCE_M
    side = "right" if ang < 90 else ("ahead" if ang == 90 else "left")
    sign_ok = (ang < 90 and ny < 0) or (ang > 90 and ny > 0) or ang == 90
    ok = ok and sign_ok
    failures += not ok
    print(f"{k:>5} {ang:>7.0f}° ({ex:6.2f}, {ey:6.2f})     ({nx:6.2f}, {ny:6.2f})        "
          f"{side:<5} {'ok' if ok else 'WRONG'}")

print(f"\n{len(scans)} printed scans checked, {failures} wrong. exe exit code {p.returncode}.")
print("PASS" if scans and failures == 0 else "FAIL")
sys.exit(0 if scans and failures == 0 else 1)
