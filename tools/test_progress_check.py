"""
Progress-check verification harness (no simulator involved).
 Test 1: scan rate + server-loss path. Mock LiDAR server up, exe runs 6 s with output drained
         continuously, server killed, expect ERR_CONNECTION then TMM critical shutdown.
 Test 2: 'q' key routine shutdown. Exe runs in its own console; a 'q' key event is written
         straight into that console's input buffer (what _kbhit/_getch read).
"""
import subprocess, time, sys, re, threading, ctypes, os
from ctypes import wintypes

EXE = r"Executables\MTRN3500 Assignment 2.exe"
MOCK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mock_lidar.py")

# If anything (e.g. the real Weeder simulator) already listens on the LiDAR port, the mock cannot bind
# and the exe would silently talk to that instead. Refuse to run rather than report bogus results.
import socket
with socket.socket() as _probe:
    if _probe.connect_ex(("127.0.0.1", 23000)) == 0:
        sys.exit("Port 23000 is already in use (is the simulator running?). Close it and re-run.")

def start_mock():
    s = subprocess.Popen([sys.executable, MOCK] + [a for a in sys.argv[1:] if a == "--sra"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    return s

def drain(p, store, t0):
    for line in p.stdout:
        store.append((time.time() - t0, line.rstrip("\n")))

def summarise(lines):
    for t, l in lines:
        if not re.match(r"\s*\d+:", l):          # skip scan data rows
            print(f"  [{t:5.2f}s] {l}")

# ---------------------------------------------------------------- Test 1
print("=== Test 1: scan rate, then server loss ===")
srv = start_mock()
t0 = time.time()
p = subprocess.Popen([EXE], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, stdin=subprocess.DEVNULL)
lines = []
th = threading.Thread(target=drain, args=(p, lines, t0), daemon=True); th.start()
time.sleep(6)
srv.kill()
p.wait(timeout=20); th.join(timeout=5)
summarise(lines)
headers = [(t, l) for t, l in lines if l.startswith("LiDAR scan #")]
if len(headers) >= 2:
    (ta, la), (tb, lb) = headers[0], headers[-1]
    na, nb = int(re.search(r"#(\d+)", la).group(1)), int(re.search(r"#(\d+)", lb).group(1))
    print(f"  scan rate: {(nb - na) / (tb - ta):.1f} scans/s  (from #{na} at {ta:.2f}s to #{nb} at {tb:.2f}s)")
print(f"  exe exit code: {p.returncode}")

# ---------------------------------------------------------------- Test 2
print("\n=== Test 2: 'q' key routine shutdown ===")
k32 = ctypes.windll.kernel32

class KEY_EVENT_RECORD(ctypes.Structure):
    _fields_ = [("bKeyDown", wintypes.BOOL), ("wRepeatCount", wintypes.WORD),
                ("wVirtualKeyCode", wintypes.WORD), ("wVirtualScanCode", wintypes.WORD),
                ("UnicodeChar", wintypes.WCHAR), ("dwControlKeyState", wintypes.DWORD)]
class INPUT_RECORD(ctypes.Structure):
    _fields_ = [("EventType", wintypes.WORD), ("KeyEvent", KEY_EVENT_RECORD)]

def inject_key(pid, ch):
    k32.FreeConsole()
    if not k32.AttachConsole(pid):
        return f"AttachConsole failed: {ctypes.GetLastError()}"
    h = k32.CreateFileW("CONIN$", 0x80000000 | 0x40000000, 3, None, 3, 0, None)
    recs = (INPUT_RECORD * 2)()
    for i, down in enumerate((1, 0)):
        recs[i].EventType = 1
        recs[i].KeyEvent.bKeyDown = down
        recs[i].KeyEvent.wRepeatCount = 1
        recs[i].KeyEvent.wVirtualKeyCode = ord(ch.upper())
        recs[i].KeyEvent.wVirtualScanCode = 0
        recs[i].KeyEvent.UnicodeChar = ch
        recs[i].KeyEvent.dwControlKeyState = 0
    written = wintypes.DWORD(0)
    ok = k32.WriteConsoleInputW(h, recs, 2, ctypes.byref(written))
    k32.CloseHandle(h)
    k32.FreeConsole()
    return f"WriteConsoleInput ok={ok} written={written.value}"

srv = start_mock()
outfile = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "qtest_out.txt"), "w")
t0 = time.time()
p = subprocess.Popen([EXE], stdout=outfile, stderr=subprocess.STDOUT,
                     creationflags=subprocess.CREATE_NEW_CONSOLE)
time.sleep(4)
print("  injecting 'q' at %.1fs:" % (time.time() - t0), inject_key(p.pid, "q"))
try:
    p.wait(timeout=15)
    print(f"  exe exited at {time.time() - t0:.1f}s with code {p.returncode}")
except subprocess.TimeoutExpired:
    p.kill(); print("  exe did NOT exit after 'q' - killed")
outfile.close(); srv.kill()
txt = open(outfile.name, encoding="utf-8", errors="replace").read().splitlines()
for l in txt:
    if not re.match(r"\s*\d+:", l):
        print("  ", l)
