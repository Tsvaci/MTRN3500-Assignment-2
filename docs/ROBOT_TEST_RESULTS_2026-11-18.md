# Handoff: real-robot test session, 18 Nov 2026 (in progress)

For the next Claude Code session. Read this, then `HANDOVER.md` and `docs/ROBOT_TEST_CHECKLIST.md` in
the `full-build` repo. This session ran on Aidan's Mac. Aidan was in the lab on a UNSW lab PC (Windows,
Visual Studio, MATLAB) with the real weeder robot. Aidan relayed console output and screenshots by hand.

## Ground rules Aidan set this session

- **Do not access anything above the `Ass2` folder** (`/Users/aidan/Desktop/University/MTRN3500/Ass2`).
  Unzipping and reading everything inside it is allowed.
- The Mac copy was extracted from `MTRN3500-Exemplar-2-full-build.zip` into
  `Ass2/MTRN3500-Exemplar-2-full-build/` (`full-build` branch contents; no `.git` in the zip).
- Nothing in the code was changed this session. `HANDOVER.md` was not updated (the repo rule says to
  update it on every push, and no push has happened yet).

## Setup that actually works in the lab

1. **Controller:** Xbox controller (not the PS4). No DS4Windows needed; the code is already Xbox
   (`INPUT_TYPE = 0`). Mapping: RT = forward, LT = reverse, right stick X = steering, **B = shutdown**
   (the checklist's "R2/L2/Circle" mean RT/LT/B).
2. **Code on the lab PC:** the lab PC's existing project under `C:\Users\z5481380\source\repos\…` was an
   **older version** (its `TMM.cpp` had no "TMM: thread started"; the exe printed only "Program complete").
   Aidan extracted the full-build zip on the lab PC and built that instead. Build: Debug | x64,
   "1 succeeded, 0 failed". `WEEDER_ADDRESS` set to `"192.168.1.200"` in `include\NetworkedModule.h`
   on the lab PC copy (the Mac copy still has `127.0.0.1` active).
3. **Running:** the console closes on exit (the project has no Linker SubSystem set, so Ctrl+F5 doesn't
   pause). Run the exe from a Command Prompt in `Executables\` so the output stays visible.
4. **MATLAB:** `MATLAB Display Engine\runner.m` must be running first (Display is critical). It shows
   "ready and waiting for connection" and a red line at 0 until the program connects.
5. **Network (the big blocker, now solved):**
   - The lab PC's wired "Ethernet" is the UNSW campus network (`129.94.48.35/22`). It cannot reach the robot.
   - "Ethernet 2" is an unused physical port. "Ethernet 4" (`192.168.56.1`) is VirtualBox, irrelevant.
   - **Working method: join the robot's Wi-Fi.** The PC gets `192.168.1.18/24`, gateway `192.168.1.101`.
     Aidan unplugged the campus cable while doing this. With it unplugged, the `H:\` home drive is
     unavailable ("The current directory is invalid" in cmd). Run from `C:`.
   - **The robot (`192.168.1.200`) does not answer ping**, but TCP works:
     `powershell -Command "Test-NetConnection 192.168.1.200 -Port 23000"` gives `TcpTestSucceeded : True`.
     Don't use ping as the connectivity test.
   - The robot is shared. Another student was using it earlier. The services likely accept one client at a time.

## Bug found (not fixed yet; fix after the robot tests)

When the robot is unreachable, LiDAR/VC/GNSS block in `TcpClient` connect (~20 s Windows timeout)
before reaching the start barrier, so the barrier (`BARRIER_TIMEOUT_MS = 15000`) times out. During that
wait, TMM's own stopwatch (`WatchList[bit_TM]`) is never restarted, because `processSharedMemory()` only
runs in TMM's main loop, after `startThreads()`. Controller and CrashAvoidance leave the barrier, call
`tmmHasStopped()` (elapsed > `TM_ALIVE_TIMEOUT_MS = 4000`), and exit at once. TMM then reports
**"CRITICAL module Controller stopped responding"**, which blames the wrong module. Seen output:

```
Display: connected to 127.0.0.1:28000
CrashAvoidance: thread shut down.
Controller: thread shut down.
TMM: WARNING - not all threads reached the start barrier in time.
TMM: all threads running. Press 'q' to shut down.
TMM: CRITICAL module Controller stopped responding.
...
TMM: WARNING - LiDAR thread did not stop within 3000 ms.
```

Relevant code: `TMM.cpp` `setupSharedMemory()` (starts the TMM watch), `startThreads()` (barrier, then
restarts only the module watches), `UGVModule.h` `tmmHasStopped()` / `waitForStartBarrier()`,
`SMObjects.h` timeouts. A fix should keep TMM's alive signal fresh during startup (or restart the TMM
watch before modules can check it), and ideally report the module that actually failed to connect.
Also consider a connect timeout shorter than the barrier timeout (e.g. `BeginConnect` + wait) so a dead
robot produces "could not connect" lines instead. Discuss with Aidan before changing anything. Once
the network works, startup is fast and this doesn't trigger.

## Test results so far (`docs/ROBOT_TEST_CHECKLIST.md`)

### Section 1, connects: PASS
```
VehicleControl: connected to 192.168.1.200:25000
LiDAR: connected to 192.168.1.200:23000
GNSS: connected to 192.168.1.200:24000
VehicleControl: authenticated.
LiDAR: authenticated.
TMM: all threads running. Press 'q' to shut down.
VehicleControl: crash avoidance active.
CrashAvoidance: LiDAR data OK - checking paths.
```
No barrier warning. zID `5481380` accepted. Real LiDAR replies parse fine: 361 points per scan, no format errors.

### Section 3, GNSS: PASS (with a note)
All CRCs match (`CRC calc X recv X OK`). Values are **not** real UTM: Northing ≈ 6.3 → 5.6,
Easting 24.6 → 25.5 (+0.1 per record), Height 108.389. This matches the synthetic `GPSData.dat`
described in `HANDOVER.md` (~line 230), so the robot replays that file rather than giving a live fix.
Decoding is correct. The checklist's "expect 6,2xx,xxx / 3xx,xxx" does not apply on this robot.

### Section 2, LiDAR + display: PARTLY DONE
- MATLAB plot is live and matches the printed scan in all four regions checked. The program maps the
  robot's left (+y) to screen left (−X), forward to +Y, in mm: points ~4 m left → X ≈ −4000; 6.9 m
  left → X ≈ −6900; 1.5 m right → X ≈ +1500; 7.3 m ahead / 4.3 m right → top right. The code's
  internal mapping is self-consistent.
- **Still to do with a person:** walk in front (plot follows), stand on the robot's **left** (should
  appear at −X; this is the real-world mounting check), stand 1 m ahead with a tape measure (should be
  ≈ X 0, Y 1000), photo of the plot with Aidan to one side.
- **Possible self-reading:** in both scans, points ~302–356 sit at x ≈ 0.0–0.3 m ahead, y ≈ 0.5–0.65 m
  left (≈ 0.55–0.6 m range). Need Aidan to say what is physically there (robot part or a nearby object).
  If it is the robot, crash avoidance may need to ignore it (check how `CrashAvoidance.cpp` treats points
  that close). **Update (Aidan, after the session):** most likely a person or a chair next to the
  robot, not the robot itself. Confirm with the area cleared. In scan #200 only, points 196–210 were ≈ 0.2 m straight ahead (probably a person or hand).

### Sections 4–8: NOT STARTED
Next is section 4 (wheels off the ground): RT forward / LT reverse; **right stick left turns the wheels
left** (most important: the simulator gauge shows the opposite sign); watchdog (close the window while
holding RT); B and `q` both shut down; unplug the controller while holding RT ("DISCONNECTED", wheels
stop). Then section 5 (ground driving), 6 (crash avoidance, record the stopping gap), 7 (measure robot
width, wheelbase, LiDAR position; replace the guesses in `CrashAvoidance.h`: 0.60 / 0.50 / front-centre),
8 (optional failure handling).

## After the lab

- Update `HANDOVER.md` with: the results above, the network method, the robot ignoring ping, GNSS
  being synthetic on the real robot, the measurements, and the startup bug.
- Fix the startup bug and put real measurements into `CrashAvoidance.h`, with Aidan's OK.
- Optionally set Linker → System → SubSystem = Console so Ctrl+F5 keeps the window open.
