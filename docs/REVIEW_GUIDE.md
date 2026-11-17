# Review guide: full build, by marking component

This guide walks through the `full-build` code one marking component at a time. For each one it covers what the
demonstrator will see, where the code is, how it works, and the questions a demonstrator is likely to ask.
File references are `file:line`. Files are in `MTRN3500 Assignment 2/` unless they start with `include/`.

---

## The big picture (read first)

```
            Xbox controller                       LiDAR (23000)          GNSS (24000)
                  |                                     |                     |
            [Controller]                            [LiDAR]               [GNSS]
                  | speed, steering                     | 361 x,y (m)          | N, E, H
                  v                                     v                     v
          SM_VehicleControl                         SM_Laser               SM_GPS
              |        \                         /         \
              |         \                       /           \
              |        [CrashAvoidance] <------              [Display] ---> MATLAB (28000)
              |                | limits
              |                v
              |        SM_CrashAvoidance
               \               |
                v              v
                 [VehicleControl] ---> vehicle (25000)  "# steer speed wdog #"

  [TMM] starts all six threads, watches their heartbeats, restarts or shuts down.
```

- **One program, seven threads.** `Main.cpp` starts the Thread Management Module (TMM). The TMM creates all the
  shared memory and the other six threads.
- **Threads never call each other.** They only read and write the shared-memory objects in `include/SMObjects.h`.
  Each object has a lock, so two threads never touch it at the same time.
- **Every module has the same three functions** required by the base class `UGVModule`:
  - `processSharedMemory()` moves data in or out of shared memory
  - `getShutdownFlag()` asks "should I stop?"
  - `threadFunction()` is the thread's main loop

  Networked modules (LiDAR, GNSS, VC, Display) also have `connect()` and `communicate()`.

---

## 1. Architecture: thread management, heartbeats, shutdown

**Demonstrator sees:** all threads start, run, and stop cleanly. A critical failure stops everything; a non-critical
failure restarts just that thread.

**Where:** `TMM.cpp`, `include/SMObjects.h`, `include/UGVModule.h`.

**How it works**
- *Start-up.* `setupSharedMemory` (TMM.cpp:13) creates every shared object. `startThreads` (TMM.cpp:50) starts
  the threads in a set order: Controller, LiDAR, CrashAvoidance, VC, Display, GNSS. Inputs come before the
  actuator, and the safety thread is running before the first drive command.
- *Start barrier.* Every thread connects, then waits at a barrier, so they all begin together. Afterwards the TMM
  removes the barrier (restarted threads skip it) and resets every heartbeat timer, so a slow start-up isn't
  mistaken for a dead thread.
- *Heartbeats.* Each thread sets its own bit in `SM_TM_->heartbeat` on every loop. `checkHeartbeats`
  (TMM.cpp:142) runs every 50 ms. When it sees a bit, it clears it and restarts that thread's stopwatch. A
  stopwatch past 2 s means that thread has stopped.
- *Critical vs non-critical* (`include/SMObjects.h:37`):
  - **Critical:** LiDAR, Controller, VC, Display. Losing one means a full shutdown, and the program exits with
    code 2.
  - **Non-critical:** GNSS and CrashAvoidance. Losing one means that thread is restarted.
- *Shutdown.* The `q` key, or the controller's B button, leads to `shutdownModules` (TMM.cpp:178). It sets every
  thread's shutdown bit, then waits up to 3 s for each thread to finish and reports it.
- *TMM itself dying.* Each module also stops if the TMM's own stopwatch goes quiet for 4 s
  (`tmmHasStopped` in `include/UGVModule.h`).

**Likely questions**
- *Why is crash avoidance allowed to be non-critical?* If it stops, its limits go stale and Vehicle Control holds the
  vehicle still (VC.cpp:68), so it is safe to restart it rather than shut down.
- *Why a barrier?* Without it a fast thread (e.g. VC) could start driving before LiDAR or crash avoidance were
  ready.
- *Why `Monitor::Enter/Exit` with try/finally?* The lock is always released, even if an exception is thrown while
  holding it.

---

## 2. Display (MATLAB plot)

**Demonstrator sees:** run `MATLAB Display Engine/runner.m` first, then the program. A live top-down view: the
vehicle is at the bottom centre, ahead is up the screen, and left is on the left.

**Where:** `Display.cpp`.

**How it works**
- `processSharedMemory` (Display.cpp:32) copies the latest scan and converts it for the display:
  - metres to **millimetres** (× 1000)
  - our frame (x forward, y left) to the display engine's frame (X to the right, Y forward):
    **X = −y × 1000, Y = x × 1000**
- `sendDisplayData` (Display.cpp:50) is the routine supplied with the spec. Each 361-number array is sent as its
  raw bytes (361 × 8 = 2888 bytes): X first, a 10 ms pause, then Y. Ten frames per second.

**Why the frame swap (found in testing, 17 Nov 2026).** The provided engine has fixed axes: X from −8000 to +8000
across the screen and Y from 0 to 8000 ahead. Sending x-forward data turned the plot sideways and dropped
everything on the vehicle's right off the chart. Checked by running the real engine in MATLAB R2025a: an obstacle
2 m ahead appears at (0, 2000), and one on the left appears on the left. **This is a spec question for you:** the
draft says "vehicle base frame", but the engine only displays correctly in X-right, Y-forward. This may also
explain some students' odd plots.

**Likely questions**
- *What units does the display need?* Millimetres.
- *Why the 10 ms gap?* So the engine can tell the X array from the Y array.
- *What happens if MATLAB isn't running?* `connect` fails with a message. Display is critical, so the program
  shuts down.

---

## 3. GNSS with both CRCs

**Demonstrator sees:** lines like
`GNSS: Northing 0.100  Easting 0.100  Height 108.389  CRC calc F227E7D6 recv F227E7D6 OK`
A bad record prints `CRC MISMATCH` with both values and is thrown away.

**Where:** `GNSS.cpp`.

**How it works**
- The receiver streams binary records continuously; there is no request and no login. A record can arrive split
  across two reads, or several can arrive at once.
- `communicate` (GNSS.cpp:36) appends whatever arrived to a buffer, then repeatedly:
  1. finds the sync bytes `AA 44 12` (`findHeader`)
  2. waits until a whole record (112 bytes) is there
  3. hands it to `decodeRecord`
- `decodeRecord` (GNSS.cpp:91):
  1. checks the header: length 28, message ID 726 (BESTUTM), body length 80
  2. computes the CRC32 over the 108 header+body bytes, using the spec's routine with its `else` fixed
     (GNSS.cpp:233)
  3. compares it with the 4-byte CRC that follows the body
  4. only if they match, reads Northing, Easting and Height (doubles at body offsets 16, 24 and 32)
- Non-critical: if the connection drops, the thread ends and the TMM restarts it (tested: it reconnects and carries
  on).

**Good to know:** the simulator's `GPSData.dat` is synthetic. Northing and Easting start at 0 and Easting rises by
0.1 per record (zone 56 H, height 108.4 m). Small numbers are correct, not a decoding bug.

**Likely questions**
- *Why look for the sync bytes instead of just reading 112 bytes at a time?* If you join mid-stream or lose a byte,
  fixed-size reads stay misaligned forever. Searching for the header re-synchronises.
- *What is `pin_ptr` for?* The CRC routine needs a plain C pointer. Pinning stops .NET's garbage collector moving
  the array while the routine reads it.

---

## 4. Driving with the controller

**Demonstrator sees:** right trigger drives forward, left trigger reverses, the right stick steers. Unplug the
controller and the vehicle stops.

**Where:** `Controller.cpp` (input), `VC.cpp` (sending).

**How it works**
- `readController` (Controller.cpp:20):
  - speed = (right trigger − left trigger) × 1 m/s
  - steering = −(right stick X) × 40°. Stick right is positive, but our steering is positive to the left, hence
    the minus.
  - a 10% deadzone ignores stick drift
  - if `isConnected` is false, speed and steering are set to zero and a message is printed once
- Results go into `SM_VehicleControl` every 20 ms.
- Vehicle Control (`VC.cpp`):
  - logs in with the zID (`authenticate`, VC.cpp:39)
  - every 50 ms reads the driver's command and the crash-avoidance limits, applies them (`processSharedMemory`,
    VC.cpp:68), and sends `# <steer> <speed> <watchdog> #`
  - the watchdog flips 0/1 on every message; if it stops changing, the vehicle stops by itself
  - a final stop command is sent on shutdown
  - twice a second while driving it prints what was asked and what was sent, marked `LIMITED` when crash
    avoidance changed it

**Settings:** `INPUT_TYPE` in `Controller.h` (0 = Xbox, 1 = keyboard). Keyboard mode has no triggers or buttons,
so speed comes from the right-stick-Y key and the B-button shutdown can't be shown.

**Likely questions**
- *Why is `controllerState` a local variable, not a class member?* It is a native C++ struct and the class is a
  managed `ref class`. Mixing them is a compile error (C4368).
- *Why does `ControllerInterface.h` have to be included first?* It pulls in `Windows.h`, which clashes with
  `using namespace System` (error C3699) if that comes first.

---

## 5. Shutdown by key and by controller button

**Demonstrator sees:** `q` in the program's window, or **B** on the controller, stops every thread. The TMM lists
each thread as stopped, and the program exits.

**Where:** `shutdownKeyPressed` (TMM.cpp:170), `requestShutdown` (Controller.cpp:76), `shutdownModules`
(TMM.cpp:178).

**How it works**
- Both routes lead to the same shutdown code in the TMM.
- The `q` key is read without blocking (`_kbhit` / `_getch`) inside the TMM's 50 ms loop.
- The B button sets the TMM's own shutdown bit. The TMM sees it on its next loop and runs the same routine.
- Measured exit time after `q`: 0.1 s.

---

## 6. Crash avoidance report

`docs/crash_avoidance_report.md`, about one page. It covers the collision scenarios, the method, the responses
table, fail-safes and limitations. The code follows it step for step (section 7 maps each step to a function).

---

## 7. Crash avoidance implementation (marking item f, 6 marks)

**Demonstrator sees:**
- An obstacle placed in front while driving forward: the vehicle slows as it gets closer and stops at 0.3 m.
  Reversing still works.
- An obstacle beside the vehicle while turning towards it: the turn is eased. Turning away is unaffected.
- The VC status line shows `LIMITED (clear path 0.62 m)` etc.

**Where:** `CrashAvoidance.cpp`, the new `SM_CrashAvoidance` (`include/SMObjects.h:112`), and the limit-applying
part of `VC.cpp:68`.

**How it works, step by step (every 20 ms)**

| Report step | Function | Plain-language summary |
|---|---|---|
| Inputs, stale check | `readInputs` (CrashAvoidance.cpp:23) | Copies the scan and the driver's command. Stale if the LiDAR's scan counter hasn't changed for 200 ms |
| Predict the path | `distanceAlongPath` (:67) | For a steering angle, the vehicle drives an arc of radius wheelbase / tan(steer). Is a point inside the vehicle-wide corridor along that arc, and how far along is it? |
| Measure clearance | `pathClearance` (:96) | Distance to the second-nearest scan point on that path (ignores single stray points and no-echo zeros), up to 2 m |
| Ease the steering | `computeLimits` (:141) | Checks 17 paths. Each side may steer up to the sharpest angle whose path is clear for 1 m. If straight ahead is blocked, steering is left free so the driver can steer out |
| Limit the speed | `speedLimitFor` (:124) | Full speed with ~1 m clear, zero at 0.3 m, a braking curve in between, with extra margin at higher requested speed |
| Publish | `publishLimits` (:172) | Writes the max forward speed, max left/right steering, clearance, and a timestamp |
| Apply | `VehicleControl::processSharedMemory` (VC.cpp:68) | Caps the driver's command to the limits, for forward motion only. If the limits are older than 300 ms, holds the vehicle still |

**Why limits rather than a yes/no flag:** the revised spec rewards slowing and steering adjustment, not just stopping.
Publishing limits lets Vehicle Control apply them to the driver's newest command, and keeps the driver in charge
inside a safe envelope.

**Tested results** (stand-in LiDAR with obstacles placed in code; 2 cm–40 cm circular posts):

| Scenario | Asked | Sent |
|---|---|---|
| Clear road | 1.0 m/s straight | 1.0 m/s straight |
| Obstacle 2.0 / 0.8 / 0.5 / 0.35 m ahead | 1.0 m/s | 1.00 / 0.76 / 0.38 / 0.00 m/s |
| Obstacle 0.35 m ahead, reversing | −1.0 | −1.0 (allowed) |
| Obstacle front-left, turn hard left | +40° | +20°, still 1.0 m/s |
| Obstacle front-left, turn hard right | −40° | −40° (unaffected) |
| Obstacle front-right, turn hard right | −40° | −20° |
| Blocked 0.5 m ahead, turn hard left | +40°, 1.0 | +40°, 0.0 (steering free, speed stopped) |
| One stray reading 0.4 m ahead | 1.0 | 1.0 (ignored) |
| 2 cm pole 0.5 m ahead | 1.0 | 0.38 |

**Likely questions**
- *Why the second-nearest point?* Dust or a single bad return shouldn't stop the vehicle. A real obstacle, even a
  2 cm pole at 1 m, returns several points (beams are about 9 mm apart at 1 m).
- *Why not limit reversing?* There's no LiDAR behind the vehicle, so we can't see what we'd hit. The spec says
  only forward and turning motion is required.
- *Why leave steering free when the way ahead is blocked?* Steering away is how you get out. The speed limit still
  applies to whichever path is chosen, so steering into the obstacle stays stopped.
- *Where does "current vehicle motion" come in?* The path is predicted from the steering being commanded. The
  stopping margin grows with the requested speed (0.1 s reaction distance), so a faster approach starts slowing
  earlier.

**Assumptions to check on the real robot:** vehicle half-width 0.30 m + 0.10 m margin, wheelbase 0.50 m, LiDAR at the
front centre (constants at the bottom of `CrashAvoidance.h`). If the LiDAR sees part of the robot, those angles need
masking. See "Open questions" below about the simulator.

---

## 8. Error handling and validation

Each item below is something a demonstrator might pull the plug on, and what the code does.

| Fault | What happens | Where |
|---|---|---|
| LiDAR / VC / Display server not running at start | Clear message; thread stops after the barrier; TMM sees no heartbeat and shuts down (critical) | each `threadFunction` |
| LiDAR / VC / Display lost mid-run | Read/write fails → thread stops → full shutdown, exit code 2 (tested with Display) | `TMM.cpp:142` |
| LiDAR connected but silent | Three 1 s timeouts in a row → thread stops → full shutdown | `LiDAR.cpp:232` |
| GNSS lost | Thread ends → TMM restarts it every 2 s until it reconnects (tested) | `GNSS.cpp:182` |
| Bad LiDAR telegram | Rejected unless it has `sRA`/`sRN LMDscandata`, a `DIST1` block, exactly 361 points and valid hex | `LiDAR.cpp:119` |
| Bad GNSS record | Rejected unless header length, message ID, body length **and** CRC all match | `GNSS.cpp:91` |
| Login refused (wrong zID) | `ERR_AUTHENTICATION` printed with the server's reply | `authenticate` in LiDAR/VC |
| Controller unplugged | Speed and steering zero, message printed once, "reconnected" when back | `Controller.cpp:20` |
| Crash avoidance thread dies | VC holds the vehicle; TMM restarts crash avoidance | `VC.cpp:68` |
| Program shutting down | VC sends a final stop command before closing | `VC.cpp:188` |

Every function returns an `error_state`, and `UGVModule::printError` turns it into a readable message.

---

## How it was tested

- `tools/test_full_build.py` drives the program against stand-in servers (`tools/mock_weeder.py` for LiDAR, GNSS
  and VC; `tools/mock_display.py`) using a test-only copy of the program whose controller reads from a file
  (`tools/build_test_exe.py`; the repo code is not changed). **36/36 checks pass.** It covers every table above,
  plus the `q` key, the B button, a lost display, and a dropping GNSS.
- The real MATLAB display engine was run against the program to confirm the plot (see section 2).
- Not yet done: the real Weeder simulator (needs your OK) and the real robot. See below.

Run it yourself (repo root, after building):
```
python tools/build_test_exe.py %TEMP%\mtrn_test
python tools/test_full_build.py "%TEMP%\mtrn_test\Executables\MTRN3500 Assignment 2.exe"
```

## Open questions for you

1. **Display frame:** the spec says "vehicle base frame", but the provided engine only shows X-right / Y-forward
   correctly (section 2). Which wording should the spec use?
2. **Steering sign and command format:** the code assumes positive steering = left and sends
   `# 12.345 0.500 1 #` with no newline. Both need confirming on the simulator.
3. **Simulator LiDAR is frozen** and shows a return 0.16 m to the left plus 0.79 m ahead. Good crash avoidance will
   therefore refuse to drive forward in the simulator. Students may disable their safety code to make the driving
   demo work. The simulator should generate live scans.
4. **Robot dimensions** for crash avoidance (width, wheelbase, LiDAR position).
