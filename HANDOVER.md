# HANDOVER — read this first in every fresh chat

This file exists so a fresh Claude Code chat can pick the project up with no other context.
**It must be updated on every push.** Edit the "Current status" and "Push log" sections at minimum.

---

## 1. What this project is

Aidan (the user) is a professor writing an **exemplar solution** for a proposed MTRN3500 Assignment 2
(UNSW, "UGV / weeder" tele-operation assignment). The goal is to prove the assignment is doable by
students and to have a reference implementation he can examine at the marking points.

The assignment: a single Visual Studio **C++/CLI** project where a Thread Management module starts,
monitors and shuts down six other threads that talk through shared memory. The threads connect over
TCP to a LiDAR, a GNSS receiver, a vehicle-control service, a MATLAB display engine, and read an Xbox
controller. A crash-avoidance thread inhibits motion near obstacles.

## 2. Standing orders from Aidan (follow these exactly)

1. **Folder access is restricted.** You may read and write only inside
   `C:\Users\User\OneDrive - Lee Family Office\Desktop\Personal\Uni\MTRN3500\Ass 2`
   (and this repo, which lives inside it). Do **not** open any sibling folder
   (`Ass 1`, `Lectures`, `Quiz 1 Practice`, `GalilSimulator`, or anything else) without his
   express approval in the chat.
   **Second machine (home PC, from 3 Nov 2026):** the same folder is
   `C:\Users\aidan\Desktop\UNI\MTRN3500\Ass 2` (repo and `WeederSimulator` only; no PDFs or zip
   there). The same restriction applies.
   **Third machine (Aidan's Mac, used 18 Nov 2026 for the robot test):** only
   `/Users/aidan/Desktop/University/MTRN3500/Ass2` and below (it holds an extracted
   `full-build` zip with no `.git`).
2. **Two development branches.**
   - `progress-check` — the Week 8 progress-check deliverable (Section 9.1 of the draft). This is a
     marking point Aidan will examine, so it must stand alone and stay clean.
   - `full-build` — the complete Week 11 final-demonstration solution (Section 9.2).
   - `main` holds the starter code, the skeleton and the docs. Branch the two above off `main`.
3. **Order of work:** skeleton commit on `main` first, then the progress-check portion on
   `progress-check`, then the full build on `full-build`.
4. **Update this file on every push.** It must be enough to bring a fresh chat fully up to speed.
5. **Aidan directs; explain in plain language.** He is not a day-to-day coder. Lead with outcomes,
   give the numbers, keep code out of prose. Flag spec problems to him rather than silently
   working around them.
6. **Verify, don't assume — but never launch or unzip the simulator.** The machine has Visual
   Studio 2026 Community and MATLAB R2025b. Build, then test against the Python stand-in servers in
   `tools/` (Aidan's order, 30 Oct 2026: "do not try launching or unzipping the simulator").
   Aidan runs the real simulator himself. **Exception, 3 Nov 2026:** in the home-PC session Aidan
   said "you can use weeder simulator", and Claude ran it for testing. Treat that as a one-session
   permission; ask again before launching it.
7. Commit messages end with the `Co-Authored-By:` line for whichever Claude model is doing the work.
8. Keep `HANDOVER.md` identical on `main` and on the working branch: when you push a branch, copy the
   updated file onto `main` too.

### Aidan's instruction, verbatim (30 Oct 2026)

> Thank you for picking up on those, I wrote this when I was tired. That skeleton commit sounds
> positive, I want to develop the code in two separate branches. One branch for the progress check
> as that will be a marking point I will be examining, and I also want to do the full build on a
> separate branch. Let's do the skeleton commit and the progress check portion first. I also want
> you to write an instructional in the github repo for when I move to a fresh chat. This md should
> be updated everytime a push is made and it should be sufficient in catching up a fresh claude
> chat up to speed.

The originating Claude Code session is named **"MTRN3500 Ass 2 Alpha"**
(ID `f54a262b-a1d1-4e74-9fe1-af1ee8879a3a`, started in `C:\Users\User`). Resume it with
`claude --resume f54a262b-a1d1-4e74-9fe1-af1ee8879a3a` from that folder if the transcript is needed.

## 3. Where things are

| Item | Location |
|---|---|
| Repo (this folder) | `…\Ass 2\MTRN3500-Exemplar-2` — remote `https://github.com/Tsvaci/MTRN3500-Exemplar-2.git` (push works via `gh`, account AidanLFO) |
| Assignment draft (v1, 14 pages) | `…\Ass 2\MTRN3500_Assignment_2_Draft-v1.pdf` |
| Service details (ports, auth) | `…\Ass 2\MTRN3500_Assignment_2_Service_Details__2026.pdf` |
| Starter code zip (original) | `…\Ass 2\weeder-assignment-unsw-main-starter-code.zip` |
| Simulator | `…\Ass 2\WeederSimulator\Weeder_Simulator.exe` (+ `GPSData.dat`, GLUT DLLs). Run it from its own folder. |
| Datasheets | `…\Ass 2\SICKLMS151.pdf`, `GPSManualExtracts.pdf` (Novatel BESTUTM), `GR-U01_GNSS_Info.pdf` |
| PDF text extraction | `pdftoppm` is not installed; `pypdf` is (pip, user site). Use `PYTHONIOENCODING=utf-8`. |
| MSBuild | `C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe` |

## 4. Repo layout

```
MTRN3500-Exemplar-2/
  HANDOVER.md                       <- this file
  CLAUDE.md                         <- one-line pointer to this file
  README.md
  .gitignore                        (from the starter; ignores .vs/, x64/, Executables/)
  MTRN3500 Assignment 2.sln
  include/                          provided headers (UGVModule.h, NetworkedModule.h,
                                    SMObjects.h, ControllerInterface.h)
  libs/                             ControllerInterface_*.lib files, linked by the project.
                                    x64_Debug is the UPDATED lib (VS > 17.9, see §6); the original is
                                    kept as ControllerInterface_x64_Debug_old.lib. include/ likewise has
                                    the updated ControllerInterface.h and the original as _old.h.
  MATLAB Display Engine/            provided, run runner.m in MATLAB (TCP server on 127.0.0.1:28000)
  MATLAB Legacy Display Engine/     provided
  MTRN3500 Assignment 2/            the Visual Studio project
    MTRN3500 Assignment 2.vcxproj   toolset v143, CLR on, .NET 4.7.2, includes ../include, links ../libs
    Main.cpp                        (starter: empty main)
    TMM.h / TMM.cpp                 (starter: stub ThreadManagement class)
```

Module files (exact names required by the spec, two files per module): `LiDAR`, `GNSS`,
`Controller`, `VC`, `Display`, `CrashAvoidance` (all present); bonus `GRU01` not written yet.
On `full-build` also:
```
  docs/crash_avoidance_report.md    the one-page crash-avoidance report (student deliverable)
  docs/REVIEW_GUIDE.md              Aidan's walkthrough of the code per marking component
  docs/ROBOT_TEST_CHECKLIST.md      what to test on the real robot, ticked with 18 Nov 2026 progress
  docs/ROBOT_TEST_RESULTS_2026-11-18.md  lab-session notes: network setup, results, startup bug
  docs/OBSTACLE_SIM_TEST_2026-11-21.md   14 scenarios against the Python UGV Obstacle Simulator
  tools/run_obstacle_sim_scenarios.py    drives the test exe against that simulator, headless
  tools/summarise_obstacle_sim.py        per-scenario summary of that run
  tools/mock_weeder.py              stand-in LiDAR (scene from a JSON file) + GNSS replay + VC logger
  tools/mock_display.py             stand-in MATLAB display (logs the X/Y arrays it receives)
  tools/build_test_exe.py           builds a test-only exe whose Controller reads inputs from a file
  tools/test_full_build.py          36-check end-to-end test (4 runs)
```
`tools/test_progress_check.py` and `test_moving_obstacle.py` only work on `progress-check` (on
`full-build` the exe also needs VC and display servers, and the LiDAR print is every 100 scans).

## 5. Spec facts you need (so you don't re-read the PDFs)

**Architecture (draft §4, diagram p.6).**
- `ThreadManagement`, `Controller`, `CrashAvoidance` inherit `UGVModule` directly.
- `LiDAR`, `GNSS`, `VehicleControl`, `Display` (and `GRU01`) inherit `NetworkedModule`, which adds
  `connect(String^, int)`, `communicate()`, and protected `TcpClient^ Client`, `NetworkStream^ Stream`,
  `array<unsigned char>^ ReadData`.
- Every module overrides `processSharedMemory()`, `getShutdownFlag()`, `threadFunction()`.
- Shared memory objects (SMObjects.h): `SM_ThreadManagement` (lock, `shutdown` byte, `Barrier^`,
  `array<Stopwatch^>^ WatchList`, `heartbeat` byte), `SM_Laser` (x[361], y[361]), `SM_GPS`
  (Northing, Easting, Height), `SM_VehicleControl` (Speed, Steering). Bit flags `bit_TM … bit_CRASHAVOIDANCE`.
  Crash avoidance needs a new SM object; students are told to add one.
- `main` creates the TMM only; TMM creates shared memory and all other threads.

**Critical vs non-critical (from the draft).** Critical: TMM, LiDAR, Controller, VC, Display.
Non-critical: GNSS and crash avoidance (draft doesn't say; **Aidan decided 2 Nov 2026: restartable**).
Critical failure → full shutdown. Non-critical failure → restart that thread.

**Network services (service details doc).**
| Service | Address | Port | Auth | Notes |
|---|---|---|---|---|
| LiDAR | weeder | 23000 | yes | send `<zID digits>\n` (8 chars), expect `OK\n`. Then `0x02 sRN LMDscandata 0x03`. Reply is `sRA LMDscandata …` per the datasheet, but **the simulator replies `sRN LMDscandata …`** (found by Aidan 2 Nov 2026); accept both. ASCII hex fields; preconfigured 50 Hz, 0–180°, 0.5° step → 361 `DIST1` points in mm. |
| GNSS | weeder | 24000 | no | continuous Novatel binary. Sync `AA 44 12`, header len at byte 3 (=28), msg ID at 4 (726 = BESTUTM), body len at 8 (=80). Northing double at H+16, Easting H+24, Height H+32, CRC32 at H+80 over header+body. `GPSData.dat` in the simulator folder is this exact stream. |
| Vehicle control | weeder | 25000 | yes | send `# <steer> <speed> <wdog> #`, steer ±40°, speed ±1 m/s, wdog alternates 0/1; freeze it to inhibit motion. |
| Display | 127.0.0.1 | 28000 | no | MATLAB server. Send X array as bytes, sleep 10 ms, then Y array (Appendix B). **Values must be in millimetres**, and the provided engine draws X across the screen (−8000..8000, + right) and Y ahead (0..8000) — so Display sends X = −y, Y = x (checked in MATLAB R2025a, 17 Nov 2026; see §6) (Aidan's clarification to students, 17 Nov 2026: "send array<Byte>^ x and array<Byte>^ y pair as millimeters"). SM_Laser holds metres, so Display multiplies by 1000 before sending. |
| GR-U01 (bonus) | weeder | 26000 | no | NMEA `$GNGGA`/`$GPGGA` text. |

"weeder" = `127.0.0.1` for the simulator, `192.168.1.200` for the real robot (`WEEDER_ADDRESS` in NetworkedModule.h).

**LiDAR angle convention (service details Fig. 1).** 0° points to the vehicle's right, 90° straight
ahead, 180° to the left (five of the figure's six labels agree; the "270°" label on +X contradicts
them and is flagged to Aidan). 361 points at 0.5° steps. Vehicle base frame: x forward, y left.
Convert (as implemented in `LiDAR.cpp`): **x = r·sin θ, y = −r·cos θ**, r in metres.
The display must be in the vehicle base frame.

**Controller.** Speed = right trigger forward, left trigger reverse. Steering = right-stick X. One
button triggers shutdown. If controller disconnected → speed and steering zero. `ControllerInterface`
is a native (non-CLR) class; keyboard mode is selected by the constructor argument `input_type = 1`.

**Marking points.**
- Progress check (§9.1, 5 marks): (i) class structure for all modules present, inheriting and
  overriding correctly, bodies may be empty; (ii) LiDAR thread printing 361 Cartesian x,y pairs live;
  (iii) pressing `q` in the TMM window shuts every thread down.
- Final (§9.2, 20 marks): display plot, GNSS print with both CRCs, drive from controller, shutdown by
  key and by controller button, crash-avoidance report, crash-avoidance implementation, architecture,
  error handling and validation.

**Crash-avoidance wording revised by Aidan (17 Nov 2026).** Described to students as a clarification,
not a new requirement. Part 7 now reads, in substance:
- The system must (1) process controller and LiDAR data, (2) assess collision risk from the movement
  commands and the LiDAR data, (3) take appropriate action *such as reducing speed, adjusting vehicle
  motion, or stopping when necessary*.
- If the UGV is moving towards an obstacle within **1 m or less** it must respond appropriately:
  reduce speed, adjust steering, or inhibit motion. No LiDAR behind the vehicle, so only forward
  movement and turning motions need avoidance.
- Marked on correctness (detects and responds), sophistication (considers both directional inputs and
  current vehicle motion), robustness (varying conditions, edge cases); mainly critical thinking about
  how the UGV can collide.
- Needs a new/changed shared-memory structure; students must understand the LiDAR data format.
- Marking item f (6 marks), crash avoidance as a combination of LiDAR and VC data: obstacle in front
  while moving → appropriate action (slow down, or stop immediately if needed); obstacle to the side
  when turning that way → adjust the turn, slow down, or stop. Must match the one-page report; the
  student must be able to explain the code or gets zero.
- Consequence for the exemplar: the old "inhibit flag + frozen watchdog" (stop-only) design was
  replaced in `full-build` by graded speed limiting and steering easing (see §7).

## 6. Problems found in the draft (Aidan has acknowledged these; fix in the spec, not in code)

- Appendix A CRC code: the `else` branch is empty and `ulCRC >>= 1;` sits outside it. Correct form:
  `if (ulCRC & 1) ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL; else ulCRC >>= 1;`
- §3.5 says keyboard mode is a `#define` at the top of ControllerInterface.h; the header actually uses
  a constructor argument.
- §6 refers to `networked_module.h`; the file is `NetworkedModule.h`.
- p.7 typo "ftle"; p.12 is blank.
- §11 says submit only the module `.h/.cpp` files plus main, but students must also edit
  `SMObjects.h` (crash avoidance) and `NetworkedModule.h` (address) — submission list should include them.
- Starter zip ships `.vs/`, `Debug/`, `x64/` and `.vcxproj.user` — strip before release.
- **Provided ControllerInterface library does not link on VS newer than 17.9** (`LNK2028`/`LNK2019`
  unresolved `ControllerInterface::GetState`). Aidan supplied an updated `ControllerInterface_x64_Debug.lib`
  and header on 2 Nov 2026 (header change: `controllerState` is now a native `struct`, was `value struct`).
  Knock-on that every student will hit: a `ref class` can no longer hold a `controllerState` member
  (`error C4368: mixed types`), so read it into a local each poll. Only the x64 Debug lib was updated
  (no `.pdb` supplied → harmless `LNK4099` warnings); the release should ship all four libs + pdbs, and
  keep the old ones for lab machines on older VS.
- **The simulator's LiDAR is a frozen recording (found 3 Nov 2026).** Every `sRN LMDscandata` reply
  is byte-for-byte identical, including the scan-counter and timestamp header fields, whether or
  not the robot is driven with the arrow keys (414 replies over 45 s, checked with a raw socket
  client that bypassed the exe). So the scan never reflects the simulated world: the obstacles it
  shows (0.79 m dead ahead, 0.17 m to the left) are not on the map, the printout can't be shown to
  change, and left/right can't be checked on it. Students can't demonstrate "live" data and a
  marker can't tell live reading from a looped copy. Fix in the simulator (generate scans from the
  world, or at least advance the counters), or change the progress-check wording.
- **Display engine frame vs "vehicle base frame" (found 17 Nov 2026).** The engine's axes are fixed at
  X −8000..8000 (across the screen, + right) and Y 0..8000 (ahead), view [0 80]. Data sent in the
  vehicle base frame (x forward, y left) appears rotated 90°, with everything on the vehicle's right
  clipped off. Display therefore sends X = −y, Y = x. The spec wording should say what frame the
  engine expects. Raised with Aidan; may explain students' odd plots.
- **Frozen simulator LiDAR blocks crash avoidance (confirmed on the simulator, 17 Nov 2026).** The
  frozen scan has returns 0.16 m to the left (indices 272–360, x ≈ 0–0.11 m) and 0.79 m ahead. The
  0.16 m points sit inside the vehicle corridor right at the sensor, so the exemplar's crash avoidance
  allows no forward motion at all in the simulator ("clear path 0.00 m"), straight or turning;
  reversing works. Students will hit this and may disable their safety code to demo driving. The
  SICK LMS151's specified minimum range is believed to be 0.5 m (check `SICKLMS151.pdf`), which
  suggests these 0.16 m readings are not realistic. Ignoring short ranges in code is unsafe (an
  approaching obstacle would vanish), so the fix belongs in the simulator. Awaiting Aidan's call.
- **Simulator steering gauge shows the opposite sign (17 Nov 2026).** Commanding +20° turns the
  simulated vehicle LEFT (wheels and world rotation agree) but its Steer gauge reads −20.0. The
  exemplar's convention (+ = left, matching the y-left vehicle frame) is correct for the motion;
  students reading the gauge will be confused. Fix the gauge or state the convention in the spec.
- **Simulator facts (17 Nov 2026):** VC login is the same as the LiDAR (`<zID>
` → `OK
`); the
  command `# <steer> <speed> <wdog> #` with no newline is accepted and logged in `log_VC.txt`; the
  server sends nothing back after login; a frozen watchdog value stops the vehicle even with speed
  0.5 (checked by comparing frames); GNSS streams 2 records/s.
- **`GPSData.dat` is synthetic.** Northing and Easting start at 0; Easting rises 0.1 per record,
  Northing wanders a few metres either side of 0; zone 56 H, height 108.389. Correct output looks
  like "Northing 0.100 Easting 0.100" — students may think they have a decoding bug.
- Project pins toolset `v143`; Visual Studio 2026 on this machine has MSVC 14.51 only, so expect a
  retarget prompt (or build with `/p:PlatformToolset=v145`). Spec should mention this.

## 7. Agreed design for the exemplar

- TMM constructs every SM object in `setupSharedMemory()` and passes handles into each module's
  constructor.
- Heartbeat: each thread ORs its bit into `SM_TM_->heartbeat` under the lock; TMM clears it and
  restarts that thread's stopwatch when seen. Stale beyond a timeout = thread dead.
- Shutdown: TMM writes `bit_ALL` to `SM_TM_->shutdown`; each `getShutdownFlag()` tests its own bit.
  `q` in the TMM console and the controller shutdown button both go through this path.
- Start order: Controller, LiDAR, CrashAvoidance, VC, Display, GNSS. Inputs before actuator, safety
  gate before the first drive command. A `Barrier` releases all threads together once connected.
- Crash avoidance publishes an inhibit flag in a new `SM_CrashAvoidance`; VC honours it by freezing
  the watchdog. Because crash avoidance is restartable, the flag is **fail-safe**: it starts `true`, so
  nothing moves until crash avoidance has cleared a scan. In `full-build`, VC must also treat a stale
  crash-avoidance update (thread down or restarting) as inhibit.
  **Superseded in `full-build` (17 Nov 2026) after the spec wording changed:** crash avoidance now
  publishes limits (maximum forward speed, steering limits) rather than a yes/no flag, so it can slow
  down and ease the steering as well as stop. The fail-safe idea stays: limits start at "stopped" and
  VC stops if crash avoidance goes quiet. `progress-check` keeps the original flag (no change there).
  Implemented design (`CrashAvoidance.cpp`, every 20 ms): predict the path as an arc of radius
  wheelbase/tan(steer), corridor = half-width 0.30 + margin 0.10 m; clearance = distance along the arc
  to the **second-nearest** scan point in the corridor (ignores single stray points; range 0 ignored;
  capped at 2 m); steering limits = per side, the sharpest of 5°…40° whose path is clear ≥ 1 m
  (left free if straight ahead is already < 1 m); speed limit on the path actually driven
  v = √((d − 0.1·v_req − 0.3)/0.7), capped at 1 m/s (full at ~1 m, zero at 0.3 m). VC applies the limits
  to forward motion only; reversing is never limited (no rear LiDAR). Assumed geometry (width,
  wheelbase, LiDAR at front centre) is flagged for measurement on the real robot.

## 8. Current status

**Branch `main`:** skeleton complete. Updated ControllerInterface lib/header in place (2 Nov), old
ones kept as `_old`; `Controller.h` no longer stores a `controllerState` member. Link verified with a
temporary `GetState()` call (removed again). All seven modules have header + cpp with the correct
inheritance, every base-class function overridden, private members and helper declarations in place,
and empty bodies. `Main.cpp` launches TMM. `SMObjects.h` has `SM_CrashAvoidance` and the critical
masks; `UGVModule.h` has two extra error codes and full `printError`; `NetworkedModule.h` has the
port numbers and `STUDENT_ID`. Builds clean (0 errors, 0 warnings) and the exe runs and exits.

**Branch `progress-check`:** complete and verified. Built from `main`. Contains:
- `TMM.cpp` fully implemented: creates all SM objects, starts threads in the order Controller, LiDAR,
  CrashAvoidance, VC, Display, GNSS, start-up `Barrier` with timeout (then nulled so restarts skip it),
  heartbeat monitor with per-thread stopwatches, critical failure → full shutdown, non-critical
  failure → restart, `q` key (via `_kbhit`/`_getch`) → routine shutdown, TMM-alive signal
  (its own stopwatch, which every module checks in `getShutdownFlag`).
- `LiDAR.cpp` fully implemented: connect, zID auth, `sRN LMDscandata` request, read-until-ETX,
  parse `DIST1` block (scale, start angle, step, count, hex mm values), convert to metres in the
  vehicle frame (x forward, y left), publish to `SM_Laser`, print 361 x,y pairs every 25th scan.
- The other five modules are **heartbeat-only stubs** (connect/communicate return SUCCESS, loop
  = heartbeat + sleep 100 ms) so TMM has six real threads to start, monitor and shut down.
- `tools/mock_lidar.py` (stand-in LiDAR service, port 23000) and `tools/test_progress_check.py`
  (harness: scan rate, server-loss path, and a `q` keystroke injected into the exe's console).
- `mock_lidar.py --moving` + `tools/test_moving_obstacle.py` (added 3 Nov 2026): a post 1.0 m away
  sweeps from the vehicle's right (10°) to its left (170°) and back, 2° per scan, and the scan
  counter advances. The test checks every printed scan shows the post in the right place. This
  stands in for the simulator, whose LiDAR is frozen (see §6). Result: 10/10 printed scans correct,
  right → y negative, left → y positive, ahead → (1.00, 0.00). **Left/right is confirmed against
  the agreed convention** (0° = right, 90° = ahead, 180° = left).

Verified results (30 Oct 2026, mock server, not the simulator):
| Check | Result |
|---|---|
| (i) class structure | all seven modules, correct inheritance, builds clean |
| (ii) live LiDAR print | 361 pairs, wall at 2 m ahead → (2.00, 0.00) at index 180; post at 45° 0.8 m → (0.57, −0.57) at index 90; ~32 scans/s |
| (iii) `q` shutdown | all six threads stopped and process exited 0.1 s after the keypress |
| critical thread lost | LiDAR server killed → `ERR_CONNECTION` → TMM full shutdown 2.1 s later |
| LiDAR absent at start | LiDAR thread exits after the barrier → TMM full shutdown at 2.3 s |

**Branch `full-build`:** built 17 Nov 2026 from `progress-check`. All six modules implemented; builds
0 errors (3 known lib warnings). Tested against stand-ins and the real MATLAB engine, **not yet on the
Weeder simulator or the robot**. Summary (details and per-component walkthrough in `docs/REVIEW_GUIDE.md`):
- **GNSS:** buffered stream, sync search `AA 44 12`, header checks (len 28, ID 726, body 80), CRC32
  (corrected Appendix A code) over 108 bytes vs the stored CRC, prints N/E/H with both CRCs, prints
  mismatches and discards. Non-critical; reconnects via TMM restart.
- **Controller:** Xbox (`INPUT_TYPE = 0`); speed = (RT − LT) × 1 m/s, steering = −stickX × 40° (+ left),
  10% deadzone, disconnected → 0/0 with a one-off message, **B button** → shutdown request (sets
  `bit_TM`; TMM runs the normal shutdown).
- **VC:** zID auth like the LiDAR, sends `# %.3f %.3f %d #` (steer, speed, watchdog) every 50 ms,
  no newline (both confirmed on the simulator); watchdog toggles each message; caps forward speed
  and steering to crash-avoidance limits; holds the vehicle (speed 0) if those limits are > 300 ms
  old; final stop command on exit; status line twice a second while driving.
- **Display:** mm, engine frame (X = −y, Y = x), provided BlockCopy routine, X, 10 ms, Y, 10 Hz.
- **CrashAvoidance (new design, see §7):** arc-path corridor, second-nearest point, 17 candidate
  steering angles → steering limits, braking-curve speed limit, fail-safe on stale LiDAR (scan
  counter added to `SM_Laser`).
- **SMObjects.h:** `SM_Laser::scanNumber`; `SM_CrashAvoidance` now holds `maxForwardSpeed`,
  `maxLeftSteer`, `maxRightSteer`, `pathClearance`, `lastUpdate` (all start at "stopped").
- **TMM/Main:** heartbeat stopwatches restarted after the barrier; exit code 2 after a critical
  failure. **LiDAR:** stops after 3 consecutive 1 s timeouts; prints every 100 scans.

Test results (`tools/test_full_build.py`, 36/36): straight approach 2.0/0.8/0.5/0.35 m → 1.00/0.76/
0.38/0.00 m/s; reversing allowed; side obstacle front-left + hard left → eased 40°→20° at full
speed, hard right unaffected; mirror case on the right; blocked ahead + hard left → steering free,
speed 0; single stray point ignored; 2 cm pole slows; disconnect → 0/0; watchdog alternates;
display mm/frame correct; GNSS 196 OK + 32 deliberate mismatches rejected, values match the file;
B → exit 0 with 6 threads stopped and a final stop command; display lost → exit 2 naming Display;
GNSS dropped every 15 records → restarted, program keeps running; `q` → exit 0.1 s later.
Real MATLAB engine (R2025a) run with a post 2 m ahead and one front-left: plot correct.

**Weeder simulator run (17 Nov 2026, Aidan's OK; test exe with scripted controller, mock display):**
both logins OK; 482 scans in 21 s (~22/s); 43 GNSS records (2/s), all CRCs OK; forward driving fully
blocked by the frozen scan as predicted (§6); reversing sent at −1.0 m/s; B button → all six threads
stopped, exit 0. Steering sign checked separately with a raw socket: +20° turns left (§6).

**Controller hardware:** Aidan has a PS4 controller, not an Xbox one. The provided library uses
XInput, which a DualShock 4 does not speak natively; it needs DS4Windows (free, uses the ViGEmBus
driver) to appear as an Xbox 360 controller. Then R2/L2 = triggers, right stick = steering, and the
shutdown button B = Circle (○). No code change needed.
**Tested 17 Nov 2026 with Aidan's PS4 controller (USB):** "DS4Windows Nexus" (an unofficial fork Aidan
downloaded first) uses a USB/IP backend (VIIPER) and refused to create the virtual Xbox controller
because Citrix USB Monitor (`ctxusbmon.sys`) is installed. Replaced with standard DS4Windows 3.9.9
(github.com/schmaldeo/DS4Windows, extracted to `C:\Users\aidan\DS4Windows`) + ViGEmBus 1.21 (+ HidHide),
which works; DS4Windows must be running. Live run of the real exe against the simulator: proportional
trigger speeds (e.g. 0.43, 0.88, −0.57 m/s), R2 blocked by crash avoidance (frozen scan), L2 reverse
at −1.0, stick steering ±40 with intermediate values, Circle → clean shutdown, exit 0. Stick left reads
−1.0 in XInput → steering +40 (left), so the sign is right. Not yet checked live: unplugging the
controller mid-run (covered by the scripted test).

**Real robot, 18 Nov 2026 (lab PC, Xbox controller; full notes in `docs/ROBOT_TEST_RESULTS_2026-11-18.md`):**
- **Network:** join the robot's Wi-Fi (PC gets 192.168.1.18/24). The lab PC's wired port is the campus
  network and can't reach 192.168.1.200. The robot ignores ping; test with
  `Test-NetConnection 192.168.1.200 -Port 23000`. The lab PC's own project was an older version, so build
  the full-build zip. Run the exe from a Command Prompt (the window closes on exit).
- **Section 1 (connect): PASS.** All three services connect, LiDAR and VC authenticate with zID 5481380,
  real LiDAR replies parse (361 points), crash avoidance reports data OK.
- **Section 3 (GNSS): PASS.** All CRCs match. The robot replays the same synthetic `GPSData.dat` as the
  simulator (Northing about 6, Easting about 25 rising 0.1 per record, height 108.389), not a live fix.
- **Section 2 (LiDAR/display): partly done.** Plot is live and matches the printed scan (left = −X, mm).
  Still to do with a person: left/right mounting check, 1 m tape check. **Possible self-reading:** points
  302–356 sit at x 0.0–0.3 m, y 0.5–0.65 m (ahead-left, ~0.55–0.6 m) in every scan. These are outside the
  straight-ahead corridor (half-width 0.40 m), so they shouldn't block straight driving, but a hard-left
  arc sweeps through them, so they would likely ease left steering. Aidan says it was most likely a person
  or a chair beside the robot, not the robot itself. Re-check with the area cleared before ruling it out.
- **Sections 4–8:** not started (steering sign, watchdog, B/q, unplug, ground driving, crash avoidance,
  measurements).
- **Bug found (not fixed; discuss with Aidan first):** if the robot is unreachable, LiDAR/VC/GNSS block
  ~20 s in the `TcpClient` constructor before the start barrier. The barrier times out at 15 s, and TMM's
  own stopwatch (`TMM.cpp:21`, restarted only in `processSharedMemory()` in the main loop) is by then
  over `TM_ALIVE_TIMEOUT_MS` (4 s). Controller and CrashAvoidance see "TMM stopped" and exit, and TMM
  reports **"CRITICAL module Controller stopped responding"**, blaming the wrong module. Fix idea: keep
  TMM's alive signal fresh while waiting at the barrier (or restart it right after), give the connect a
  timeout shorter than the barrier (`BeginConnect` + wait), and print which module failed to connect.
  When the network works, startup is fast and this doesn't trigger.

**UGV Obstacle Simulator, 21 Nov 2026** (Python stand-in for the Weeder simulator with movable boxes;
the zip lives in Aidan's Downloads, not in the repo; report in `docs/OBSTACLE_SIM_TEST_2026-11-21.md`).
Its vehicle is 1.2 × 0.70 m, wheelbase 0.8 m, LiDAR at the front, 1.0 m ahead of the rear axle. Results:
14 scenarios, no collision in any LiDAR-visible case, stops 0.30–0.39 m from obstacles, left/right
symmetric, the 1 m gap is passed at full speed, no-echo zeros are ignored, pop-up boxes at 1.5 and 0.6 m
are stopped for. Expected exceptions: a box beside the body (blind spot) is hit when turning into it, and
preset 2/3 (box edge 0.40 m off-centre) stops straight driving because it sits on the 0.40 m safety
corridor edge (design choice, flagged to Aidan). All 256 GNSS CRCs OK, about 32 scans/s, exit 0.

### Build notes (important)

- Build from the command line with the toolset override, because v143 is not installed here:
  ```
  "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" "MTRN3500 Assignment 2.sln" -p:Configuration=Debug -p:Platform=x64 -p:PlatformToolset=v145 -m -v:m -nologo
  ```
  The `.vcxproj` still says v143 on purpose so the student-facing project is untouched.
- Output lands in `Executables\MTRN3500 Assignment 2.exe` (git-ignored).
- **Include-order trap:** `ControllerInterface.h` includes `<Windows.h>`. It must be included
  *before* `UGVModule.h`/`SMObjects.h` (which do `using namespace System;`), or you get
  `error C3699 … IServiceProvider`. `Controller.h` and `TMM.h` are ordered accordingly, with comments.
  Worth a note in the spec; every student will hit this.
- Aidan's rule: **do not launch or unzip the simulator** from Claude. Test networking against the
  Python stand-in servers in `tools/` instead (`mock_lidar.py` reproduces the LiDAR service: auth,
  `sRN LMDscandata`, 361-point `DIST1` telegram). Run the harness from the repo root:
  `python tools/test_progress_check.py` (needs the exe built first).
- **Never run the harness while the simulator is open.** Both use port 23000; the mock can't bind and
  the exe ends up talking to the simulator. The harness now refuses to start if 23000 is taken.
  `--sra` makes the mock reply in datasheet form; default is the simulator's `sRN` form.
- **Test-harness trap:** if you run the exe from Python with `stdout=PIPE` and don't read the pipe
  continuously, the 7 KB scan print fills the pipe, `Console::Write` blocks, the LiDAR heartbeat
  stops, and TMM "correctly" declares it dead. Always drain stdout on a thread (the harness does).
- **Full-build tests** (repo root, after building; never with the simulator or MATLAB open — ports
  23000/24000/25000/28000 must be free):
  `python tools/build_test_exe.py <scratch dir>` then
  `python tools/test_full_build.py "<scratch dir>\Executables\MTRN3500 Assignment 2.exe"`.
  To check the real MATLAB engine non-interactively, run it with `matlab -batch` from a wrapper that
  starts a `timer` to `exportgraphics` the figure (don't probe port 28000 with a bare connect first:
  the engine takes that as its client).
- Scan rate against the mock is ~32/s with the 20 ms sleep in the LiDAR loop. If the real simulator
  shows ~5/s, suspect Nagle + delayed ACK on its side (the mock needed `TCP_NODELAY`).

## 9. Next steps (in order)

1. ~~Skeleton on `main`.~~ Done.
2. ~~`progress-check` branch.~~ Done and verified. Aidan to review it as the Week 8 marking point.
3. ~~`full-build` Parts 3–7 + report.~~ Done 17 Nov 2026 (see §8). Still to do on `full-build`:
   (a) ~~simulator run~~ done 17 Nov 2026 (§8); (a2) ~~PS4 controller~~ done 17 Nov 2026 (§8); (b) Aidan's review using
   `docs/REVIEW_GUIDE.md`; (c) GR-U01 bonus; (d) real-robot check of crash-avoidance geometry
   constants (`CrashAvoidance.h`) and whether the LiDAR sees the robot itself. **18 Nov 2026:**
   sections 1 and 3 passed, 2 partly done (§8); finish 2 and do 4–8, then (e) put the measurements into
   `CrashAvoidance.h` and handle self-readings, (f) fix the unreachable-robot startup bug (§8), both
   with Aidan's OK. Optional: set Linker → System → SubSystem = Console so Ctrl+F5 keeps the window open.
4. Figure 1's "270°" label on the +X axis is still wrong in the spec; the code follows the other
   five labels (0° = right, 90° = forward, 180° = left), confirmed by the moving-post mock test.
   `STUDENT_ID` is `5481380` (z5481380, set 3 Nov 2026; was the placeholder `5555555`).
5. ~~Aidan to run `progress-check` against the real simulator.~~ Done 2 Nov 2026, passed; re-run
   by Claude 3 Nov 2026, passed. Left/right can't be checked on the simulator (its LiDAR is frozen,
   §6); it was checked against the moving-post mock instead. Confirm once on the real robot.
6. ~~Low-priority robustness points~~ — all three fixed in `full-build` (17 Nov 2026). Were: (found in review 3 Nov 2026, not touched in
   `progress-check`): (a) if start-up takes over ~2 s (slow connect to the real robot), the
   heartbeat stopwatches are already stale when TMM's loop starts, which could cause a false
   critical shutdown; restart them after the barrier. (b) The exe exits with code 0 even after a
   critical-failure shutdown. (c) A LiDAR that stays connected but sends nothing produces repeated
   timeouts while the thread still heartbeats, so TMM never notices; count consecutive timeouts.

## 10. Push log (newest first)

- **2026-11-21** — `full-build`: tested against the UGV Obstacle Simulator (§8). Added the report and two
  test scripts. No code changes.

- **2026-11-18** — `full-build`: docs only. Recorded the first real-robot session (§8): network method
  (robot Wi-Fi, no ping), sections 1 and 3 pass, section 2 partly done, possible ahead-left self-reading,
  GNSS synthetic on the robot too, and the unreachable-robot startup bug (not fixed). Added
  `docs/ROBOT_TEST_RESULTS_2026-11-18.md`; checklist ticked and switched to the lab's Xbox controller.
  Follow-up: Aidan thinks the ahead-left readings were a person or a chair; re-check with the area cleared.

- **2026-11-17** — `full-build`: added `docs/ROBOT_TEST_CHECKLIST.md` for Aidan's real-robot test on
  18 Nov 2026 (connect, LiDAR left/right and self-readings, GNSS, wheels-up driving and steering sign,
  watchdog, crash avoidance with a tape measure, robot measurements). Results to feed back into
  `CrashAvoidance.h` constants and §6/§8.

- **2026-11-17** — `full-build`: docs only. Live test with Aidan's PS4 controller via DS4Windows
  3.9.9 + ViGEmBus passed (§8); setup notes added to the review guide. **All 17 Nov work pushed**
  (`main`, `progress-check`, new `full-build`) with plain `git push` (credential manager; `gh` not
  needed on the home PC).

- **2026-11-17** — `full-build` (new branch from `progress-check`, home PC): Parts 3–7 implemented,
  crash avoidance redesigned for the revised wording (graded speed + steering easing), report and
  review guide in `docs/`, new stand-ins and a 36-check test in `tools/` (36/36 pass), real MATLAB
  engine checked (found the display-frame issue, §6). Then run on the Weeder simulator with Aidan's
  permission (results in §8; steering sign confirmed, gauge sign quirk and frozen-scan blocking
  recorded in §6).

- **2026-11-17** — docs only (`progress-check` and `main`). Recorded Aidan's two spec changes: the
  revised crash-avoidance wording (Part 7 and marking item f, §5) and the display millimetre
  clarification (§5 table). Noted in §7 that `full-build` replaces the stop-only flag with speed and
  steering limits. A zip of `main` (`MTRN3500-Exemplar-2-main.zip`, commit `ce0fee0`) is now in the
  home-PC `Ass 2` folder; it matches the local `main`.

- **2026-11-03** — `progress-check` (home PC): comment rewrite and student ID. Aidan found the
  comments over-explanatory and "AI-ish"; all comments in the seven modules, Main and our additions
  to the provided headers are now short and plain (no narration, no restating function names, no
  spec part/appendix numbers, no personal names). Original starter comments untouched. Script check
  confirmed the code (comments and whitespace stripped) is identical in all 18 files. `STUDENT_ID`
  set to `5481380` (z5481380). Builds 0 errors; simulator logged "authenticated 5481380"; moving-post
  test 10/10; original harness 32.2 scans/s, server loss and `q` clean. Style rule for future
  comments: one short line where the code isn't self-explanatory, otherwise none.

- **2026-11-03** — `progress-check` (home PC): tests only, no exe code changed. Claude ran the
  real simulator (Aidan's permission): build 0 errors; 361 points per scan at ~25 scans/s; `q` ->
  all threads stopped, exit 0.1 s later; simulator killed mid-run -> critical LiDAR shutdown 2.2 s
  later; starting straight after the simulator works. Found the simulator's LiDAR is a frozen
  recording (§6). Added `mock_lidar.py --moving` and `tools/test_moving_obstacle.py` (10/10 pass,
  left/right confirmed). Original harness re-run: 32.1 scans/s, server loss and `q` clean.
  Gotcha: a bare TCP connect-and-close to the simulator's port 23000 just before the exe connects
  made the exe's login time out once; don't probe the port while testing against the simulator.

- **2026-11-02** — `progress-check`: comments only. Removed the `// ----` separator lines and rewrote
  comments in very plain language (Aidan's request: no spec section numbers or jargon). Script check
  confirmed no code lines changed. Builds 0 errors; harness 32.1 scans/s, LiDAR loss and `q` both clean.

- **2026-11-02** — `progress-check`: readability pass at Aidan's request, no behaviour change. Spec-fixed
  names untouched. Added two protected helpers to `UGVModule.h` (`waitForStartBarrier()`,
  `tmmHasStopped()`) replacing code copied into all six modules; TMM now has `MODULE_START_ORDER`,
  `moduleName()` (messages say "LiDAR" not "bit 2"), `monitorHeartbeats` renamed `checkHeartbeats`,
  `JOIN_TIMEOUT_MS` renamed `THREAD_STOP_WAIT_MS`; LiDAR parser variables renamed (`dist1Index`,
  `scaleFactor`, `angleStep`, `pointCount`, `distanceMm`...) and magic numbers named (`STX`, `ETX`,
  timeouts, `SCAN_PERIOD_MS`). Builds 0 errors/0 compiler warnings. Harness re-run (mock in simulator
  `sRN` form): 32.1 scans/s, LiDAR loss -> full shutdown 2.1 s later, `q` -> exit 0.1 s, exit code 0;
  index 90 = (0.57, -0.57), index 180 = (2.00, 0.00) as before. Aidan: hold `full-build` for now.

- **2026-11-02** — docs only. **progress-check passed on the real simulator (Aidan's run):** auth OK with
  the placeholder ID `5555555`, 361 points per scan printed continuously, `q` shuts everything down.
  Stationary vehicle: obstacle ~0.78 m dead ahead (indices 156–201), something ~0.16 m off one side
  (indices 272–360), ~5.4 m on the other (index 0), index 271 = no echo. Left/right orientation still to
  be confirmed by Aidan on the simulator (this also settles the Figure 1 "270°" question). Mock harness
  still not re-run since the `sRN` change (simulator was open).

- **2026-11-02** — `progress-check`: Aidan's first real-simulator run. Threads start, LiDAR connects and
  authenticates, `q` shutdown works. Every scan was rejected because the simulator answers `sRN`
  instead of `sRA`; `LiDAR.cpp` now accepts both. Mock defaults to `sRN`; harness refuses to run if
  port 23000 is busy. Builds 0 errors. Harness NOT re-run after this change (simulator was open);
  awaiting Aidan's re-test on the simulator.

- **2026-11-02** — `main` (merged into `progress-check`): crash avoidance made non-critical per Aidan
  (moved to `NONCRITICALMASK`), `SM_CrashAvoidance::inhibit` now starts `true` (fail-safe). Note: the
  two `tools/` files that kept showing as deleted were removed by the PC's security software, not Aidan;
  restore with `git checkout -- tools/` if it happens again.

- **2026-11-02** — `main` (merged into `progress-check`): swapped in Aidan's updated
  `ControllerInterface_x64_Debug.lib` and `ControllerInterface.h` for VS > 17.9, old files renamed
  `_old`. Removed `state_` member from `Controller.h` (C4368 with the native struct). Builds 0 errors;
  3 linker warnings (LNK4075, 2× LNK4099 missing pdb). Temporary `GetState()` call links fine.

- **2026-11-02** — `progress-check` and `main`: docs only. Fixed standing order 6 (no simulator),
  corrected the LiDAR conversion formula in §5, made `main`'s HANDOVER.md match. Re-ran
  `tools/test_progress_check.py` on today's build: 32.1 scans/s, server loss → full shutdown 2.05 s
  later, `q` → exit 0.1 s later, exit code 0. Still open: real `STUDENT_ID` (placeholder `5555555`
  in `include/NetworkedModule.h`), crash-avoidance criticality, Figure 1 label, go-ahead for `full-build`.

- **2026-10-30** — `progress-check` (new branch from `main`): full TMM and LiDAR, five
  heartbeat-only stubs, `tools/` mock server + harness, verified as tabled in §8.

- **2026-10-30** — `main`: skeleton commit. Seven module file pairs, project file updated, provided
  headers extended, builds clean with the v145 override, exe runs. Progress-check item (i) satisfied.

- **2026-10-30** — `main`: added Aidan's verbatim instruction and the originating session name to
  HANDOVER.md. Docs only; skeleton header edits are in progress locally and not yet committed.
- **2026-10-30** — `main`: starter code laid in, HANDOVER.md, CLAUDE.md, README. No code yet.
