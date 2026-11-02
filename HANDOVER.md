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
   Aidan runs the real simulator himself.
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

Planned additions (exact names required by the spec, two files per module):
`LiDAR`, `GNSS`, `Controller`, `VC`, `Display`, `CrashAvoidance`, and bonus `GRU01`.
Plus `docs/crash_avoidance_report.md` (the one-page theory report).

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
| LiDAR | weeder | 23000 | yes | send `<zID digits>\n` (8 chars), expect `OK\n`. Then `0x02 sRN LMDscandata 0x03`. Reply is `sRA LMDscandata …` ASCII hex fields; preconfigured 50 Hz, 0–180°, 0.5° step → 361 `DIST1` points in mm. |
| GNSS | weeder | 24000 | no | continuous Novatel binary. Sync `AA 44 12`, header len at byte 3 (=28), msg ID at 4 (726 = BESTUTM), body len at 8 (=80). Northing double at H+16, Easting H+24, Height H+32, CRC32 at H+80 over header+body. `GPSData.dat` in the simulator folder is this exact stream. |
| Vehicle control | weeder | 25000 | yes | send `# <steer> <speed> <wdog> #`, steer ±40°, speed ±1 m/s, wdog alternates 0/1; freeze it to inhibit motion. |
| Display | 127.0.0.1 | 28000 | no | MATLAB server. Send X array as bytes, sleep 10 ms, then Y array (Appendix B). |
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

Verified results (30 Oct 2026, mock server, not the simulator):
| Check | Result |
|---|---|
| (i) class structure | all seven modules, correct inheritance, builds clean |
| (ii) live LiDAR print | 361 pairs, wall at 2 m ahead → (2.00, 0.00) at index 180; post at 45° 0.8 m → (0.57, −0.57) at index 90; ~32 scans/s |
| (iii) `q` shutdown | all six threads stopped and process exited 0.1 s after the keypress |
| critical thread lost | LiDAR server killed → `ERR_CONNECTION` → TMM full shutdown 2.1 s later |
| LiDAR absent at start | LiDAR thread exits after the barrier → TMM full shutdown at 2.3 s |

**Branch `full-build`:** not created yet.

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
- **Test-harness trap:** if you run the exe from Python with `stdout=PIPE` and don't read the pipe
  continuously, the 7 KB scan print fills the pipe, `Console::Write` blocks, the LiDAR heartbeat
  stops, and TMM "correctly" declares it dead. Always drain stdout on a thread (the harness does).
- Scan rate against the mock is ~32/s with the 20 ms sleep in the LiDAR loop. If the real simulator
  shows ~5/s, suspect Nagle + delayed ACK on its side (the mock needed `TCP_NODELAY`).

## 9. Next steps (in order)

1. ~~Skeleton on `main`.~~ Done.
2. ~~`progress-check` branch.~~ Done and verified. Aidan to review it as the Week 8 marking point.
3. `full-build` branch, branched from `progress-check` (so TMM and LiDAR carry over): Parts 3–7
   (GNSS decode + CRC, Controller, VC command stream, Display to MATLAB, CrashAvoidance), the
   one-page report in `docs/`, then the GR-U01 bonus. Add mock servers for GNSS (replay
   `GPSData.dat`), VC and display to `tools/` as needed. Crash avoidance is non-critical
   (restartable), decided by Aidan 2 Nov 2026.
4. Open questions for Aidan: Figure 1's "270°" label sits on the +X axis, which contradicts the
   other five labels (they imply 0° = right, 90° = forward, 180° = left). The code follows the
   five; confirm or correct the figure. Also: which student ID to put in `STUDENT_ID`
   (currently the placeholder `5555555`; the mock accepts any 7 digits, the simulator may not).
5. Aidan to run `progress-check` once against the real simulator himself (never done by Claude).

## 10. Push log (newest first)

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
