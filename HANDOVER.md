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
6. **Verify, don't assume.** The machine has Visual Studio 2026 Community and MATLAB R2025b, and the
   simulator is in the `Ass 2` folder. Build and run against the simulator before reporting done.
7. Commit messages end with `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.

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
  libs/                             four ControllerInterface_*.lib files, linked by the project
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
Non-critical: GNSS. Crash avoidance: draft doesn't say; **Aidan to decide** (default: critical).
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

**LiDAR angle convention (service details Fig. 1).** 0° points to the right-hand side (−Y of the
device frame), 90° straight ahead (+X? — confirm against the figure), 180° to the left. 361 points at
0.5° steps. Convert: x = r·cos θ, y = r·sin θ, with θ offset so that forward is consistent for the
vehicle base frame. The display must be in the vehicle base frame.

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
  the watchdog.

## 8. Current status

**Branch `main`:** starter code committed as received (minus build junk), plus this file, CLAUDE.md
and README. No module code written yet. Nothing has been built on this machine yet.

**Branch `progress-check`:** not created yet.
**Branch `full-build`:** not created yet.

## 9. Next steps (in order)

1. On `main`: skeleton commit — all seven module header/cpp pairs with correct inheritance and
   overridden functions, empty bodies, registered in the `.vcxproj`; `Main.cpp` launching TMM;
   `SM_CrashAvoidance` added to SMObjects.h; confirm it builds with MSBuild. Push, update this file.
2. Create `progress-check` from `main`: TMM shared-memory setup, thread start, heartbeat monitor,
   `q`-key routine shutdown; LiDAR connect + auth + `sRN LMDscandata` + parse + Cartesian conversion +
   readable terminal print. Test against the simulator. Push, update this file.
3. Create `full-build` from `main` (or from `progress-check`, Aidan to confirm): the rest.

## 10. Push log (newest first)

- **2026-10-30** — `main`: added Aidan's verbatim instruction and the originating session name to
  HANDOVER.md. Docs only; skeleton header edits are in progress locally and not yet committed.
- **2026-10-30** — `main`: starter code laid in, HANDOVER.md, CLAUDE.md, README. No code yet.
