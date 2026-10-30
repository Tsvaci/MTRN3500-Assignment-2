# MTRN3500 Exemplar 2

Exemplar solution for the proposed MTRN3500 Assignment 2 (UGV / weeder tele-operation), written to
show that the assignment is achievable by students.

## Branches

| Branch | Purpose |
|---|---|
| `main` | Starter code as provided, the class skeleton for all modules, and the docs. |
| `progress-check` | The Week 8 progress-check deliverable (class structure, live LiDAR print, `q`-key shutdown). |
| `full-build` | The complete Week 11 final-demonstration solution. |

## Building

Open `MTRN3500 Assignment 2.sln` in Visual Studio 2022 or 2026 and build `Debug | x64`.
The output goes to `Executables\`. Headers are in `include\`, the controller library in `libs\`.

## Running against the simulator

1. Start `Weeder_Simulator.exe` (from the assignment's `WeederSimulator` folder). It serves the LiDAR,
   GNSS and vehicle-control services on `127.0.0.1`.
2. For the display, run `MATLAB Display Engine\runner.m` in MATLAB. It listens on `127.0.0.1:28000`.
3. Run the built executable. Press `q` in its console to shut every thread down.

`WEEDER_ADDRESS` in `include\NetworkedModule.h` switches between the simulator (`127.0.0.1`) and the
real robot (`192.168.1.200`).

## For Claude Code sessions

See `HANDOVER.md`.
