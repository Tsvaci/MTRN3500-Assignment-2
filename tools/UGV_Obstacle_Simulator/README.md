# UGV Obstacle Simulator (MTRN3500 Assignment 2)

An unofficial stand-in for `Weeder_Simulator.exe` that lets you **place obstacles while you drive**,
so you can test crash avoidance at home. It is a test tool only — it contains no assignment code.

Your program connects exactly as it does to the normal simulator (`WEEDER_ADDRESS` = `127.0.0.1`,
LiDAR 23000, GNSS 24000, Vehicle Control 25000, same zID authentication — any zID is accepted).
The LiDAR scans it returns include your obstacles, so they go through your LiDAR parser, shared
memory and Display module, and appear on the MATLAB plot.

## Requirements
- Windows with **Python 3** from python.org (tested on 3.12; the default install includes tkinter).
  Nothing else to install.

## How to run
1. **Close `Weeder_Simulator.exe`** — both use the same ports and can't run together
   (you'll get a "Port 23000 is in use" message if it's still open).
2. Double-click **`run_obstacle_sim.bat`**.
3. Start MATLAB `runner.m`, then your Visual Studio program, as usual.

## Controls
| Input | Action |
|---|---|
| Left-drag a box | move it |
| Left-drag a box's white corner | resize it |
| Right-click a box | delete it |
| `N` | new 0.4 m box at the mouse |
| `Del` | delete the box under the mouse |
| `1` | test: wall 1.5 m straight ahead |
| `2` / `3` | test: box ahead-left / ahead-right (for turning into it) |
| `4` | test: 1 m gap between two boxes, 2 m ahead |
| `R` | reset the UGV to the start |
| `W` | walls on/off |

## What you see
- **Blue UGV** (1.2 m × 0.7 m), yellow dot = LiDAR at the front-centre, dashed orange arc = 1 m.
- LiDAR hit points: light blue, **red when within 1 m**.
- Side panel: the command your program is sending (steer, speed), actual speed, whether your
  watchdog is toggling, connection counts, and **"Nearest ahead"** — closest LiDAR point in the
  vehicle-width corridor straight ahead.
- Status: `OK`, `NO VC CONNECTION`, `INHIBITED (wdog)`, or **`COLLISION`** (red UGV + banner) —
  a collision means crash avoidance didn't stop the UGV in time. Press `R` to reset.

## How it behaves
- LiDAR: 361 points, 0–180° in 0.5° steps (0° = vehicle's right, 90° = ahead, 180° = left),
  ranges in mm, max 8 m, ±5 mm noise. **No echo = 0** — press `W` (walls off) to check your
  code ignores those instead of treating them as an obstacle at 0 m.
- Commands are clamped to ±40° and ±1 m/s; speed ramps at 2 m/s².
- **Inhibiting motion:** sending speed 0 works, and so does holding `wdog` at a fixed value —
  if `wdog` stops alternating for 0.3 s (or Vehicle Control disconnects) the UGV stops.
- GNSS replays the same data file as the original simulator (`GPSData.dat`).

## Adjusting it
Constants at the top of `sim_core.py`:
- `STEER_SIGN = +1` — positive steer turns **left**. Set to `-1` if your UGV turns the other way.
- `BODY_LEN`, `BODY_WID`, `WHEELBASE`, `LIDAR_MAX`, `ROOM`, …

## Files
`obstacle_sim.py` (window, start here) · `sim_core.py` (vehicle, LiDAR, presets) ·
`servers.py` (TCP ports) · `protocols.py` (message formats) · `GPSData.dat` (GNSS data) ·
`run_obstacle_sim.bat` (launcher)
