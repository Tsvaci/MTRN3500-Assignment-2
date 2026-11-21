# Obstacle Simulator test, 21 Nov 2026

The exemplar (`full-build`) was run against `UGV_Obstacle_Simulator` (the Python stand-in for
`Weeder_Simulator.exe` with movable boxes, from Aidan's Downloads). The simulator was run headless, using
its own servers and its own physics step. The controller was scripted with the usual test exe from
`tools/build_test_exe.py`, and the display was the mock on port 28000. Scripts:
`tools/run_obstacle_sim_scenarios.py` and `tools/summarise_obstacle_sim.py`.

**Simulator vehicle:** 1.2 m long, 0.70 m wide, wheelbase 0.8 m, LiDAR at the front-centre (1.0 m ahead of
the rear axle). Speed ramps at 2 m/s². "Gap" below means the shortest distance from the vehicle's body to any
obstacle, measured by the simulator's geometry.

## Overall

- **No collisions in any situation the LiDAR can see** (13 scenarios). Every approach ended with the body
  0.30–0.39 m from the obstacle (target 0.30 m).
- Plumbing all correct:
  - LiDAR parsed the simulator's telegrams: 3,280 scans in 102 s, about 32 per second.
  - All 256 GNSS records had matching CRCs.
  - The watchdog toggled throughout.
  - The display received 821 frames.
  - The B button gave a clean shutdown with exit code 0.
- The one collision was a box **beside** the body, where the LiDAR can't see. That is expected (H1 below).
- One over-cautious result: a box 5 cm clear of the straight path still stopped the vehicle (B1/C1 below).

## Results

| # | Situation | What the weeder did | Expected? |
|---|---|---|---|
| A1 | Wall 1.5 m ahead, full throttle | Full speed, then slowed from about 1 m (0.73 m/s at 0.7 m, 0.31 at 0.44 m), stopped with a 0.39 m gap | Yes |
| A2 | Reverse from A1's stop | Reversed at the full −1.0 m/s, no limit | Yes (reverse isn't limited, per the spec) |
| A3 | Wall ahead, half throttle | Kept 0.5 m/s until about 0.6 m, stopped with a 0.34 m gap | Yes (slower means it brakes later) |
| B1 | Box ahead-left, drive straight | **Stopped 0.35 m short**, although driving straight would have missed by 5 cm | Per design, see note 1 |
| B2 | Box ahead-left, hard left into it | Turn eased 40°→25° at first. Once the box was within 1 m it slowed to 0.25 m/s, rounded the box (closest 0.35 m), sped up, did a U-turn and stopped 0.53 m from the wall | Yes |
| B3 | Box ahead-left, hard right away | Unlimited, full speed; stopped 0.53 m from the wall after the U-turn | Yes |
| C1–C3 | Mirror of B1–B3 on the right | Same numbers to within 1 cm (0.35 / 0.35 / 0.54 m) | Yes, left and right are symmetric |
| D1 | 1 m gap between two boxes, then far wall | Passed through the gap at full speed with 0.15 m each side, stopped with 0.39 m to the far wall | Yes (0.8 m safety corridor fits a 1.0 m gap) |
| E1 | Walls off (no echo, so ranges read 0) | Full speed, no false stop | Yes (ranges under 5 cm are ignored) |
| F1 | Box appears 1.5 m ahead at 1 m/s | Slowed straight away, stopped with a 0.39 m gap | Yes |
| F2 | Box appears 0.6 m ahead at 1 m/s | Speed cut at once, stopped with a 0.30 m gap | Yes (better than expected for this distance) |
| G1 | Wall ahead, steering 20° left | Steering left alone, speed reduced, stopped with a 0.44 m gap | Yes (turning doesn't help with a full-width wall, so only speed is cut) |
| H1 | Box 10 cm beside the body, hard left | **Collision** after 0.45 s | Yes, an inherent limit; see note 2 |

## Notes for Aidan

1. **Over-caution with a 5 cm squeeze (B1/C1).** Preset 2's box starts 0.40 m left of the centreline. The
   exemplar's safety corridor is 0.40 m each side (assumed half-width 0.30 m plus a 0.10 m margin), so the box
   sits exactly on its edge. With the simulator's ±5 mm noise, some points fall inside, and crash avoidance
   treats the straight path as blocked. The simulator's vehicle is 0.70 m wide, so it would actually pass
   with 5 cm to spare.
   - I'd keep the behaviour: refusing a 5 cm squeeze on a real robot is reasonable, and the report says the
     margin is deliberate.
   - The checklist line "drive straight past a box beside the path: no change" only holds when the box is
     more than about 0.45 m out from the centreline. If the preset's author meant it as a clean pass, the
     expected result for preset 2 is "stops", not "no change".
2. **Blind spot (H1).** The LiDAR is at the front and sees 180° forward. Anything beside or behind the front
   edge is invisible, so turning into it can't be detected by any LiDAR-only design. Nothing in the spec asks
   for this. It may be worth a sentence in the spec so students don't lose marks over it.
3. **Geometry assumptions.** The simulator's vehicle differs from the exemplar's guesses in
   `CrashAvoidance.h`:

   | | Simulator | Exemplar guess |
   |---|---|---|
   | Width | 0.70 m | 0.60 m |
   | Wheelbase | 0.8 m | 0.5 m |
   | LiDAR position | 1.0 m ahead of the rear axle | at the turning point |

   Because of this, the exemplar thinks the vehicle turns more tightly than it really does. It caused no
   collision here, but it's another reason to put the real robot's measurements in (checklist section 7).
   If the simulator's figures came from the real weeder, they could be used now.
4. **Stopping gap.**
   - The obstacle's distance is taken from the LiDAR at the front edge, which matches where this simulator
     puts it.
   - At full speed the vehicle stops about 9 cm earlier than the 0.30 m target, because the slowdown curve
     reaches zero while the last 0.1 s reaction allowance is still counted.
   - At low speed it stops closer to 0.30 m.

## How to repeat

Ports 23000–25000 and 28000 must be free (no Weeder simulator, MATLAB or Obstacle Simulator window open).

```
python tools/build_test_exe.py <scratch>\obtest
python tools/run_obstacle_sim_scenarios.py <sim folder> "<scratch>\obtest\Executables\MTRN3500 Assignment 2.exe" . <scratch>\obsim_out
python tools/summarise_obstacle_sim.py <scratch>\obsim_out\run.json
```

Interactive use: double-click `run_obstacle_sim.bat` in the simulator folder, start MATLAB `runner.m`, then the
normal exe with the controller.
