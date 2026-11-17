# Crash Avoidance: Proposed Solution

**Aim.** Stop the UGV hitting obstacles in front of it or beside it when it turns, while still letting the driver
manoeuvre. The LiDAR only covers 0 to 180° in front of the vehicle, so only forward and turning motion is
protected. Reversing is left to the driver.

**How a collision can happen.** (1) Driving straight at an obstacle. (2) Turning into an obstacle that is beside
the vehicle but not in front of it. The front corner sweeps sideways, so "nothing ahead" is not enough.
(3) Going too fast to stop in the space left. (4) Acting on old data: a stale scan, or a crash-avoidance thread that
has died.

**Inputs.** The latest 361-point scan (x forward, y left, metres) from `SM_Laser`, and the driver's requested speed
and steering from `SM_VehicleControl`.

**Method (every 20 ms).**
1. *Predict the path.* For a steering angle δ the vehicle follows an arc of radius R = wheelbase / tan δ (a straight
   line when δ = 0). It sweeps a corridor the width of the vehicle plus a 0.1 m margin each side.
2. *Measure clearance.* For each scan point inside the corridor, work out the distance along the arc to reach it.
   The clearance is the **second-nearest** such distance, so one stray reading cannot stop the vehicle. A 2 cm pole
   at 1 m still returns several points. Ranges of 0 (no echo) are ignored. Clearance is checked up to 2 m.
3. *Ease the steering.* Clearance is computed for 17 paths (−40° to +40° in 5° steps). Starting from straight,
   each side may steer up to the sharpest angle whose path is clear for at least **1 m**. A driver who steers
   harder into an obstacle beside the vehicle has the turn eased back to the sharpest safe angle, instead of the
   vehicle simply stopping. If straight ahead is already blocked, steering is left free, because steering away
   may be the only way out. Step 4 still limits the speed on whichever path the driver picks.
4. *Limit the speed.* Using the clearance d on the path actually being driven:
   - v = v<sub>max</sub>·√((d − d<sub>react</sub> − 0.3)/0.7), capped at v<sub>max</sub>, and zero if negative.
     This is the speed from which the vehicle can brake evenly to a stop 0.3 m short of the obstacle.
   - d<sub>react</sub> = requested speed × 0.1 s, which covers scan age and the 50 ms command period.
   - So: full speed with about 1 m clear, zero at 0.3 m or less, and a smooth slow-down in between. A faster
     approach starts slowing earlier.
5. *Publish limits, not commands.* Crash avoidance writes the maximum forward speed, maximum left and right steering,
   and a timestamp to a new `SM_CrashAvoidance` object. Vehicle Control caps the driver's latest command to these
   limits just before each send. The driver stays in control within the safe envelope.

**Responses.**

| Situation | Response |
|---|---|
| Obstacle ahead, more than 1 m | None |
| Obstacle ahead, 0.3–1 m | Speed reduced smoothly as it gets closer |
| Obstacle ahead, 0.3 m or less | Forward motion stopped; reversing and steering still allowed |
| Turning towards an obstacle beside the vehicle | Turn eased to the sharpest clear angle; slowed if no angle is clear |
| Turning away from it | None |

**Robustness and fail-safe behaviour.**
- The limits start at "stopped", so nothing moves until a scan has been checked.
- If the scan counter in `SM_Laser` stops advancing for 200 ms, crash avoidance sets the forward speed limit
  to zero.
- If crash avoidance stops updating for 300 ms (thread crashed or being restarted), Vehicle Control holds the
  vehicle still. Crash avoidance is therefore safe to treat as non-critical: the Thread Management Module restarts
  it.
- Controller disconnected: speed and steering go to zero. On shutdown a final stop command is sent.

**Limitations.** No coverage behind the vehicle. Obstacles thinner than the beam spacing at long range can be missed
(about 9 mm at 1 m). Vehicle width and wheelbase are assumed (0.6 m and 0.5 m) and must be measured on the real
robot. If the LiDAR sees part of the robot itself, those angles must be masked out.
