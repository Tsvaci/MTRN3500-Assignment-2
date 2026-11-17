# Real-robot test checklist

Work through this in order. The first three sections don't move the robot. Each item settles something the
simulator couldn't show. Note anything that behaves differently, plus the measurements in section 7.

## Before you go

- [ ] In `include\NetworkedModule.h`, swap which `WEEDER_ADDRESS` line is commented out, so
      `192.168.1.200` is active. Then rebuild.
- [ ] Bring: the PS4 controller and cable, a tape measure, and a box or bin to use as an obstacle.
- [ ] Laptop on the robot's network, **DS4Windows open**, then MATLAB `runner.m`, then start the program.

## 1. It connects (robot stationary)

- [ ] LiDAR and vehicle control both print "authenticated", and all six threads start.
- [ ] If LiDAR scans are rejected, write down the exact error. The real sensor may reply in its datasheet
      format rather than the simulator's; the code accepts both, but check.

## 2. LiDAR and display (robot stationary)

- [ ] **Live data:** walk around in front of the robot, and the MATLAB plot moves with you.
- [ ] **Left and right:** stand on the robot's **left**, and you appear on the **left** of the plot.
      If it's the other side, report it. This also settles the wrong "270°" label in the spec's figure.
- [ ] **Distance:** stand 1 m in front (tape-measure it), and the plot shows about 1000 mm ahead.
- [ ] **Self-readings:** with nothing near the robot, look for points closer than about 0.5 m. If there are
      some, the LiDAR is seeing part of the robot itself. Note where (left, right or front).

## 3. GNSS (robot stationary)

- [ ] Lines print with matching CRCs ("CRC calc … recv … OK").
- [ ] Real coordinates look like Northing ≈ 6,2xx,xxx and Easting ≈ 3xx,xxx (zone 56). Indoors there may be
      no satellite fix, giving zeros or stale values. That's the receiver, not the code; try near a window or
      outside.

## 4. Driving with the wheels off the ground

- [ ] R2 spins the wheels forward, L2 in reverse.
- [ ] **Right stick left turns the wheels left.** This is the most important check, because the simulator's
      gauge shows the opposite sign.
- [ ] **Watchdog:** hold R2 and close the program window abruptly. The wheels stop by themselves.
- [ ] Circle shuts everything down, and so does `q` in the program window.
- [ ] Hold R2 and unplug the controller cable. The wheels stop and "DISCONNECTED" prints.

## 5. Driving on the ground

Clear area, with someone ready at the robot's emergency stop.

- [ ] Half-press R2 first: speed follows how far you press. Then full speed, and steering both ways.

## 6. Crash avoidance (marking item f, 6 marks)

Watch the program window: "LIMITED (clear path …)" prints whenever crash avoidance steps in.

- [ ] **Obstacle ahead:** box about 3 m ahead, drive straight at it with R2 fully held. The robot slows from
      about 1 m and stops about 0.3 m short.
- [ ] Tape-measure the actual stopping gap: ______ m
- [ ] L2 still reverses away from the box.
- [ ] **Obstacle to the side:** box ahead-left, about 1 m out to the side, so driving straight would miss it.
  - [ ] Drive straight: no change.
  - [ ] Turn hard left towards it: the turn is eased.
  - [ ] Turn hard right: no change.
- [ ] Same test on the right-hand side.
- [ ] A person walking in front while driving: the robot slows or stops.

## 7. Measurements to bring back

Crash avoidance currently uses guessed values (`CrashAvoidance.h`) for these:

- [ ] Robot width, wheel to wheel at the widest point: ______ m (guessed 0.60)
- [ ] Wheelbase, front axle to rear axle: ______ m (guessed 0.50)
- [ ] LiDAR position: ______ m behind the front edge, centred? ______ (assumed: at the front, centred)

## 8. Failure handling (optional)

- [ ] Close MATLAB mid-run: the program shuts down and names the Display.
- [ ] Unplug the LiDAR network cable: the program shuts down within a few seconds.

## Bring back

- [ ] Anything that behaved differently from the above
- [ ] The stopping gap and the three measurements
- [ ] A photo of the MATLAB plot with you standing to one side of the robot
