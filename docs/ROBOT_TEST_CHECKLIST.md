# Real-robot test checklist

Work through this in order. The first three sections don't move the robot. Each item settles something the
simulator couldn't show. Note anything that behaves differently, plus the measurements in section 7.

**Progress (18 Nov 2026 lab session):** sections 1 and 3 passed; section 2 partly done; 4–8 not started.
Full notes in `docs/ROBOT_TEST_RESULTS_2026-11-18.md`.

## Before you go

- [x] In `include\NetworkedModule.h`, swap which `WEEDER_ADDRESS` line is commented out, so
      `192.168.1.200` is active. Then rebuild. *(Done on the lab PC copy only. Build the full-build zip,
      not the older project already in `C:\Users\z5481380\source\repos`.)*
- [ ] Bring: a controller and cable, a tape measure, and a box or bin to use as an obstacle.
      *The lab has an Xbox controller, which needs no DS4Windows: RT = forward, LT = reverse, B = shutdown.
      Wherever this list says R2/L2/Circle, read RT/LT/B.*
- [ ] **Network:** join the **robot's Wi-Fi**. The PC gets 192.168.1.x. The lab PC's wired "Ethernet" is
      the campus network and can't reach the robot, so unplug it. That makes `H:\` unavailable; work from `C:`.
- [ ] Check the link with `powershell -Command "Test-NetConnection 192.168.1.200 -Port 23000"`
      (`TcpTestSucceeded : True`). **The robot doesn't answer ping**, so ping is no use as a test.
- [ ] The robot is shared and probably accepts only one client at a time, so make sure nobody else is connected.
- [ ] Start MATLAB `runner.m` first, then run the exe **from a Command Prompt** in `Executables\` so the
      output stays on screen (the window closes on exit otherwise).

## 1. It connects (robot stationary): PASS

- [x] LiDAR and vehicle control both print "authenticated", and all six threads start.
- [x] If LiDAR scans are rejected, write down the exact error. *(None: 361 points per scan, real replies parse fine.)*
- If startup hangs about 20 s and then blames the **Controller**, the robot isn't reachable. That's a known
  misleading message (see the results file); fix the network, not the controller.

## 2. LiDAR and display (robot stationary): PARTLY DONE

- [x] Plot is live and matches the printed scan (four regions checked, mm, left = −X).
- [ ] **Live data:** walk around in front of the robot, and the MATLAB plot moves with you.
- [ ] **Left and right:** stand on the robot's **left**, and you appear on the **left** of the plot.
      If it's the other side, report it. This also settles the wrong "270°" label in the spec's figure.
- [ ] **Distance:** stand 1 m in front (tape-measure it), and the plot shows about 1000 mm ahead.
- [ ] **Self-readings:** on 18 Nov, points 302–356 sat about 0.55–0.6 m away, ahead-left. Aidan thinks that
      was a person or a chair, not the robot. To confirm, clear the area within 1 m and check that nothing
      under 0.5 m shows up. Self-readings found: ______

## 3. GNSS (robot stationary): PASS

- [x] Lines print with matching CRCs ("CRC calc … recv … OK").
- [x] ~~Real coordinates look like Northing ≈ 6,2xx,xxx~~. The robot replays the same synthetic
      `GPSData.dat` as the simulator (Northing about 6, Easting about 25 rising by 0.1 per record, height
      108.389). Small numbers are correct here.

## 4. Driving with the wheels off the ground

- [ ] RT spins the wheels forward, LT in reverse.
- [ ] **Right stick left turns the wheels left.** This is the most important check, because the simulator's
      gauge shows the opposite sign.
- [ ] **Watchdog:** hold RT and close the program window abruptly. The wheels stop by themselves.
- [ ] B shuts everything down, and so does `q` in the program window.
- [ ] Hold RT and unplug the controller cable. The wheels stop and "DISCONNECTED" prints.

## 5. Driving on the ground

Clear area, with someone ready at the robot's emergency stop.

- [ ] Half-press RT first: speed follows how far you press. Then full speed, and steering both ways.

## 6. Crash avoidance (marking item f, 6 marks)

Watch the program window: "LIMITED (clear path …)" prints whenever crash avoidance steps in.

- [ ] **Obstacle ahead:** box about 3 m ahead, drive straight at it with RT fully held. The robot slows from
      about 1 m and stops about 0.3 m short.
- [ ] Tape-measure the actual stopping gap: ______ m
- [ ] LT still reverses away from the box.
- [ ] **Obstacle to the side:** box ahead-left, about 1 m out to the side, so driving straight would miss it.
  - [ ] Drive straight: no change.
  - [ ] Turn hard left towards it: the turn is eased.
  - [ ] Turn hard right: no change.
- [ ] Same test on the right-hand side.
- [ ] A person walking in front while driving: the robot slows or stops.
- If the robot refuses to drive forward or turn left even in open space, the self-readings from section 2
  are the likely cause. Note it and carry on with the other checks.

## 7. Measurements to bring back

Crash avoidance currently uses the UGV Obstacle Simulator's sizes (`CrashAvoidance.h`) for these:

- [ ] Robot width, wheel to wheel at the widest point: ______ m (currently 0.70)
- [ ] Wheelbase, front axle to rear axle: ______ m (currently 0.80)
- [ ] LiDAR position: ______ m behind the front edge, centred? ______ (currently: at the front, centred, 1.0 m ahead of the rear axle)

## 8. Failure handling (optional)

- [ ] Close MATLAB mid-run: the program shuts down and names the Display.
- [ ] Disconnect from the robot's Wi-Fi mid-run: the program shuts down within a few seconds.

## Bring back

- [ ] Anything that behaved differently from the above
- [ ] Whether anything under 0.5 m shows up with the area cleared
- [ ] The stopping gap and the three measurements
- [ ] A photo of the MATLAB plot with you standing to one side of the robot
