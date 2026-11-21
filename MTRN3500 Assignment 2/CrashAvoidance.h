#pragma once
/*
CrashAvoidance.h - Crash Avoidance Module
Combines the LiDAR scan with the driver's speed and steering to work out how fast the vehicle
may go forward and how far it may steer each way. Publishes the limits in SM_CrashAvoidance,
which VC applies. Non-critical.
*/

#include <UGVModule.h>

ref class CrashAvoidance : public UGVModule {
public:
    CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA);

    error_state processSharedMemory() override;   // Read scan and command, publish limits
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    array<double>^ x_;
    array<double>^ y_;
    double speed_;                        // Driver's request
    double steering_;
    unsigned int lastScanNumber_;
    Stopwatch^ scanAge_;                  // Time since the scan last changed
    bool scanStale_;

    double maxForwardSpeed_;              // Outputs
    double maxLeftSteer_;
    double maxRightSteer_;
    double pathClearance_;

    error_state readInputs();
    void computeLimits();
    void publishLimits();
    double pathClearance(double steerDeg);                                // Free distance along a path
    double distanceAlongPath(double px, double py, double steerDeg);      // -1 if the point is off the path
    double speedLimitFor(double clearance);
    void heartbeat();

    // Vehicle geometry, matched to the UGV Obstacle Simulator (body 1.2 m x 0.70 m, wheelbase 0.8 m,
    // LiDAR at the front centre). Re-measure on the real robot.
    literal double VEHICLE_HALF_WIDTH_M = 0.35;
    literal double SIDE_MARGIN_M = 0.10;
    literal double WHEELBASE_M = 0.80;
    literal double LIDAR_AHEAD_OF_REAR_AXLE_M = 1.00;   // The vehicle turns about a point level with the rear axle

    literal double CAUTION_DISTANCE_M = 1.0;      // Start slowing / easing the steering
    literal double STOP_DISTANCE_M = 0.30;        // Forward motion stops here
    literal double LOOKAHEAD_M = 2.0;             // Paths are only checked this far
    literal double REACTION_TIME_S = 0.10;        // Scan age + send period
    literal double MIN_VALID_RANGE_M = 0.05;      // A range of 0 means no echo
    literal double STEER_STEP_DEG = 5.0;
    literal double MAX_STEER_DEG = 40.0;
    literal double MAX_SPEED_MPS = 1.0;
    literal int SCAN_STALE_MS = 200;
    literal int CHECK_PERIOD_MS = 20;
};
