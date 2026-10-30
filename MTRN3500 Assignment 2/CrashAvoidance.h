#pragma once
/*
CrashAvoidance.h - Crash Avoidance Module (Part 7)
Combines the LiDAR scan with the commanded speed and steering to decide whether forward
motion must be inhibited, and publishes that decision in SM_CrashAvoidance for the vehicle
control module. Critical thread.
*/

#include <UGVModule.h>

ref class CrashAvoidance : public UGVModule {
public:
    CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA);

    error_state processSharedMemory() override;   // Read scan + commands, write the inhibit decision
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    // Local copies of the inputs
    array<double>^ x_;
    array<double>^ y_;
    double speed_;
    double steering_;

    // Outputs
    bool inhibit_;
    double nearestObstacle_;

    bool collisionLikely();                // The decision logic (described in the one-page report)
    void heartbeat();

    literal double STOP_DISTANCE_M = 1.0;  // Inhibit when an obstacle is within this range ahead
    literal int CHECK_PERIOD_MS = 20;
};
