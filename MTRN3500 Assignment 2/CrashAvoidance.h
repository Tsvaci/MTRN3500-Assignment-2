#pragma once
/*
CrashAvoidance.h - Crash Avoidance Module
Uses the scan and the commanded speed/steering to set the inhibit flag in SM_CrashAvoidance.
Non-critical.
*/

#include <UGVModule.h>

ref class CrashAvoidance : public UGVModule {
public:
    CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA);

    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    array<double>^ x_;
    array<double>^ y_;
    double speed_;
    double steering_;

    bool inhibit_;
    double nearestObstacle_;

    bool collisionLikely();
    void heartbeat();

    literal double STOP_DISTANCE_M = 1.0;
    literal int CHECK_PERIOD_MS = 20;
};
