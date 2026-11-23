#pragma once
/*
Controller.h - Controller Module
Converts Xbox controller input to speed and steering in SM_VehicleControl. The B button
requests shutdown. Critical.
*/

// Must come before UGVModule.h (C3699 otherwise).
#include <ControllerInterface.h>
#include <UGVModule.h>

ref class Controller : public UGVModule {
public:
    Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);
    ~Controller();

    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_VehicleControl^ SM_VC_;

    // Native class, so held by pointer. controllerState is native too and can't be a member (C4368).
    ControllerInterface* controllerInterface_;

    double speed_;                         // m/s, + forward
    double steering_;                      // deg, + left
    bool wasConnected_;

    error_state readController();                  // Fills speed_ and steering_
    static double applyDeadzone(double value);     // Small readings -> 0, otherwise clamp to -1..1
    void requestShutdown();                        // B button
    void heartbeat();

    literal int INPUT_TYPE = 0;            // 0 = Xbox controller, 1 = keyboard
    literal int PLAYER_NUMBER = 1;
    literal double MAX_STEER_DEG = 40.0;
    literal double MAX_SPEED_MPS = 1.0;
    literal double DEADZONE = 0.1;         // Ignore small stick/trigger readings
    literal int POLL_PERIOD_MS = 20;
};
