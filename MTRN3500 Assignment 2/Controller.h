#pragma once
/*
Controller.h - Controller Module (Part 4)
Reads the Xbox controller through ControllerInterface, converts triggers and right stick to
speed and steering commands in SM_VehicleControl, and raises a shutdown request on a button
press. Critical thread.
*/

// This include must come before UGVModule.h. In the other order the build fails (error C3699).
#include <ControllerInterface.h>
#include <UGVModule.h>

ref class Controller : public UGVModule {
public:
    Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);
    ~Controller();

    error_state processSharedMemory() override;   // Write speed and steering to SM_VehicleControl
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_VehicleControl^ SM_VC_;

    // The controller library is plain C++, so it has to be kept as a pointer here.
    // For the same reason the controller's state can't be stored in this class;
    // it is read fresh each time instead.
    ControllerInterface* controller_;

    // Latest commands derived from the controller state
    double speed_;
    double steering_;

    error_state readController();          // Poll the controller and compute speed/steering
    void requestShutdown();                // Ask TMM to shut everything down (shutdown button)
    void heartbeat();

    literal int INPUT_TYPE = 1;            // 0 = Xbox controller, 1 = keyboard (for testing without a controller)
    literal int PLAYER_NUMBER = 1;
    literal double MAX_STEER_DEG = 40.0;
    literal double MAX_SPEED_MPS = 1.0;
    literal int POLL_PERIOD_MS = 20;
};
