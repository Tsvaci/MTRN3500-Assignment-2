#pragma once
/*
VC.h - Vehicle Control Module (Part 5)
Connects to the vehicle control service, authenticates, and sends "# <steer> <speed> <wdog> #"
commands built from SM_VehicleControl. Honours the crash-avoidance inhibit flag by freezing the
watchdog. Critical thread.
*/

#include <NetworkedModule.h>

ref class VehicleControl : public NetworkedModule {
public:
    VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA);

    error_state connect(String^ hostName, int portNumber) override;   // TCP connect + authenticate
    error_state communicate() override;                               // Send one command string
    error_state processSharedMemory() override;                       // Read speed, steering and inhibit flag
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    double speed_;
    double steering_;
    bool inhibit_;
    int watchdog_;                         // Alternates 0/1 on each message while motion is allowed

    error_state authenticate();
    String^ buildCommand();                // "# <steer> <speed> <wdog> #"
    void heartbeat();

    literal int SEND_PERIOD_MS = 50;
};
