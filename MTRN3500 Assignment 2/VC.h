#pragma once
/*
VC.h - Vehicle Control Module
Sends "# <steer> <speed> <wdog> #" from SM_VehicleControl. Freezes the watchdog while crash
avoidance inhibits motion. Critical.
*/

#include <NetworkedModule.h>

ref class VehicleControl : public NetworkedModule {
public:
    VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA);

    error_state connect(String^ hostName, int portNumber) override;   // Connect and authenticate
    error_state communicate() override;
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    double speed_;
    double steering_;
    bool inhibit_;
    int watchdog_;                         // Toggles 0/1 each message

    error_state authenticate();
    String^ buildCommand();
    void heartbeat();

    literal int SEND_PERIOD_MS = 50;
};
