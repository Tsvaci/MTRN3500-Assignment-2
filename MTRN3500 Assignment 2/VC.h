#pragma once
/*
VC.h (Vehicle Control Module)
Takes the driver's speed and steering from SM_VehicleControl, caps them to the crash-avoidance
limits in SM_CrashAvoidance and sends "# <steer> <speed> <wdog> #" to the vehicle. Critical.
*/

#include <NetworkedModule.h>

ref class VehicleControl : public NetworkedModule {
public:
    VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA);

    error_state connect(String^ hostName, int portNumber) override;   // Connect and authenticate
    error_state communicate() override;                               // Send one command
    error_state processSharedMemory() override;                       // Read command and limits, apply limits
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    double requestedSpeed_;                // From the controller
    double requestedSteering_;
    double sentSpeed_;                     // What is actually sent, after the limits
    double sentSteering_;
    double pathClearance_;                 // From crash avoidance, for the status display
    bool crashAvoidanceStale_;
    bool isLimited_;                       // Crash avoidance changed the command
    int watchdog_;                         // Toggles 0/1 each message
    Stopwatch^ statusTimer_;
    bool wasLimited_;                      // isLimited_ at the last status line

    error_state authenticate();
    String^ buildCommand();                // "# <steer> <speed> <wdog> #"
    void printStatus();
    static String^ describeMotion(double speed, double steering);
    void heartbeat();

    literal int SEND_PERIOD_MS = 50;
    literal int RECEIVE_TIMEOUT_MS = 1000;
    literal int SEND_TIMEOUT_MS = 1000;
    literal int CRASH_AVOIDANCE_STALE_MS = 300;   // Crash avoidance updates every ~20 ms
    literal int STATUS_PERIOD_MS = 2000;   // Status line while driving
    literal int MIN_STATUS_GAP_MS = 500;   // Extra lines when limiting starts/stops, but no faster than this
    literal int VIEW_PERIOD_MS = 200;      // Live status screen refresh of this module's rows
    literal double MAX_STEER_DEG = 40.0;
    literal double MAX_SPEED_MPS = 1.0;
};
