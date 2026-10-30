#pragma once
/*
Display.h - Display Module (Part 6)
Streams the LiDAR x,y arrays from SM_Laser to the MATLAB display engine over a local TCP
connection (127.0.0.1:28000) as raw bytes, X array then Y array. Critical thread.
*/

#include <NetworkedModule.h>

ref class Display : public NetworkedModule {
public:
    Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser);

    error_state connect(String^ hostName, int portNumber) override;   // TCP connect to MATLAB (no authentication)
    error_state communicate() override;                               // Send the current x,y arrays
    error_state processSharedMemory() override;                       // Copy x,y out of SM_Laser
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;

    array<double>^ x_;
    array<double>^ y_;

    void sendDisplayData(array<double>^ xData, array<double>^ yData);  // Appendix B of the specification
    void heartbeat();

    literal int SEND_PERIOD_MS = 100;
};
