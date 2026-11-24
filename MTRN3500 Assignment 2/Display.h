#pragma once
/*
Display.h (Display Module)
Sends the latest scan, in millimetres, to the MATLAB display engine (127.0.0.1:28000). Critical.
*/

#include <NetworkedModule.h>

ref class Display : public NetworkedModule {
public:
    Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser);

    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;                               // Send the latest scan
    error_state processSharedMemory() override;                       // Copy the scan, convert to mm and display frame
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;

    array<double>^ displayX_;             // mm, + to the right (display frame)
    array<double>^ displayY_;             // mm, + forward

    void sendDisplayData(array<double>^ xData, array<double>^ yData);
    void heartbeat();

    literal int SEND_PERIOD_MS = 100;
    literal int SEND_TIMEOUT_MS = 1000;
    literal int X_TO_Y_GAP_MS = 10;       // The display engine needs a gap between the two arrays
};
