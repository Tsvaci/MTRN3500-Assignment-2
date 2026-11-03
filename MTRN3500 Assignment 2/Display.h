#pragma once
/*
Display.h - Display Module
Sends the SM_Laser x,y arrays to the MATLAB display engine (127.0.0.1:28000). Critical.
*/

#include <NetworkedModule.h>

ref class Display : public NetworkedModule {
public:
    Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser);

    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;

    array<double>^ x_;
    array<double>^ y_;

    void sendDisplayData(array<double>^ xData, array<double>^ yData);
    void heartbeat();

    literal int SEND_PERIOD_MS = 100;
};
