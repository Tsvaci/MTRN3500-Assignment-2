#pragma once
/*
TMM.h - Thread Management Module
Creates shared memory, starts and monitors all module threads, restarts non-critical
threads and handles shutdown.
*/

// Must come before UGVModule.h (Windows.h conflicts with "using namespace System", C3699).
#include "Controller.h"
#include <UGVModule.h>
#include "LiDAR.h"
#include "GNSS.h"
#include "VC.h"
#include "Display.h"
#include "CrashAvoidance.h"

ref class ThreadManagement : public UGVModule {
public:
    ThreadManagement();

    error_state setupSharedMemory();
    error_state processSharedMemory() override;   // Refresh the TMM alive signal
    void shutdownModules();
    bool getShutdownFlag() override;              // Shutdown requested by another module
    void threadFunction() override;
    bool endedWithCriticalFailure();              // For the program's exit code

private:
    SM_Laser^ SM_Laser_;
    SM_GPS^ SM_GPS_;
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;
    bool criticalFailure_;

    Controller^ controller_;
    LiDAR^ lidar_;
    CrashAvoidance^ crashAvoidance_;
    VehicleControl^ vehicleControl_;
    Display^ display_;
    GNSS^ gnss_;

    Thread^ controllerThread_;
    Thread^ lidarThread_;
    Thread^ crashAvoidanceThread_;
    Thread^ vehicleControlThread_;
    Thread^ displayThread_;
    Thread^ gnssThread_;

    error_state startThreads();
    error_state startModule(uint8_t moduleBit);   // Also used for restarts
    uint8_t checkHeartbeats();                    // Returns the bit of a dead critical module, or 0
    bool shutdownKeyPressed();
    static String^ moduleName(uint8_t moduleBit);

    // Inputs, then crash avoidance, then the actuator, display, and GNSS last.
    static array<uint8_t>^ MODULE_START_ORDER = gcnew array<uint8_t> {
        bit_CONTROLLER, bit_LASER, bit_CRASHAVOIDANCE, bit_VC, bit_DISPLAY, bit_GPS
    };

    literal int MONITOR_PERIOD_MS = 50;
    literal int THREAD_STOP_WAIT_MS = 3000;
    literal int THREAD_COUNT = 7;                 // TMM + six modules
};
