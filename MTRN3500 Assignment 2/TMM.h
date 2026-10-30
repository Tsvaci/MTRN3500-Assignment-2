#pragma once
/*
TMM.h - Thread Management Module (Part 1)
Creates the shared memory, starts every other thread, monitors their heartbeats,
restarts failed non-critical threads, and performs routine or emergency shutdown.
*/

// Controller.h first: it brings in <Windows.h>, which must be seen before "using namespace System;"
// (see the note in Controller.h).
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

    // Create shared memory objects
    error_state setupSharedMemory();

    // Send/Recieve data from shared memory structures
    error_state processSharedMemory() override;

    // Shutdown all modules in the software
    void shutdownModules();

    // Get Shutdown signal for module, from Thread Management SM
    bool getShutdownFlag() override;

    // Thread function for TMM
    void threadFunction() override;

private:
    // Shared memory owned by TMM and handed to the other modules
    SM_Laser^ SM_Laser_;
    SM_GPS^ SM_GPS_;
    SM_VehicleControl^ SM_VC_;
    SM_CrashAvoidance^ SM_CA_;

    // Module objects
    Controller^ controller_;
    LiDAR^ lidar_;
    CrashAvoidance^ crashAvoidance_;
    VehicleControl^ vehicleControl_;
    Display^ display_;
    GNSS^ gnss_;

    // Threads, one per module
    Thread^ controllerThread_;
    Thread^ lidarThread_;
    Thread^ crashAvoidanceThread_;
    Thread^ vehicleControlThread_;
    Thread^ displayThread_;
    Thread^ gnssThread_;

    // Start all module threads in the order required for tele-operation
    error_state startThreads();

    // Create (or re-create) the module object and thread identified by its heartbeat bit
    error_state startModule(uint8_t moduleBit);

    // Check every heartbeat; returns the bit of a failed critical thread, or 0 if all healthy
    uint8_t monitorHeartbeats();

    // Routine shutdown request from the keyboard ('q')
    bool shutdownKeyPressed();

    // Timing constants (milliseconds)
    literal int HEARTBEAT_TIMEOUT_MS = 2000;
    literal int MONITOR_PERIOD_MS = 50;
    literal int JOIN_TIMEOUT_MS = 3000;
    literal int THREAD_COUNT = 7;    // TMM + six modules, used for the start-up barrier
};
