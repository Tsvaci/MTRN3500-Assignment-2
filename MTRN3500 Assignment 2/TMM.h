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

    // Create every shared memory object (called once from main before the thread starts)
    error_state setupSharedMemory();

    // Publish TMM's own "alive" signal so the modules know TMM is still supervising them
    error_state processSharedMemory() override;

    // Tell every module to stop, then wait for each thread to finish
    void shutdownModules();

    // True when another module (e.g. the controller's shutdown button) has asked TMM to stop
    bool getShutdownFlag() override;

    // TMM's main loop: start the modules, supervise them, shut them down
    void threadFunction() override;

private:
    // Shared memory owned by TMM and handed to the other modules (SM_TM_ itself lives in UGVModule)
    SM_Laser^ SM_Laser_;               // LiDAR scan, x/y in metres
    SM_GPS^ SM_GPS_;                   // GNSS position
    SM_VehicleControl^ SM_VC_;         // Speed and steering commands
    SM_CrashAvoidance^ SM_CA_;         // Crash-avoidance "inhibit motion" decision

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

    // Check every module's heartbeat and restart dead non-critical ones.
    // Returns the bit of a dead critical module (so the caller can shut down), or 0 if all is well.
    uint8_t checkHeartbeats();

    // True if 'q' has been pressed in this console window
    bool shutdownKeyPressed();

    // Readable module name for console messages, e.g. bit_LASER -> "LiDAR"
    static String^ moduleName(uint8_t moduleBit);

    // The six module threads, in the order TMM starts them: inputs (controller, LiDAR) first,
    // then the safety check (crash avoidance), then the actuator (vehicle control), then the
    // display, and finally the non-critical GNSS.
    static array<uint8_t>^ MODULE_START_ORDER = gcnew array<uint8_t> {
        bit_CONTROLLER, bit_LASER, bit_CRASHAVOIDANCE, bit_VC, bit_DISPLAY, bit_GPS
    };

    // Timing constants (milliseconds). HEARTBEAT_TIMEOUT_MS is in SMObjects.h because modules use it too.
    literal int MONITOR_PERIOD_MS = 50;      // How often TMM checks heartbeats and the keyboard
    literal int THREAD_STOP_WAIT_MS = 3000;  // How long shutdown waits for each thread to finish
    literal int THREAD_COUNT = 7;            // TMM + six modules: the number of threads the barrier waits for
};
