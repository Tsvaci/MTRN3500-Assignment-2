#pragma once
/*
LiDAR.h - LiDAR Module (Part 2)
Connects to the SICK LMS151 service, authenticates, requests scans with sRN LMDscandata,
converts the 361 range readings to Cartesian x,y in the vehicle base frame and publishes
them to SM_Laser. Critical thread.
*/

#include <NetworkedModule.h>

ref class LiDAR : public NetworkedModule {
public:
    LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser);

    error_state connect(String^ hostName, int portNumber) override;   // TCP connect + authenticate
    error_state communicate() override;                               // Request one scan, read and parse it
    error_state processSharedMemory() override;                       // Copy the latest scan into SM_Laser
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;

    // Latest parsed scan, local to this thread until processSharedMemory copies it out
    array<double>^ x_;
    array<double>^ y_;
    unsigned int scanCount_;

    error_state authenticate();                   // Send zID, expect "OK"
    error_state requestScan();                    // Send <STX>sRN LMDscandata<ETX>
    error_state readTelegram(String^% telegram);  // Read until <ETX>, return the ASCII payload
    error_state parseScan(String^ telegram);      // Pull DIST1 data out and convert to x,y
    void printScan();                             // Readable x,y pairs on the terminal
    void heartbeat();                             // Mark this thread alive in SM_TM

    literal int READ_BUFFER_SIZE = 16384;
    literal int PRINT_EVERY_N_SCANS = 25;         // 50 Hz sensor -> print twice per second
};
