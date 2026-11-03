#pragma once
/*
LiDAR.h - LiDAR Module
Requests scans from the SICK LMS151, converts the 361 ranges to x,y in the vehicle frame
and writes them to SM_Laser. Critical.
*/

#include <NetworkedModule.h>

ref class LiDAR : public NetworkedModule {
public:
    LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser);

    error_state connect(String^ hostName, int portNumber) override;   // Connect and authenticate
    error_state communicate() override;                               // Request, read and parse one scan
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_Laser^ SM_Laser_;

    array<double>^ x_;
    array<double>^ y_;
    unsigned int scanCount_;

    error_state authenticate();
    error_state requestScan();
    error_state readTelegram(String^% telegram);  // Payload between STX and ETX
    error_state parseScan(String^ telegram);
    void printScan();
    void heartbeat();

    literal int READ_BUFFER_SIZE = 16384;         // One reply is ~2.5 KB
    literal int RECEIVE_TIMEOUT_MS = 2000;
    literal int SEND_TIMEOUT_MS = 1000;
    literal int SCAN_PERIOD_MS = 20;              // Sensor runs at 50 Hz
    literal int PRINT_EVERY_N_SCANS = 25;
    literal unsigned char STX = 0x02;
    literal unsigned char ETX = 0x03;
};
