#pragma once
/*
GNSS.h - GNSS Module (Part 3)
Connects to the Novatel BESTUTM binary stream, validates each record with the CRC32,
publishes Northing/Easting/Height to SM_GPS and prints them with both CRCs. Non-critical thread.
*/

#include <NetworkedModule.h>

ref class GNSS : public NetworkedModule {
public:
    GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GPS);

    error_state connect(String^ hostName, int portNumber) override;   // TCP connect (no authentication)
    error_state communicate() override;                               // Read and decode one BESTUTM record
    error_state processSharedMemory() override;                       // Publish position to SM_GPS
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_GPS^ SM_GPS_;

    // Latest decoded record
    double northing_;
    double easting_;
    double height_;
    unsigned long receivedCRC_;
    unsigned long calculatedCRC_;

    error_state findHeader(int% offset);         // Locate the AA 44 12 sync bytes in ReadData
    error_state decodeRecord(int offset);        // Pull the fields out of a complete record
    void printPosition();
    void heartbeat();

    // CRC32 routines from Appendix A of the specification
    static unsigned long CRC32Value(int i);
    static unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer);

    literal int READ_BUFFER_SIZE = 4096;
    literal unsigned short BESTUTM_MESSAGE_ID = 726;
    literal unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;
};
