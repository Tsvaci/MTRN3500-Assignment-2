#pragma once
/*
GNSS.h - GNSS Module
Reads Novatel BESTUTM records, checks the CRC32 and writes Northing/Easting/Height to SM_GPS.
Non-critical.
*/

#include <NetworkedModule.h>

ref class GNSS : public NetworkedModule {
public:
    GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GPS);

    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_GPS^ SM_GPS_;

    double northing_;
    double easting_;
    double height_;
    unsigned long receivedCRC_;
    unsigned long calculatedCRC_;

    error_state findHeader(int% offset);         // Sync bytes AA 44 12
    error_state decodeRecord(int offset);
    void printPosition();
    void heartbeat();

    // CRC32 routines provided with the spec
    static unsigned long CRC32Value(int i);
    static unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer);

    literal int READ_BUFFER_SIZE = 4096;
    literal unsigned short BESTUTM_MESSAGE_ID = 726;
    literal unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;
};
