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
    error_state communicate() override;           // Read the stream and decode any complete records
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;

private:
    SM_GPS^ SM_GPS_;

    int bufferedBytes_;                           // Bytes waiting in ReadData
    double northing_;
    double easting_;
    double height_;
    unsigned long receivedCRC_;
    unsigned long calculatedCRC_;
    unsigned int recordCount_;
    Stopwatch^ printTimer_;
    unsigned int rejectedCount_;

    int findRecordStart();                        // Index of the first AA 44 12 sync bytes in ReadData, or -1
    error_state decodeRecord(int offset);         // Check and decode the record starting at offset
    void discardBytes(int count);                 // Drop bytes from the front of ReadData
    void printPosition(bool crcOk);
    void heartbeat();

    // CRC32 routines provided with the spec
    static unsigned long CRC32Value(int i);
    static unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer);

    literal int READ_BUFFER_SIZE = 4096;
    literal int RECEIVE_TIMEOUT_MS = 1000;
    literal int MAX_CONSECUTIVE_TIMEOUTS = 3;
    literal int PRINT_PERIOD_MS = 2000;           // Good records: print the latest one this often
    literal int VIEW_PERIOD_MS = 200;             // Live status screen refresh
    literal int HEADER_LENGTH = 28;
    literal int BODY_LENGTH = 80;
    literal int CRC_LENGTH = 4;
    literal int RECORD_LENGTH = HEADER_LENGTH + BODY_LENGTH + CRC_LENGTH;   // 112 bytes
    literal unsigned short BESTUTM_MESSAGE_ID = 726;
    literal unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;
};
