#pragma once
/*****************
SMObjects.h
ALEXANDER CUNIO 2024
*****************/

/*
This file outlines the objects that should be utilised for shared memory between your
individual modules. These should all be created by the thread management module and
shared to other modules as required.
*/

#using <System.dll>
#include <iostream>
#include <cstdint>

using namespace System;
using namespace System::Threading;
using namespace System::Net::Sockets;
using namespace System::Net;
using namespace System::Text;
using namespace System::Diagnostics;

constexpr uint8_t bit_ALL =             0b01111111;
constexpr uint8_t bit_TM =              0b00000001;
constexpr uint8_t bit_LASER =           0b00000010;
constexpr uint8_t bit_GPS =             0b00000100;
constexpr uint8_t bit_VC =              0b00001000;
constexpr uint8_t bit_CONTROLLER =      0b00010000;
constexpr uint8_t bit_DISPLAY =         0b00100000;
constexpr uint8_t bit_CRASHAVOIDANCE =  0b01000000;

// Defines which processes are critical and not critical
// Ordering of processes in the mask should be based on the order in the UnitFlags type above.
// Critical failure -> full shutdown. Non-critical failure -> restart that thread.
// Crash avoidance can be non-critical because its limits start at "stopped" and VC stops if they go stale.
constexpr uint8_t CRITICALMASK =    bit_TM | bit_LASER | bit_VC | bit_CONTROLLER | bit_DISPLAY;
constexpr uint8_t NONCRITICALMASK = bit_GPS | bit_CRASHAVOIDANCE;

constexpr int HEARTBEAT_TIMEOUT_MS = 2000;   // TMM declares a module dead after this
constexpr int TM_ALIVE_TIMEOUT_MS = 4000;    // Modules stop if TMM is silent this long
constexpr int BARRIER_TIMEOUT_MS = 15000;

// Bit position, used as the WatchList index (bit_TM -> 0 ... bit_CRASHAVOIDANCE -> 6).
inline int bitIndex(uint8_t bit) {
    int index = 0;
    while (bit > 1) { bit >>= 1; ++index; }
    return index;
}

ref class SM_ThreadManagement
{
public:
    Object^ lockObject;
    uint8_t shutdown = 0;
    Barrier^ ThreadBarrier;
    array<Stopwatch^>^ WatchList;
    uint8_t heartbeat = 0;

    SM_ThreadManagement() {
        lockObject = gcnew Object();
    }
};

// This is the size of the data recieved from the laser
#define STANDARD_LASER_LENGTH 361

ref class SM_Laser
{
public:
    Object^ lockObject;
    array<double>^ x;           // m, forward
    array<double>^ y;           // m, left
    unsigned int scanNumber;    // Counts up with every new scan

    SM_Laser() {
        lockObject = gcnew Object();
        scanNumber = 0;
        x = gcnew array<double>(STANDARD_LASER_LENGTH);
        y = gcnew array<double>(STANDARD_LASER_LENGTH);
    }
};

ref class SM_GPS
{
public:
    Object^ lockObject;
    double Northing;
    double Easting;
    double Height;

    SM_GPS() {
        lockObject = gcnew Object();
    }
};

// Written by the controller: what the driver is asking for.
ref class SM_VehicleControl
{
public:
    Object^ lockObject;
    double Speed;       // m/s, + forward
    double Steering;    // deg, + left

    SM_VehicleControl() {
        lockObject = gcnew Object();
    }
};

// Written by crash avoidance, read by vehicle control.
// VC caps the driver's forward speed and steering to these limits.
ref class SM_CrashAvoidance
{
public:
    Object^ lockObject;
    double maxForwardSpeed;     // m/s
    double maxLeftSteer;        // deg, magnitude
    double maxRightSteer;       // deg, magnitude
    double pathClearance;       // m, free distance along the path being driven
    Stopwatch^ lastUpdate;      // Restarted on every update. VC stops if it goes stale

    // Starts at "stopped": nothing moves until crash avoidance has checked a scan.
    SM_CrashAvoidance() {
        lockObject = gcnew Object();
        maxForwardSpeed = 0.0;
        maxLeftSteer = 0.0;
        maxRightSteer = 0.0;
        pathClearance = 0.0;
        lastUpdate = Stopwatch::StartNew();
    }
};
