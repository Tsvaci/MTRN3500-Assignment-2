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
// Critical threads: a failure causes a complete shutdown.
// Non-critical threads: a failure causes TMM to attempt a restart of that thread only.
// Crash avoidance is non-critical (Aidan, 2 Nov 2026): it is restarted rather than shutting everything
// down. That is only safe because SM_CrashAvoidance starts with inhibit = true (see below).
constexpr uint8_t CRITICALMASK =    bit_TM | bit_LASER | bit_VC | bit_CONTROLLER | bit_DISPLAY;
constexpr uint8_t NONCRITICALMASK = bit_GPS | bit_CRASHAVOIDANCE;

// Heartbeat timing shared by TMM and every module.
// A module is considered dead if TMM has not seen its heartbeat for HEARTBEAT_TIMEOUT_MS.
// A module considers TMM dead (and shuts itself down) if TMM's stopwatch exceeds TM_ALIVE_TIMEOUT_MS.
constexpr int HEARTBEAT_TIMEOUT_MS = 2000;
constexpr int TM_ALIVE_TIMEOUT_MS = 4000;
constexpr int BARRIER_TIMEOUT_MS = 15000;   // Maximum wait for all threads to finish starting up

// Position of a module's bit in the heartbeat/shutdown bytes, used as its index into WatchList.
// bit_TM -> 0, bit_LASER -> 1, bit_GPS -> 2, bit_VC -> 3, bit_CONTROLLER -> 4, bit_DISPLAY -> 5, bit_CRASHAVOIDANCE -> 6
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
    array<double>^ x;
    array<double>^ y;

    SM_Laser() {
        lockObject = gcnew Object();
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

ref class SM_VehicleControl
{
public:
    Object^ lockObject;
    double Speed;
    double Steering;

    SM_VehicleControl() {
        lockObject = gcnew Object();
    }
};

// Added for Part 7. Written by the crash avoidance module, read by vehicle control.
ref class SM_CrashAvoidance
{
public:
    Object^ lockObject;
    bool inhibit;               // true when forward motion must be blocked. Fail-safe: starts true, so
                                // nothing moves until crash avoidance has checked a scan and cleared it.
    double nearestObstacle;     // distance (m) to the nearest obstacle in the checked region

    SM_CrashAvoidance() {
        lockObject = gcnew Object();
        inhibit = true;
        nearestObstacle = 0.0;
    }
};
