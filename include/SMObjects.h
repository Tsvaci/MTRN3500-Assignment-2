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
constexpr uint8_t CRITICALMASK =    bit_TM | bit_LASER | bit_VC | bit_CONTROLLER | bit_DISPLAY | bit_CRASHAVOIDANCE;
constexpr uint8_t NONCRITICALMASK = bit_GPS;

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
    bool inhibit;               // true when forward motion must be blocked
    double nearestObstacle;     // distance (m) to the nearest obstacle in the checked region

    SM_CrashAvoidance() {
        lockObject = gcnew Object();
        inhibit = false;
        nearestObstacle = 0.0;
    }
};
