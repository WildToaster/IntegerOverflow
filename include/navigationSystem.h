#pragma once

#include "vex.h"
#include "drivetrain.h"

struct Location {
    float x = 0;
    float y = 0;
    float heading = 0;

    Location(float x, float y, float heading): x(x), y(y), heading(heading) {};
};

class NavigationSystem {
    public:
    NavigationSystem(vex::gps& gpsSensor, Drivetrain& drive);
    Location getLocation();

    private:
    Location getGPSPacket();
    static void odomLoop(void* thisArg);
    void odomStep();

    vex::gps& gpsSensor;

    vex::thread odomThread;
    Drivetrain& drive;

    Location lastPosition();
    Location basePosition();
};