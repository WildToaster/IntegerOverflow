#pragma once

#include "vex.h"

struct Location {
    float x = 0;
    float y = 0;
    float heading = 0;

    Location(float x, float y, float heading): x(x), y(y), heading(heading) {};
};

class NavigationSystem {
    public:
    NavigationSystem(vex::gps& gpsSensor);
    Location getLocation();

    private:
    Location getGPSPacket();
    static void odomLoop(void* thisArg);
    void odomStep();

    vex::gps& gpsSensor;

    vex::thread odomThread;

    Location lastPosition();
    Location basePosition();
};