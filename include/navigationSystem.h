#pragma once

#include "vex.h"

struct Location {
    float x, y, heading;

    Location(float x, float y, float heading): x(x), y(y), heading(heading) {};
};

class NavigationSystem {
    public:
    NavigationSystem(vex::gps& gpsSensor);
    Location getLocation();

    private:
    Location getGPSPacket();
    vex::gps& gpsSensor;
};