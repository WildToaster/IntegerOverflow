#pragma once

#include "vex.h"

namespace nav {
struct Location {
    float x = 0;
    float y = 0;
    float heading = 0;

    Location() {};
    Location(float x, float y, float heading): x(x), y(y), heading(heading) {};
};

extern Location getLocation();
extern Location getAverageLocation(int iterations = 10);

extern void start();

}
