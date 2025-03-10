#pragma once

#include "vex.h"

namespace nav {
struct Location {
    float x = 0;
    float y = 0;
    float heading = 0;

    Location() {};
    Location(float x, float y, float heading): x(x), y(y), heading(heading) {};

    inline Location operator+(Location b) {
        return Location(x + b.x, y + b.y, heading + b.heading);
    }

};

extern bool gpsEnabled;
extern bool inertialEnabled;
extern bool odomEnabled;

extern Location getLocation();
extern Location getAverageLocation(int iterations = 10);

extern void start();

extern bool syncToGPS();
extern bool syncToPos(float x, float y, float heading);
extern bool usingGPS();

}
