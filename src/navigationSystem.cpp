#include "navigationSystem.h"
#include "robotConfig.h"

namespace nav {

Location getGPSPacket() {
    return Location(
        config::gpsSensor.xPosition(vex::distanceUnits::in),
        config::gpsSensor.yPosition(vex::distanceUnits::in),
        config::gpsSensor.heading());
}

Location getLocation() {
    return getGPSPacket();
}

Location getAverageLocation(int iterations) {
    Location locationSum;

    for (int i = 0; i < iterations; i++) {
        Location currentLocation = getLocation();
        locationSum.x += currentLocation.x;
        locationSum.y += currentLocation.y;
        locationSum.heading += currentLocation.heading;

        vex::this_thread::sleep_for(20);
    }

    locationSum.x /= iterations;
    locationSum.y /= iterations;
    locationSum.heading /= iterations;
    return locationSum;
}

}