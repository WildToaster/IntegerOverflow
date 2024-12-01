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

}