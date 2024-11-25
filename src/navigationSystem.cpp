#include "navigationSystem.h"

NavigationSystem::NavigationSystem(vex::gps& gps): gpsSensor(gps) {};

Location NavigationSystem::getGPSPacket() {
    return Location(
        gpsSensor.xPosition(vex::distanceUnits::in),
        gpsSensor.yPosition(vex::distanceUnits::in),
        gpsSensor.heading());
}

Location NavigationSystem::getLocation() {
    return getGPSPacket();
}