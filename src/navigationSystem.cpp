#include "navigationSystem.h"

void NavigationSystem::odomLoop(void* thisArg) {
    while (true) {
        ((NavigationSystem*)thisArg)->odomStep();
    }
}

NavigationSystem::NavigationSystem(vex::gps& gps): gpsSensor(gps) {
    // odomThread = vex::thread(odomLoop, this);
    // odomThread.detach();
};

Location NavigationSystem::getGPSPacket() {
    return Location(
        gpsSensor.xPosition(vex::distanceUnits::in),
        gpsSensor.yPosition(vex::distanceUnits::in),
        gpsSensor.heading());
}

void NavigationSystem::odomStep() {

}

Location NavigationSystem::getLocation() {
    return getGPSPacket();
}