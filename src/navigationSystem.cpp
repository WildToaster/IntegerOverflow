#include "navigationSystem.h"

void NavigationSystem::odomLoop(void* thisArg) {
    while (true) {
        ((NavigationSystem*)thisArg)->odomStep();
    }
}

NavigationSystem::NavigationSystem(vex::gps& gps, Drivetrain& drive): gpsSensor(gps), drive(drive) {
    odomThread = vex::thread(odomLoop, this);
    odomThread.detach();
};

Location NavigationSystem::getGPSPacket() {
    return Location(
        gpsSensor.xPosition(vex::distanceUnits::in),
        gpsSensor.yPosition(vex::distanceUnits::in),
        gpsSensor.heading());
}

Location NavigationSystem::getLocation() {
    return getGPSPacket();
}

void NavigationSystem::odomStep() {
    

    vex::this_thread::sleep_for(10);
}