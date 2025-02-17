#include "navigationSystem.h"
#include "robotConfig.h"
#include <cmath>

namespace nav {

float odomX = 0;
float odomY = 0;
float odomHeading = 0;

float baselineOdomX = 0;
float baselineOdomY = 0;
float baselineOdomHeading = 0;

vex::thread odomThread;

float getAverageLeftOdomWheelPosition() {
    return (config::leftFrontBase.position(vex::rotationUnits::deg) +
        config::leftMiddleBase.position(vex::rotationUnits::deg) + 
        config::leftBackBase.position(vex::rotationUnits::deg)) / 3;
}

float getAverageRightOdomWheelPosition() {
    return (config::rightFrontBase.position(vex::rotationUnits::deg) +
        config::rightMiddleBase.position(vex::rotationUnits::deg) + 
        config::rightBackBase.position(vex::rotationUnits::deg)) / 3;
}

// See https://wiki.purduesigbots.com/software/odometry
void odometryLoop() {
    const float wheelCircumference = 3.25 * M_PI * 1.01384125992;
    const float gearing = 36.0 / 48.0;
    const float wheelbase = 12.75 * 0.98720303194;

    float lastLeftDist = 0;
    float lastRightDist = 0;
    float lastHeading = 0;

    while (true) {
        // Get current encoder values
        float leftEncoder = getAverageLeftOdomWheelPosition();
        float rightEncoder = getAverageRightOdomWheelPosition();

        // Calculate distance traveled by left and right wheels
        float totalLeftDist = leftEncoder * gearing / 360 * wheelCircumference;
        float totalRightDist = rightEncoder * gearing / 360 * wheelCircumference;

        float deltaLeftDist = totalLeftDist - lastLeftDist;
        float deltaRightDist = totalRightDist - lastRightDist;

        // Update previous encoder values
        lastLeftDist = totalLeftDist;
        lastRightDist = totalRightDist;
        lastHeading = odomHeading;

        // Calculate new orientation in radians
        odomHeading = baselineOdomHeading + (totalLeftDist - totalRightDist) / wheelbase;

        // Calculate change in orientation
        float deltaHeading = odomHeading - lastHeading;

        float localOffset;
        if (deltaHeading == 0) {
            localOffset = deltaRightDist;
        } else {
            localOffset = 2 * std::sin(deltaHeading * 0.5) * (deltaRightDist / deltaHeading + wheelbase * 0.5);
        }

        // Transform localOffset into the global frame
        float offsetAngle = lastHeading + deltaHeading / 2;

        float globalOffsetX = -localOffset * std::sin(-offsetAngle);
        float globalOffsetY =  localOffset * std::cos(-offsetAngle);

        odomX += globalOffsetX;
        odomY += globalOffsetY;

        vex::this_thread::sleep_for(5);
    }
}

void start() {
    odomThread = vex::thread(odometryLoop);
}

Location getGPSPacket() {
    return Location(
        config::gpsSensor.xPosition(vex::distanceUnits::in),
        config::gpsSensor.yPosition(vex::distanceUnits::in),
        config::gpsSensor.heading());
}

Location getLocation() {
    Location currentLocation;

    float xPosSum = odomX;
    float yPosSum = odomY;
    float headingSum = odomHeading + config::inertial.rotation(vex::rotationUnits::deg);
    bool usingGps = config::gpsSensor.quality() >= 95;

    if (usingGps) {
        Location gpsLocation = getGPSPacket();
        xPosSum += gpsLocation.x;
        yPosSum += gpsLocation.y;
        headingSum += gpsLocation.heading;

        currentLocation.x = xPosSum / 2;
        currentLocation.y = yPosSum / 2;
        currentLocation.heading = headingSum / 3;
    } else {
        currentLocation.x = xPosSum;
        currentLocation.y = yPosSum;
        currentLocation.heading = headingSum / 2;
    }

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