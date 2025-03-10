#include "navigationSystem.h"
#include "robotConfig.h"
#include <cmath>

namespace nav {

float odomX = 0;
float odomY = 0;
float odomHeading = 0;

float lastOdomLeftDist = 0;
float lastOdomRightDist = 0;
float lastOdomHeading = 0;

Location odomOffset(0, 0, 0);

vex::thread odomThread;

bool gpsEnabled = true;
bool inertialEnabled = false;
bool odomEnabled = true;

Location getGPSPacket() {
    return Location(
        config::gpsSensor.xPosition(vex::distanceUnits::in),
        config::gpsSensor.yPosition(vex::distanceUnits::in),
        config::gpsSensor.rotation());
}

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
    const float wheelCircumference = 3.25 * M_PI * 1.0038552651359253499222395023328;
    const float gearing = 36.0 / 48.0;
    const float wheelbase = 12.75 * 0.98408982033690525572916666666667;

    config::leftBaseMotors.resetPosition();
    config::rightBaseMotors.resetPosition();

    while (true) {
        // Get current encoder values
        float leftEncoder = getAverageLeftOdomWheelPosition();
        float rightEncoder = getAverageRightOdomWheelPosition();

        // Calculate distance traveled by left and right wheels
        float totalLeftDist = leftEncoder * gearing / 360 * wheelCircumference;
        float totalRightDist = rightEncoder * gearing / 360 * wheelCircumference;

        float deltaLeftDist = totalLeftDist - lastOdomLeftDist;
        float deltaRightDist = totalRightDist - lastOdomRightDist;

        // Update previous encoder values
        lastOdomLeftDist = totalLeftDist;
        lastOdomRightDist = totalRightDist;
        lastOdomHeading = odomHeading;

        // Calculate new orientation in radians
        odomHeading = (totalLeftDist - totalRightDist) / wheelbase;

        // Calculate change in orientation
        float deltaHeading = odomHeading - lastOdomHeading;

        float localOffset;
        if (deltaHeading == 0) {
            localOffset = deltaRightDist;
        } else {
            localOffset = 2 * std::sin(deltaHeading * 0.5) * (deltaRightDist / deltaHeading + wheelbase * 0.5);
        }

        // Transform localOffset into the global frame
        float offsetAngle = lastOdomHeading + deltaHeading / 2;

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

Location rotatePoint(float x, float y, float degrees) {
    float radians = degrees * M_PI / 180;
    float c = std::cos(radians);
    float s = std::sin(radians);

    nav::Location out;
    out.x = x * c - y * s;
    out.y = x * s + y * c;
    return out;
}

Location getLocation() {
    Location currentLocation;

    Location odomPos = rotatePoint(odomX, odomY, -odomOffset.heading);
    float xPosSum = 0;
    float yPosSum = 0;
    float headingSum = 0;

    bool useGPS = usingGPS();

    if (odomEnabled) {
        xPosSum += odomPos.x + odomOffset.x;
        yPosSum += odomPos.y + odomOffset.y;    
        headingSum += odomHeading * 180 / M_PI + odomOffset.heading;
    }

    if (inertialEnabled) {
        headingSum += config::inertial.rotation(vex::rotationUnits::deg);
    }

    if (useGPS) {
        Location gpsLocation = getGPSPacket();

        xPosSum += gpsLocation.x;
        yPosSum += gpsLocation.y;
        headingSum += gpsLocation.heading;
    }

    int numSources = ((int) odomEnabled) + ((int) inertialEnabled) + ((int) useGPS);
    currentLocation.x = xPosSum / numSources;
    currentLocation.y = yPosSum / numSources;
    currentLocation.heading = headingSum / numSources;

    return currentLocation;
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

bool usingGPS() {
    return config::gpsSensor.quality() >= 95 && gpsEnabled;
}

bool syncToGPS() {
    if (!usingGPS()) return false;
    Location gpsLoc = getGPSPacket();

    odomOffset = gpsLoc;
    odomX = 0;
    odomY = 0;
    odomHeading = 0;
    lastOdomHeading = 0;
    lastOdomLeftDist = 0;
    lastOdomRightDist = 0;
    config::leftBaseMotors.resetPosition();
    config::rightBaseMotors.resetPosition();

    config::inertial.setRotation(gpsLoc.heading, vex::rotationUnits::deg);
    return true;
}

bool syncToPos(float x, float y, float heading) {
    if (usingGPS()) return false;
    odomOffset = Location(x, y, heading);
    odomX = 0;
    odomY = 0;
    odomHeading = 0;
    lastOdomHeading = 0;
    lastOdomLeftDist = 0;
    lastOdomRightDist = 0;
    config::leftBaseMotors.resetPosition();
    config::rightBaseMotors.resetPosition();

    config::inertial.setRotation(heading, vex::rotationUnits::deg);
    return true;
}

}