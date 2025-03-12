#include "drivetrain.h"
#include <cmath>
#include <algorithm>
#include "filters.h"
#include "vex.h"

Drivetrain::Drivetrain(vex::brain& brain, vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::inertial& inertial, float wheelDiameter, float wheelTrack, float gearing, pid::PIDGains distanceGains, pid::PIDGains trackingGains, pid::PIDGains turnGains):
    brain(brain),
    leftMotors(leftMotors), // Sets the internal leftMotors property equal to the leftMotors parameter
    rightMotors(rightMotors),
    inertial(inertial),
    wheelCircumference(wheelDiameter * 3.141592653589),
    wheelTrack(wheelTrack),
    gearing(gearing),
    distanceGains(distanceGains),
    trackingGains(trackingGains),
    turnGains(turnGains) {}

void Drivetrain::moveCurvatureVoltage(float straightSpeed, float turnSpeed) {
    // Using spin for voltage bypasses internal motor PID, which is better for driving.

    float maxOutput = std::copysign(std::max(std::abs(straightSpeed), std::abs(turnSpeed)), straightSpeed);
    
    float leftOutput;
    float rightOutput;

    // Only subtract from the max output to make turns more controllable
    if (straightSpeed >= 0) {
        if (turnSpeed >= 0) {
            leftOutput = maxOutput;
            rightOutput = straightSpeed - turnSpeed;
        } else {
            leftOutput = straightSpeed + turnSpeed;
            rightOutput = maxOutput;
        }
    } else {
        if (turnSpeed >= 0) {
            leftOutput = straightSpeed + turnSpeed;
            rightOutput = maxOutput;
        } else {
            leftOutput = maxOutput;
            rightOutput = straightSpeed - turnSpeed;
        }
    }

    if (leftOutput == 0) {
        leftMotors.stop();
    } else {
        // Divide by 100 to get into range of -1.0 to 1
        leftMotors.spin(vex::directionType::fwd, leftOutput / 100 * 12000, vex::voltageUnits::mV);
    }

    if (rightOutput == 0) {
        rightMotors.stop();
    } else {
        // Divide by 100 to get into range of -1.0 to 1
        rightMotors.spin(vex::directionType::fwd, rightOutput / 100 * 12000, vex::voltageUnits::mV);
    }
}

void Drivetrain::stop() {
    // Will eventually stop all async movements as well
    leftMotors.stop();
    rightMotors.stop();
}

void Drivetrain::setBrakeMode(vex::brakeType mode) {
    leftMotors.setStopping(mode);
    rightMotors.setStopping(mode);
}

void Drivetrain::setStraightSpeed(float speed) {
    leftMotors.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    rightMotors.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}

void Drivetrain::setTurnSpeed(float millivolts) {
    leftMotors.spin(vex::directionType::fwd, millivolts, vex::voltageUnits::mV);
    rightMotors.spin(vex::directionType::fwd, -millivolts, vex::voltageUnits::mV);
}

float Drivetrain::getLeftDistance() {
    return leftMotors.position(vex::rotationUnits::rev) * gearing * wheelCircumference;
}

float Drivetrain::getRightDistance() {
    return rightMotors.position(vex::rotationUnits::rev) * gearing * wheelCircumference;
}

// Distance is in inches
void Drivetrain::moveDistance(float distance, float maxSpeed) {
    setBrakeMode(vex::brakeType::coast);
    // The controller will not move for distances smaller than this.
    const float minDist = 0.3;
    const float maxEndOutput = 0.07; // Inches / 20 msec
    float endDeadTime = 0.4;

    const float startLeftPos = getLeftDistance();
    const float startRightPos = getRightDistance();

    pid::PIDPacket distancePidPacket;
    distancePidPacket.lastError = distance;
    distancePidPacket.setpoint = distance;

    pid::PIDPacket trackingPidPacket;

    // For graphing
    std::vector<float> errorHistory;
    std::vector<float> outputHistory;
    float usedTime = 0;
    float startTime = brain.Timer.system();
    //goalTime assumes maxSpeed at all times, tolerance term accounts for acc/deleration + variability
    float goalTime = std::abs(distance)/(maxSpeed/100 * 450 *(3.75 * 3.14) / 60 / 1000) + 500;//msec 450rpm,3.75"diam,pi,min2sec,msec
    float currentTime = startTime;
    float distanceError = distance;
    float filteredDistanceError = distance;
    bool closeToTarget = std::abs(distanceError) < minDist;

    distancePidPacket.lastTime = brain.Timer.system();

    bool isStopping = false;

    while (!closeToTarget) {
        float distanceTraveled = (getLeftDistance() - startLeftPos + getRightDistance() - startRightPos) / 2;
        distanceError = distance - distanceTraveled;

        distanceError = filters::lowPass(distanceError, distancePidPacket.lastError);
        filteredDistanceError = filters::lowPass(distanceError, filteredDistanceError);
        distanceError = filteredDistanceError;

        if (std::abs(distanceError) < minDist) isStopping = true;
        
        if (isStopping) endDeadTime -= 0.005; 
        
        closeToTarget = std::abs(distanceError) < minDist && std::abs(distanceError - distancePidPacket.lastError) < maxEndOutput;

        usedTime += 5;
        distancePidPacket = pid::pidStep(distanceError, brain.Timer.system(), distancePidPacket, distanceGains);

        float trackingError = (getRightDistance() - startRightPos) - (getLeftDistance() - startLeftPos);
        
        trackingError = filters::lowPass(trackingError, trackingPidPacket.lastError);
        
        trackingPidPacket = pid::pidStep(trackingError, brain.Timer.system(), trackingPidPacket, trackingGains);

        float straightSpeed = distancePidPacket.output;
        // printf("%f %f\n", trackingError, trackingPidPacket.output);
        // printf("%f %f\n", distance, distancePidPacket.output);

        if (std::abs(trackingPidPacket.output) > std::abs(straightSpeed)) {
            trackingPidPacket.output = std::copysign(straightSpeed, trackingPidPacket.output);
        }

        errorHistory.push_back(distanceError);
        outputHistory.push_back(distancePidPacket.output / 100);
        // errorHistory.push_back(trackingError * 10);
        // outputHistory.push_back(trackingPidPacket.output / 20);

        // printf("%f %f\n", straightSpeed, trackingPidPacket.output);

        leftMotors.spin(vex::directionType::fwd, (straightSpeed + trackingPidPacket.output) * (maxSpeed / 100) * 120, vex::voltageUnits::mV);
        rightMotors.spin(vex::directionType::fwd, (straightSpeed - trackingPidPacket.output) * (maxSpeed / 100) * 120, vex::voltageUnits::mV);
        
        if(currentTime - startTime > goalTime) {
            printf("WARNING: MoveDistance timeout on move %0.1fin @ %0.1f%%\n", distance, maxSpeed);
            closeToTarget = true;
        }

        currentTime = brain.Timer.system();
        vex::this_thread::sleep_for(5);
    }

    setBrakeMode(vex::brakeType::brake);
    stop();
    pid::graphPID(brain, errorHistory, outputHistory, distance, distanceError, usedTime);
}

void Drivetrain::turnAngle(float degrees, float maxSpeed) {
    // The controller will not move for distances smaller than this.
    const float minDist = 0.3;
    const float maxEndOutput = 0.75;
    const float timeoutGain = 0.0005; // Shortened from 0.0007 as a 
    const float timeoutStatic = 2;

    float startAngle = inertial.rotation(vex::rotationUnits::deg);
    float startTime = brain.Timer.system();
    float maxTime = (timeoutStatic + maxSpeed * std::abs(degrees) * timeoutGain) * 1000;
    float overshootTime = 400;
    bool isStopping = false;

    pid::PIDPacket pidPacket;
    pidPacket.lastError = degrees;

    // For graphing
    std::vector<float> errorHistory;
    std::vector<float> outputHistory;
    float usedTime = 0;

    float error = degrees;
    bool closeToTarget = std::abs(error) < minDist;

    pidPacket.lastTime = brain.Timer.system();

    while (!closeToTarget && brain.Timer.system() - startTime < maxTime && !(isStopping && overshootTime <= 0)) {
        float currentAngle = inertial.rotation(vex::rotationUnits::deg) - startAngle;
        error = degrees - currentAngle;

        // Filter noise from inertial sensor
        error = filters::lowPass(error, pidPacket.lastError);
        // filteredError = filters::lowPass(error, filteredError);
        // error = filteredError;

        closeToTarget = std::abs(error) < minDist && std::abs(error - pidPacket.lastError) < maxEndOutput;
        printf("error %f %f\n", error, pidPacket.output);

        usedTime += brain.Timer.system() - pidPacket.lastTime;
        if (error > 0 != pidPacket.lastError > 0) {
            isStopping = true;
        }
        if (isStopping) overshootTime -= brain.Timer.system() - pidPacket.lastTime;

        pidPacket = pid::pidStep(error, brain.Timer.system(), pidPacket, turnGains);

        errorHistory.push_back(error);
        outputHistory.push_back(pidPacket.output / 100);

        setTurnSpeed(pidPacket.output * (maxSpeed / 100) * 120);

        vex::this_thread::sleep_for(11);
    }

    setBrakeMode(vex::brakeType::brake);
    stop();
    pid::graphPID(brain, errorHistory, outputHistory, degrees, degrees - (inertial.rotation(vex::rotationUnits::deg) - startAngle), usedTime);
}

float getDistanceBetweenPoints(float startX, float startY, float endX, float endY, float angle) {
    float offsetX = startX - endX;
    float offsetY = startY - endY;

    // printf("%.2f %0.1f %0.1f %0.2f %0.2f\n", angleToTarget * 180 / M_PI, offsetX, offsetY, std::cos(angleToTarget), std::sin(angleToTarget));
    return offsetY * std::cos(angle) + offsetY * std::sin(angle);
}

void Drivetrain::toPoint(float targetX, float targetY, bool reverse, float maxSpeed) {
    // pid::PIDGains xyPID(16, 0.015, 1800, 12, -1, 0, 0);
    // pid::PIDGains angularPID(2, 0.55, 160, 24, -1, 0, 0);
    pid::PIDGains xyPID(7, 0.15, 800, 24, -1, 0, 0);
    pid::PIDGains angularPID(1.75, 2, 300, 18, -1, 0, 0);

    const float lookaheadDist = 48; // Aim for 24 inches past the target
    
    // https://www.desmos.com/calculator/nr8xrruhid
    const float priorityStart = 6;
    const float priorityEnd = 1;
    const float slope = 1 / (priorityEnd - priorityStart);
    const float slewRate = 0.05;
    float slew = 0;

    float stalledTimeout = 3000000;
    const float minimumOutput = 15;

    float inRangeTimeout = 2000000;
    const float minimumDistance = 2;

    nav::Location startLocation = nav::getLocation();

    const float initialTargetAngle = atan2(targetY - startLocation.y, targetX - startLocation.x);
    const float aimX = targetX + lookaheadDist * std::cos(initialTargetAngle);
    const float aimY = targetY + lookaheadDist * std::sin(initialTargetAngle);

    // https://www.desmos.com/calculator/i8fojr2n08
    float cos = std::cos(-initialTargetAngle);
    float sin = std::sin(-initialTargetAngle);
    const float targetDistance = (targetX - startLocation.x) * cos - (targetY - startLocation.y) * sin + startLocation.x;

    pid::PIDPacket xyPacket, angularPacket;

    while (stalledTimeout > 0 && inRangeTimeout > 0) {
        printf("\n");
        nav::Location currentLocation = nav::getLocation();
        int32_t currentTime = brain.Timer.system();
        int32_t deltaTime = currentTime - xyPacket.lastTime;

        float currentDistance = (currentLocation.x - startLocation.x) * cos - (currentLocation.y - startLocation.y) * sin + startLocation.x;
        float distanceRemaining = targetDistance - currentDistance;

        // printf("%f %f\n", sin, cos);
        // printf("%0.3f %0.3f %0.3f\n", initialTargetAngle * 180 / M_PI, targetDistance, currentDistance);

        float targetHeading = std::atan2(aimX - currentLocation.x, aimY - currentLocation.y) * 180 / M_PI;
        float headingOffset = targetHeading - currentLocation.heading;
        if (std::abs(headingOffset) > 180) headingOffset = 360 - std::abs(headingOffset);

        distanceRemaining = filters::lowPass(distanceRemaining, xyPacket.lastError);
        headingOffset = filters::lowPass(headingOffset, angularPacket.lastError);
        
        printf("Current Location (%f, %f, %f)\n", currentLocation.x, currentLocation.y, currentLocation.heading);
        printf("onGPS %d\n", nav::usingGPS());
        printf("aim %f %f %f %f\n", aimX, aimY, currentLocation.x, currentLocation.y);
        printf("Errors D %0.3f H %0.3f %0.3f\n", distanceRemaining, targetHeading, headingOffset);

        xyPacket = pid::pidStep(distanceRemaining, currentTime, xyPacket, xyPID);
        angularPacket = pid::pidStep(headingOffset, currentTime, angularPacket, angularPID);

        float xySpeed = xyPacket.output;
        xySpeed = 0;
        float angularSpeed = angularPacket.output;

        printf("initial %f %f\n", xySpeed, angularSpeed);
        xySpeed *= std::fmin(std::fmax(slope * std::abs(headingOffset) - priorityStart * slope, 0), 1);
        
        float leftSpeed = xySpeed + angularSpeed;
        float rightSpeed = xySpeed - angularSpeed;

        if (std::fmax(std::abs(leftSpeed), std::abs(rightSpeed)) < minimumOutput) {
            stalledTimeout -= deltaTime;
            printf("Stalled\n");
        }

        leftSpeed *= slew;
        rightSpeed *= slew;

        float largestSpeed = std::fmax(std::fabs(leftSpeed), std::fabs(rightSpeed));
        if (largestSpeed > maxSpeed) {
            float correction = maxSpeed / largestSpeed;
            leftSpeed *= correction;
            rightSpeed *= correction;
        }

        leftMotors.spin(vex::directionType::fwd, leftSpeed * 120, vex::voltageUnits::mV);
        rightMotors.spin(vex::directionType::fwd, rightSpeed * 120, vex::voltageUnits::mV);

        if (distanceRemaining < minimumDistance) {
            inRangeTimeout -= deltaTime;
            printf("In Range\n");
        }

        slew = std::fmin(slew + slewRate, 1);

        printf("Outputs %.3f %.3f %.3f %.3f\n", xySpeed, angularSpeed, leftSpeed, rightSpeed);

        vex::this_thread::sleep_for(20);
    }

    leftMotors.stop();
    rightMotors.stop();

    printf("%f %f %f\n", initialTargetAngle * 180 / M_PI, aimX, aimY);
}

nav::Location rotatePoint(float x, float y, float degrees) {
    float radians = degrees * M_PI / 180;
    float c = std::cos(radians);
    float s = std::sin(radians);

    nav::Location out;
    out.x = x * c - y * s;
    out.y = x * s + y * c;
    return out;
}

void Drivetrain::continuousToPoint(float x, float y, float maxSpeed) {
    nav::Location currentLocation = nav::getLocation();

    float xError = x - currentLocation.x;
    float yError = y - currentLocation.y;

    float turnKp = 1;
    float straightKp = 1;

    while (true) {
        currentLocation = nav::getLocation();
        xError = x - currentLocation.x;
        yError = y - currentLocation.y;

        nav::Location rotatedError = rotatePoint(xError, yError, currentLocation.heading);

        printf("Rotated Error: (%.3f, %.3f) %.3f\n", rotatedError.x, rotatedError.y, currentLocation.heading);

        float turnSpeed = rotatedError.x * turnKp;
        float straightSpeed = rotatedError.y * straightKp;

        if (std::abs(turnSpeed) > maxSpeed) turnSpeed = std::copysign(maxSpeed, turnSpeed);
        if (std::abs(straightSpeed) > maxSpeed) straightSpeed = std::copysign(maxSpeed, straightSpeed);

        float leftSpeed = straightSpeed + turnSpeed;
        float rightSpeed = straightSpeed - turnSpeed;

        if (std::abs(leftSpeed) > maxSpeed) leftSpeed = std::copysign(maxSpeed, leftSpeed);
        if (std::abs(rightSpeed) > maxSpeed) rightSpeed = std::copysign(maxSpeed, rightSpeed);
        
        leftMotors.spin(vex::directionType::fwd, leftSpeed * 120, vex::voltageUnits::mV);
        rightMotors.spin(vex::directionType::fwd, rightSpeed * 120, vex::voltageUnits::mV);

        printf("Speeds %f %f %f %f\n", turnSpeed, straightSpeed, leftSpeed, rightSpeed);
        
        vex::this_thread::sleep_for(20);
    }
    
}


