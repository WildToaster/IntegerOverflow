#include "drivetrain.h"
#include <cmath>
#include <algorithm>
#include "filters.h"

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
    const float maxEndOutput = 0.3; // Inches / 20 msec

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

    float distanceError = distance;
    bool closeToTarget = std::abs(distanceError) < minDist;

    distancePidPacket.lastTime = brain.Timer.system();

    // setTurnSpeed(0);

    while (!closeToTarget) {
        float distanceTraveled = (getLeftDistance() - startLeftPos + getRightDistance() - startRightPos) / 2;
        distanceError = distance - distanceTraveled;

        distanceError = filters::lowPass(distanceError, distancePidPacket.lastError);

        usedTime += 5;
        distancePidPacket = pid::pidStep(distanceError, brain.Timer.system(), distancePidPacket, distanceGains);

        float trackingError = (getRightDistance() - startRightPos) - (getLeftDistance() - startLeftPos);
        
        trackingError = filters::lowPass(trackingError, trackingPidPacket.lastError);
        
        trackingPidPacket = pid::pidStep(trackingError, brain.Timer.system(), trackingPidPacket, trackingGains);

        float straightSpeed = distancePidPacket.output;
        // printf("%f %f\n", trackingError, trackingPidPacket.output);

        if (std::abs(trackingPidPacket.output) > std::abs(straightSpeed)) {
            trackingPidPacket.output = std::copysign(straightSpeed, trackingPidPacket.output);
        }

        // errorHistory.push_back(distanceError);
        // outputHistory.push_back(distancePidPacket.output / 100);
        errorHistory.push_back(trackingError * 10);
        outputHistory.push_back(trackingPidPacket.output / 20);

        printf("%f %f\n", straightSpeed, trackingPidPacket.output);

        leftMotors.spin(vex::directionType::fwd, (straightSpeed + trackingPidPacket.output) * (maxSpeed / 100) * 120, vex::voltageUnits::mV);
        rightMotors.spin(vex::directionType::fwd, (straightSpeed - trackingPidPacket.output) * (maxSpeed / 100) * 120, vex::voltageUnits::mV);

        closeToTarget = std::abs(distanceError) < minDist && std::abs(distanceError - distancePidPacket.lastError) < maxEndOutput;
        vex::this_thread::sleep_for(5);
    }

    stop();
    setBrakeMode(vex::brakeType::brake);
    pid::graphPID(brain, errorHistory, outputHistory, distance, distanceError, usedTime);
}

void Drivetrain::turnAngle(float degrees, float maxSpeed) {
    // The controller will not move for distances smaller than this.
    const float minDist = 0.3;
    const float maxEndOutput = 0.75;
    const float timeoutGain = 0.0007; // Shortened from 0.0007 as a 
    const float timeoutStatic = 2;

    float startAngle = inertial.rotation(vex::rotationUnits::deg);
    float startTime = brain.Timer.system();
    float maxTime = (timeoutStatic + maxSpeed * std::abs(degrees) * timeoutGain) * 1000;

    printf("startangle %f\n", startAngle);

    pid::PIDPacket pidPacket;
    pidPacket.lastError = degrees;

    // For graphing
    std::vector<float> errorHistory;
    std::vector<float> outputHistory;
    float usedTime = 0;

    float error = degrees;
    bool closeToTarget = std::abs(error) < minDist;

    pidPacket.lastTime = brain.Timer.system();

    while (!closeToTarget && brain.Timer.system() - startTime < maxTime) {
        float currentAngle = inertial.rotation(vex::rotationUnits::deg) - startAngle;
        error = degrees - currentAngle;

        // Filter noise from inertial sensor
        error = filters::lowPass(error, pidPacket.lastError);

        closeToTarget = std::abs(error) < minDist && std::abs(error - pidPacket.lastError) < maxEndOutput;
        // printf("error %f %f\n", error, pidPacket.output);

        usedTime += brain.Timer.system() - pidPacket.lastTime;
        pidPacket = pid::pidStep(error, brain.Timer.system(), pidPacket, turnGains);

        errorHistory.push_back(error);
        outputHistory.push_back(pidPacket.output / 100);

        setTurnSpeed(pidPacket.output * (maxSpeed / 100) * 120);

        vex::this_thread::sleep_for(11);
    }

    stop();
    pid::graphPID(brain, errorHistory, outputHistory, degrees, error, usedTime);
}
