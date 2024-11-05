#include "drivetrain.h"
#include <cmath>
#include <algorithm>

Drivetrain::Drivetrain(vex::brain& brain, vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::inertial& inertial, float wheelDiameter, float wheelTrack, float gearing, pid::PIDGains distanceGains, pid::PIDGains turnGains):
    brain(brain),
    leftMotors(leftMotors), // Sets the internal leftMotors property equal to the leftMotors parameter
    rightMotors(rightMotors),
    inertial(inertial),
    wheelCircumference(wheelDiameter * 3.141592653589),
    wheelTrack(wheelTrack),
    gearing(gearing),
    distanceGains(distanceGains),
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

void Drivetrain::setTurnSpeed(float speed) {
    leftMotors.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    rightMotors.spin(vex::directionType::fwd, -speed, vex::velocityUnits::pct);
}

float Drivetrain::getLeftDistance() {
    return leftMotors.position(vex::rotationUnits::rev) * gearing * wheelCircumference;
}

float Drivetrain::getRightDistance() {
    return rightMotors.position(vex::rotationUnits::rev) * gearing * wheelCircumference;
}

// Distance is in inches
void Drivetrain::moveDistance(float distance, float maxSpeed) {
    // The controller will not move for distances smaller than this.
    const float minDist = 0.5;
    const float maxEndOutput = 15; // Feet / 20 msec

    const float startLeftPos = getLeftDistance();
    const float startRightPos = getRightDistance();

    pid::PIDPacket pidPacket;
    pidPacket.lastError = distance;

    // For graphing
    std::vector<float> errorHistory;
    std::vector<float> outputHistory;
    float usedTime = 0;

    float error = distance;
    bool closeToTarget = std::abs(error) < minDist;

    pidPacket.lastTime = brain.Timer.system();

    while (!closeToTarget) {
        float distanceTraveled = (getLeftDistance() - startLeftPos + getRightDistance() - startRightPos) / 2;
        error = distance - distanceTraveled;

        usedTime += 20;
        pidPacket = pid::pidStep(error, brain.Timer.system(), pidPacket, distanceGains);

        errorHistory.push_back(error);
        outputHistory.push_back(pidPacket.output / 100);

        setStraightSpeed(pidPacket.output * (maxSpeed / 100));

        closeToTarget = std::abs(error) < minDist && std::abs(error - pidPacket.lastError) < maxEndOutput;
        vex::this_thread::sleep_for(20);
    }

    stop();
    pid::graphPID(brain, errorHistory, outputHistory, distance, error, usedTime);
}

void Drivetrain::turnAngle(float degrees, float maxSpeed) {
    // The controller will not move for distances smaller than this.
    const float minDist = 1;
    const float maxEndOutput = 10;

    float startAngle = inertial.rotation(vex::rotationUnits::deg);

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

    while (!closeToTarget) {
        float currentAngle = inertial.rotation(vex::rotationUnits::deg) - startAngle;
        error = degrees - currentAngle;

        usedTime += 20;
        pidPacket = pid::pidStep(error, brain.Timer.system(), pidPacket, turnGains);

        errorHistory.push_back(error);
        outputHistory.push_back(pidPacket.output / 100);

        setTurnSpeed(pidPacket.output * (maxSpeed / 100));

        closeToTarget = std::abs(error) < minDist && std::abs(pidPacket.output) < maxEndOutput;
        vex::this_thread::sleep_for(20);
    }

    stop();
    pid::graphPID(brain, errorHistory, outputHistory, degrees, error, usedTime);
}
