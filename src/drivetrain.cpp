#include "drivetrain.h"
#include <cmath>
#include <algorithm>

Drivetrain::Drivetrain(vex::brain& brain, vex::motor_group& leftMotors, vex::motor_group& rightMotors, float wheelDiameter, float wheelTrack, float gearing, pid::PIDGains distanceGains):
    brain(brain),
    leftMotors(leftMotors), // Sets the internal leftMotors property equal to the leftMotors parameter
    rightMotors(rightMotors),
    wheelCircumference(wheelDiameter * 3.141592653589),
    wheelTrack(wheelTrack),
    gearing(gearing),
    distanceGains(distanceGains) {}

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

// Distance is in inches
void Drivetrain::moveDistance(float distance, float maxspeed) {
    // The controller will not move for distances smaller than this.
    const float minDist = 0.5;
    const float maxEndVelocity = 0.2 / 12; // Feet / 20 msec

    const float startLeftPos = leftMotors.position(vex::rotationUnits::rev);
    const float startRightPos = rightMotors.position(vex::rotationUnits::rev);

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
        float averageMotorPos = (leftMotors.position(vex::rotationUnits::rev) - startLeftPos + rightMotors.position(vex::rotationUnits::rev) - startRightPos) / 2;
        float distanceTraveled = wheelCircumference * averageMotorPos;
        error = distance - distanceTraveled;

        usedTime += 20;
        printf("dt %f\n", brain.Timer.system() - pidPacket.lastTime);
        pidPacket = pid::pidStep(error, brain.Timer.system(), pidPacket, distanceGains);

        errorHistory.push_back(error);
        outputHistory.push_back(pidPacket.output / 100);

        setStraightSpeed(pidPacket.output * (maxspeed / 100));

        closeToTarget = std::abs(error) < minDist && std::abs(error - pidPacket.lastError) < maxEndVelocity;
        vex::this_thread::sleep_for(20);
    }

    stop();
    pid::graphPID(brain, errorHistory, outputHistory, distance, error, usedTime);
}
