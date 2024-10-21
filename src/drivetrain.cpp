#include "drivetrain.h"
#include <cmath>
#include <algorithm>

Drivetrain::Drivetrain(vex::motor_group& leftMotors, vex::motor_group& rightMotors, float wheelDiameter, float wheelTrack, float gearing):
    leftMotors(leftMotors), // Sets the internal leftMotors property equal to the leftMotors parameter
    rightMotors(rightMotors),
    wheelDiameter(wheelDiameter),
    wheelTrack(wheelTrack),
    gearing(gearing) {}

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
