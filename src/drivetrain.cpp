#include "drivetrain.h"

Drivetrain::Drivetrain(vex::motor_group& leftMotors, vex::motor_group& rightMotors, float wheelDiameter, float wheelTrack, float gearing):
    leftMotors(leftMotors), // Sets the internal leftMotors property equal to the leftMotors parameter
    rightMotors(rightMotors),
    wheelDiameter(wheelDiameter),
    wheelTrack(wheelTrack),
    gearing(gearing) {}

void Drivetrain::moveCurvatureVoltage(float straightSpeed, float turnSpeed) {
    // Using spin for voltage bypasses internal motor PID, which is better for driving.
    float leftMotorSpeed = 12000 * (straightSpeed + turnSpeed);
    float rightMotorSpeed = 12000 * (straightSpeed - turnSpeed);
    
    if (leftMotorSpeed == 0) {
        leftMotors.stop();
    } else {
        leftMotors.spin(vex::directionType::fwd, leftMotorSpeed, vex::voltageUnits::mV);
    }

    if (rightMotorSpeed == 0) {
        rightMotors.stop();
    } else {
        rightMotors.spin(vex::directionType::fwd, rightMotorSpeed, vex::voltageUnits::mV);
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
