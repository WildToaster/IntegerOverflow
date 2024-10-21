#pragma once

#include "vex.h"

class Drivetrain {
public:
// Gearing is (Motor gear / Output gear)
// Wheel diameter and track is in inches
Drivetrain(vex::motor_group& leftMotors, vex::motor_group& rightMotors, float wheelDiameter, float wheelTrack, float gearing = 1);

// Positive turn speed = Clockwise
void moveCurvatureVoltage(float straightSpeed, float turnSpeed);
void moveDistance(float distance, float maxSpeed = 100);
void turnAngle(float degrees, float maxSpeed = 100);
void stop();

void setBrakeMode(vex::brakeType mode);

private:
vex::motor_group& leftMotors;
vex::motor_group& rightMotors;

void setStraightSpeed(float speed);
void setTurnSpeed(float speed);

float wheelDiameter;
float wheelTrack;
float gearing;

};