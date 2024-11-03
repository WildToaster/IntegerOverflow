#pragma once

#include "vex.h"
#include "controlLoop.h"

class Drivetrain {
public:
// Gearing is (Motor gear / Output gear)
// Wheel diameter and track is in inches
Drivetrain(vex::brain& brain, vex::motor_group& leftMotors, vex::motor_group& rightMotors, float wheelDiameter, float wheelTrack, float gearing, pid::PIDGains distanceGains);

// Positive turn speed = Clockwise
void moveCurvatureVoltage(float straightSpeed, float turnSpeed);
void moveDistance(float distance, float maxSpeed = 80);
void turnAngle(float degrees, float maxSpeed = 80);
void stop();

void setBrakeMode(vex::brakeType mode);

private:
vex::brain& brain;

vex::motor_group& leftMotors;
vex::motor_group& rightMotors;

void setStraightSpeed(float speed);
void setTurnSpeed(float speed);

float wheelCircumference;
float wheelTrack;
float gearing;

pid::PIDGains distanceGains;

};