#pragma once

#include "vex.h"
#include "controlLoop.h"
#include "navigationSystem.h"

class Drivetrain {
public:
// Gearing is (Motor gear / Output gear)
// Wheel diameter and track is in inches
Drivetrain(vex::brain& brain, vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::inertial& inertial, float wheelDiameter, float wheelTrack, float gearing, pid::PIDGains distanceGains, pid::PIDGains trackingGains, pid::PIDGains turnGains);

// Positive turn speed = Clockwise
void moveCurvatureVoltage(float straightSpeed, float turnSpeed);
void moveDistance(float distance, float maxSpeed = 70);
void turnAngle(float degrees, float maxSpeed = 70);
void toPoint(float x, float y, bool reverse = false, float maxSpeed = 70);
void stop();

void setBrakeMode(vex::brakeType mode);

private:
vex::brain& brain;

vex::motor_group& leftMotors;
vex::motor_group& rightMotors;

vex::inertial& inertial;

void setStraightSpeed(float speed);
void setTurnSpeed(float speed);

float getLeftDistance();
float getRightDistance();

float wheelCircumference;
float wheelTrack;
float gearing;

pid::PIDGains distanceGains;
pid::PIDGains trackingGains;
pid::PIDGains turnGains;

};