#pragma once
#include "vex.h"

namespace config {

extern vex::competition competition;
extern vex::brain brain;
extern vex::controller controller;

//// Base Motors ////
// Left
extern vex::motor leftFrontBase;
extern vex::motor leftMiddleBase;
extern vex::motor leftBackBase;
extern vex::motor_group leftBaseMotors;

// Right
extern vex::motor rightFrontBase;
extern vex::motor rightMiddleBase;
extern vex::motor rightBackBase;
extern vex::motor_group rightBaseMotors;

//// Aux motors ////
// Intake
extern vex::motor collectionMotor;
extern vex::motor conveyerMotor;

//// arm ////
extern vex::motor armMotor;

//// Pneumatics ////
// Clamp
extern vex::digital_out leftClampPiston;
extern vex::digital_out rightClampPiston;

// Plow
extern vex::digital_out plowPiston;

// Intake Lift
extern vex::digital_out intakePiston;

//// Sensors ////
extern vex::inertial inertial;
extern vex::gps gpsSensor;
extern vex::rotation armRotationSensor;
extern vex::optical ringColorSensor;
extern vex::limit ringExitSensor;

} // namespace config