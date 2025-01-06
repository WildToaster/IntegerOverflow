#include "robotConfig.h"

namespace config {

vex::competition competition = vex::competition();
vex::brain brain = vex::brain();
vex::controller controller = vex::controller();

//// Base Motors ////
// Left
vex::motor leftFrontBase(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor leftMiddleBase(vex::PORT9, vex::gearSetting::ratio6_1, true);
vex::motor leftBackBase(vex::PORT10, vex::gearSetting::ratio6_1, true);
vex::motor_group leftBaseMotors(leftFrontBase, leftMiddleBase, leftBackBase);

// Right
vex::motor rightFrontBase(vex::PORT19, vex::gearSetting::ratio6_1);
vex::motor rightMiddleBase(vex::PORT17, vex::gearSetting::ratio6_1);
vex::motor rightBackBase(vex::PORT20, vex::gearSetting::ratio6_1);
vex::motor_group rightBaseMotors(rightFrontBase, rightMiddleBase, rightBackBase);

vex::inertial inertial(vex::PORT3);

//// Aux motors ////
// Intake
vex::motor collectionMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor conveyerMotor(vex::PORT2, vex::gearSetting::ratio6_1, true);
vex::motor armMotor(vex::PORT16, vex::gearSetting::ratio18_1, false);

//// Pneumatics ////
// Clamp
vex::digital_out leftClampPiston(brain.ThreeWirePort.C);
vex::digital_out rightClampPiston(brain.ThreeWirePort.A);

// Plow
vex::digital_out plowPiston(brain.ThreeWirePort.E);

//// Sensors ////
vex::gps gpsSensor(vex::PORT5, 4.46, -3.08, vex::distanceUnits::in, 176.85);
vex::rotation armRotationSensor(vex::PORT4, true);

} // namespace config