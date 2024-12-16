#include "robotConfig.h"

namespace config {

vex::competition competition = vex::competition();
vex::brain brain = vex::brain();
vex::controller controller = vex::controller();

//// Base Motors ////
// Left
vex::motor leftFrontBase(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor leftMiddleBase(vex::PORT7, vex::gearSetting::ratio6_1, true);
vex::motor leftBackBase(vex::PORT10, vex::gearSetting::ratio6_1, true);
vex::motor_group leftBaseMotors(leftFrontBase, leftMiddleBase, leftBackBase);

// Right
vex::motor rightFrontBase(vex::PORT17, vex::gearSetting::ratio6_1);
vex::motor rightMiddleBase(vex::PORT9, vex::gearSetting::ratio6_1);
vex::motor rightBackBase(vex::PORT8, vex::gearSetting::ratio6_1);
vex::motor_group rightBaseMotors(rightFrontBase, rightMiddleBase, rightBackBase);

vex::inertial inertial(vex::PORT3);

//// Aux motors ////
// Intake
vex::motor collectionMotor(vex::PORT6, vex::gearSetting::ratio6_1, true);
vex::motor conveyerMotor(vex::PORT20, vex::gearSetting::ratio18_1, true);

//// Pneumatics ////
// Clamp
vex::digital_out leftClampPiston(brain.ThreeWirePort.G);
vex::digital_out rightClampPiston(brain.ThreeWirePort.H);

// Plow
vex::digital_out plowPiston(brain.ThreeWirePort.F);

//// Sensors ////
vex::gps gpsSensor(vex::PORT5, 4.46, -3.08, vex::distanceUnits::in, 176.85);

} // namespace config