#include "vex.h"
#include "robotConfig.h"
#include "drivetrain.h"

Drivetrain drive(config::leftBaseMotors, config::rightBaseMotors, 3.25, 13.75, 36.0 / 48.0);

void autonomous() {}

void userControl() {
    while (true) {
        drive.moveCurvatureVoltage(
            config::controller.Axis3.position(), // Forward
            config::controller.Axis4.position()  // Turn
        );

        vex::wait(20, vex::msec); // Prevent hogging resources
    }
}

int main() {
    // Set up callbacks for autonomous and driver control periods.
    config::competition.autonomous(autonomous);
    config::competition.drivercontrol(userControl);

    // Initialize things

    // Prevent main from exiting with an infinite loop.
    while (true) {
        vex::wait(100, vex::msec);
    }
}
