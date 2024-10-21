#include "vex.h"
#include "robotConfig.h"
#include "drivetrain.h"
#include "autonSelector.h"
#include <cmath>

using namespace config;

Drivetrain drive(leftBaseMotors, rightBaseMotors, 3.25, 13.75, 36.0 / 48.0);

void autonomous() {
    printf("hi\n");
    printf("Selected route %d\n", selector::selectedRoute);
}

void userControl() {
    while (true) {
        /// Drive Code ///
        int controllerForward = controller.Axis3.position();
        int controllerTurn = controller.Axis4.position() * 0.85;

        if (std::abs(controllerForward) < 5) {
            controllerForward = 0;
        }

        if (std::abs(controllerTurn) < 5) {
            controllerTurn = 0;
        }

        drive.moveCurvatureVoltage(
            controllerForward,
            controllerTurn
        );

        //// Aux Modes ////
        // Intake
        if (controller.ButtonR1.pressing()) {
            collectionMotor.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
            conveyerMotor.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
        } else if (controller.ButtonR2.pressing()) {
            collectionMotor.spin(vex::directionType::fwd, -80, vex::velocityUnits::pct);
            conveyerMotor.spin(vex::directionType::fwd, -80, vex::velocityUnits::pct);
        } else {
            collectionMotor.stop();
            conveyerMotor.stop();
        }

        // Clamp
        if (controller.ButtonL1.pressing()) {
            leftClampPiston.set(false);
            rightClampPiston.set(false);
        } else if (controller.ButtonL2.pressing()) {
            leftClampPiston.set(true);
            rightClampPiston.set(true);
        }

        // Plow
        plowPiston.set(controller.ButtonA.pressing());

        vex::wait(20, vex::msec); // Prevent hogging resources
    }
}

int main() {
    printf("hi1\n");
    // Set up callbacks for autonomous and driver control periods.
    competition.autonomous(autonomous);
    competition.drivercontrol(userControl);

    // Initialize things
    drive.setBrakeMode(vex::brakeType::brake);
    selector::start(brain);

    // Prevent main from exiting with an infinite loop.
    while (true) {
        vex::wait(100, vex::msec);
    }
}
