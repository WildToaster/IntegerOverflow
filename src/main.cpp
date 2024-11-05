#include "vex.h"
#include "robotConfig.h"
#include "drivetrain.h"
#include "autonSelector.h"
#include <cmath>

using namespace config;

pid::PIDGains distanceGains({2.35, 0.46, 15, 20, 2});
pid::PIDGains turnGains({0.61, 0.8, 15, 40, 2});

Drivetrain drive(brain, leftBaseMotors, rightBaseMotors, config::inertial, 3.25 * 1.020833, 13.75, 36.0 / 48.0, distanceGains, turnGains);

void setClamp(bool clamping) {
    leftClampPiston.set(!clamping);
    rightClampPiston.set(!clamping);
}

void intake(int speed) {
    collectionMotor.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    conveyerMotor.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}

void blueRight() {
    setClamp(false);
    drive.moveDistance(-27, 70);
    setClamp(true);

    drive.turnAngle(-70);
    intake(80);
    drive.moveDistance(18);
    
    drive.turnAngle(-70);
    drive.moveDistance(15);
    drive.turnAngle(40);
}

void skills() {
    drive.turnAngle(360);
}

void autonomous() {
    selector::stop();
    printf("Selected route %d\n", selector::selectedRoute);

    // Wait if inertial sensor has not calibrated yet
    while (inertial.isCalibrating()) {
        vex::this_thread::sleep_for(20);
    }

    skills();
    return;

    switch (selector::selectedRoute) {
        case selector::BLUE_RIGHT:
            blueRight();
            break;
        case selector::NONE:
            controller.rumble("."); // Notify that it is intentially doing nothing
        default:
            controller.rumble("..."); // Notify that no route was found
            break;
    }
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
            setClamp(true);
        } else if (controller.ButtonL2.pressing()) {
            setClamp(false);
        }

        // Plow
        plowPiston.set(controller.ButtonA.pressing());

        vex::wait(20, vex::msec); // Prevent hogging resources
    }
}

int main() {
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
