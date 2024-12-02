#include "vex.h"
#include "robotConfig.h"
#include "drivetrain.h"
#include "autonSelector.h"
#include <cmath>

using namespace config;

//                           {P, I, D, Max-I-term, Slew Rate, Max Slew Speed}
pid::PIDGains distanceGains({2.15, 2.2, 15, 15, 1.026, 0.01}); // .045
pid::PIDGains trackingGains({7.03, 0, 0.29, 20, -1, -1});
pid::PIDGains turnGains({0.38, 2, 15, 7.74, 1.2, 0.1});

Drivetrain drive(brain, leftBaseMotors, rightBaseMotors, config::inertial, 3.25 * 1.10590242, 13.75, 36.0 / 48.0, distanceGains, trackingGains, turnGains);

void setClamp(bool clamping) {
    leftClampPiston.set(clamping);
    rightClampPiston.set(clamping);
}

void intake(int speed) {
    collectionMotor.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    conveyerMotor.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}

void redLeft() {
    drive.moveDistance(-28);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    drive.turnAngle(95);
    intake(60);
    drive.moveDistance(14.5);
    vex::this_thread::sleep_for(1000);
    intake(-60);
    drive.moveDistance(3);
    drive.turnAngle(95);
    intake(60);
    drive.moveDistance(16);
    drive.turnAngle(115);
    intake(0);
    drive.moveDistance(42, 40);
}

void redRight() {
    drive.moveDistance(-47, 100);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    vex::this_thread::sleep_for(300);
    drive.moveDistance(18);
    // drive.turnAngle(-55); Clearing out corner, cut because of time
    drive.turnAngle(100);
    intake(80);
    drive.moveDistance(35, 40);
    intake(0);
}

void blueLeft() {
    drive.moveDistance(-47, 100);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    vex::this_thread::sleep_for(300);
    drive.moveDistance(18);
    // drive.turnAngle(-55); Clearing out corner, cut because of time
    drive.turnAngle(-90);
    intake(80);
    drive.moveDistance(35, 40);
    intake(0);
}

void blueRight() {
    drive.moveDistance(-28);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    drive.turnAngle(-95);
    intake(60);
    drive.moveDistance(14.5);
    vex::this_thread::sleep_for(1000);
    intake(-60);
    drive.moveDistance(3);
    drive.turnAngle(-95);
    intake(60);
    drive.moveDistance(16);
    drive.turnAngle(-115);
    intake(0);
    drive.moveDistance(42, 40);
}

void autonomous() {
    selector::stop();
    printf("Selected route %d\n", selector::selectedRoute);
    // inertial.calibrate();

    // Wait if inertial sensor has not calibrated yet
    while (inertial.isCalibrating()) {
        vex::this_thread::sleep_for(20);
    }

    drive.turnAngle(45);
    return;

    switch (selector::selectedRoute) {
        case selector::AutonRoute::RED_LEFT:
            redLeft();
            break;
        case selector::AutonRoute::RED_RIGHT:
            redRight();
            break;
        case selector::AutonRoute::BLUE_LEFT:
            blueLeft();
            break;
        case selector::AutonRoute::BLUE_RIGHT:
            blueRight();
            break;
        case selector::AutonRoute::NONE:
            controller.rumble("."); // Notify that it is intentially doing nothing
            break;
        default:
            controller.rumble("..."); // Notify that no route was found
            break;
    }
}

void userControl() {
    inertial.calibrate();
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
