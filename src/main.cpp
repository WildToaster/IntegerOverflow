#include "vex.h"
#include "robotConfig.h"
#include "drivetrain.h"
#include "autonSelector.h"
#include "navigationSystem.h"
#include <cmath>

using namespace config;

/*

** Distance Tuning **
The Distance PID controller affects the speed at which the robot drives
forwards. The input for the controller is the distance left to travel
to get to the target.

Increasing the P term on distance gains causes the robot to be at a higher
speed for longer, but the deceleration slew rate will be higher (more abrupt).
If the robot has a "rough" deceleration, then the P term is most likely
too high. If the robot either does not continue on after a straight move
or it hits the timeout for the move, then the P term is not high enough,
since it does not have enough oomph to make it to the finish line. With
distance PID, the I and D terms are not needed, nor is a minOutput required.
This is because when going straight, the robot is not going to overshoot
because of the slower speeds as it gets closer to the target.

For adjusting the P term: increments of 0.1 - 0.25 should be sufficient.

** Tracking Tuning **
The tracking gains adjust the PID controller's response to the robot's
offset from going straight. The controller takes its input from the difference
in the distance traveled from the left and right side of the base
(Right travel - left travel).

In the tracking gains, the P term affects how strongly the controller
reacts to an offset. The higher the P term, the faster it will react, but
it will also be more prone to oscillation. The I term is used to correct for
any steady-state error. The D term is used to quickly react to abrupt changes
to the error and also helps to dampen oscillations.

More detail will be added as to how to tune this once I learn more

*/

// Parameters are: {P term, I term, D term, Max I effect, Slew Rate, Max Slew Speed, minOutput}
pid::PIDGains distanceGains({6, 0.015, 700, 12, 12, 0.03, 0}); // .045
pid::PIDGains trackingGains({37, 0.00105, 50, 20, -1, -1, 0});
pid::PIDGains turnGains({2.27, 0, 188.675, 7.74, 2, 0.4, 15});

Drivetrain drive(brain, leftBaseMotors, rightBaseMotors, config::inertial, 3.25 * 0.9791667, 13.75, 36.0 / 48.0, distanceGains, trackingGains, turnGains);

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
    drive.turnAngle(90);
    intake(60);
    drive.moveDistance(14.5);
    drive.turnAngle(90);
    drive.moveDistance(16);
    drive.turnAngle(110);
    intake(0);
    drive.moveDistance(42, 40);
}

void redRight() {
    drive.moveDistance(-43, 100);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    vex::this_thread::sleep_for(300);
    drive.moveDistance(18);
    // drive.turnAngle(-55); Clearing out corner, cut because of time
    drive.turnAngle(75);
    intake(80);
    drive.moveDistance(48, 40);
    intake(0);
}

void blueLeft() {
    drive.moveDistance(-43, 100);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    vex::this_thread::sleep_for(300);
    drive.moveDistance(18);
    // drive.turnAngle(-55); Clearing out corner, cut because of time
    drive.turnAngle(-75);
    intake(80);
    drive.moveDistance(48, 55);
    intake(0);
}

void blueRight() {
    drive.moveDistance(-28);
    vex::this_thread::sleep_for(300);
    setClamp(true);
    drive.turnAngle(-90);
    intake(60);
    drive.moveDistance(14.5);
    drive.turnAngle(-90);
    drive.moveDistance(16);
    drive.turnAngle(-110);
    intake(0);
    drive.moveDistance(42, 40);
}

void autonomous() {
    selector::stop();
    printf("Selected route %d\n", selector::selectedRoute);

    // Wait if inertial sensor has not calibrated yet
    while (inertial.isCalibrating()) {
        vex::this_thread::sleep_for(20);
    }

    while (gpsSensor.isCalibrating()) {
        vex::this_thread::sleep_for(20);
    }

    drive.toPoint(-48, 24);
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
        printf("%f %f %f\n", nav::getLocation().x, nav::getLocation().y, nav::getLocation().heading);
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
    inertial.calibrate();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        vex::wait(100, vex::msec);
    }
}
