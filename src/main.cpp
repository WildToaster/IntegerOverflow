#include "vex.h"
#include "robotConfig.h"

// A global instance of competition
vex::competition competition;

void autonomous(void) {
  
}

void usercontrol(void) {
  while (true) {
    vex::wait(20, vex::msec); // Prevent hogging resources
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  competition.autonomous(autonomous);
  competition.drivercontrol(usercontrol);

  // Initialize things

  // Prevent main from exiting with an infinite loop.
  while (true) {
    vex::wait(100, vex::msec);
  }
}
