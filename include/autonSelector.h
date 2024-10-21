#pragma once
#include "vex.h"

namespace selector {
/**
 * The possible routes to be picked from.
 * Adding another route to this enum does not mean that it will automatically be displayed in the selector.
*/
enum AutonRoute {
    NONE,
    RED_1,
    RED_2,
    BLUE_1,
    BLUE_2,
    SKILLS
};

/**
 * The route that has been selected by the user.
 * The default route is AutonRoute::NONE.
*/
extern AutonRoute selectedRoute;

/**
 * Starts the autonomous selector.
 * While the autonomous selector is active, the whole screen is used, so other functions can't print to the screen until it is stopped.
 * @param brain The refrence to the robot brain
*/
extern void start(vex::brain& brain);
/**
 * Stops the autonomous selector and allows other functions to draw to the screen.
*/
extern void stop();

} // namespace selector