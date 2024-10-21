#include "autonSelector.h"
#include <cstdio>

namespace selector {

AutonRoute selectedRoute = AutonRoute::NONE;

vex::thread autonSelectorThread;
vex::brain::lcd* screen = nullptr;

int textColor = 0x000000;
int borderColor = 0x000000;

int activeItem = 0;
int currentID = 1;

int pressX = -1, pressY = -1;
bool pressing = false;

void prepareGUI() {
    // Clear the screen for drawing
    screen->clearScreen();

    // Reset the item ids
    currentID = 1;
    
    // Get the new inputs
    pressX = screen->xPosition();
    pressY = screen->yPosition();
    pressing = screen->pressing();
}

void finishGUI() {
    // Reset the active item
    if (!screen->pressing()) {
        // If the screen was released, reset the active item for the next frame
        activeItem = 0;
    } else if (activeItem == 0) {
        // Prevent activating something by dragging
        activeItem = -1;
    }
}

bool inArea(int x, int y, int width, int height) {
    // Check x axis
    const bool inXAxis = pressX >= x && pressX < x + width;

    // Check y axis
    const bool inYAxis = pressY >= y && pressY < y + height;

    return inXAxis && inYAxis;
}

void label(int x, int y, const char* text) {
    // Calculate the left edge of the text
    int leftEdge = x - (screen->getStringWidth(text) / 2);
    // Calculate the baseline of the text
    int baseline = y + (screen->getStringHeight(text) / 2);

    // Draw the text
    screen->setPenColor(vex::color(textColor));
    screen->printAt(leftEdge, baseline, false, text);
}

bool button(int x, int y, int width, int height, const char* text, int color) {
    const bool isHovered = inArea(x, y, width, height);

    // Get the current id and increment it by 1 for the next item.
    // Equivalent to:
    // id = currentID; currentID += 1;
    const int id = currentID++;

    if (isHovered && pressing && activeItem == 0) {
        activeItem = id;
    }

    // Draw the button
    screen->setPenColor(vex::color(borderColor));
    if (activeItem == id) {
        color *= 0.6; // Darken the button when pressed
    }
    screen->drawRectangle(x, y, width, height, vex::color(color));
    label(x + width / 2, y + height / 2, text);

    // If the user released the button, register a button press event
    return activeItem == id && isHovered && !pressing;
}

void autonSelector() {
    int unselectedColor = 0xb4b4b4;
    int selectedColor = 0x5ae800;

    while (true) { // Never stop updating the GUI
        // Make sure the screen refrence is not lost
        if (screen == nullptr) {
            std::printf("ERROR: Screen Refrence is nullptr\n");
            return;
        }

        // Call initialization stuff
        prepareGUI();

        // Autonomous Route Buttons
        int noRouteColor = selectedRoute == AutonRoute::NONE ? 0xe80000 : unselectedColor;
        if (button(0, 0, 80, 80, "None", noRouteColor)) {
            selectedRoute = AutonRoute::NONE;
        }

        // If red 1 is selected, change the color to green.
        int red1Color = selectedRoute == AutonRoute::RED_1 ? selectedColor : unselectedColor;
        if (button(80, 0, 80, 80, "Red 1", red1Color)) {
            selectedRoute = AutonRoute::RED_1;
        }

        int red2Color = selectedRoute == AutonRoute::RED_2 ? selectedColor : unselectedColor;
        if (button(160, 0, 80, 80, "Red 2", red2Color)) {
            selectedRoute = AutonRoute::RED_2;
        }

        int blue1Color = selectedRoute == AutonRoute::BLUE_1 ? selectedColor : unselectedColor;
        if (button(240, 0, 80, 80, "Blue 1", blue1Color)) {
            selectedRoute = AutonRoute::BLUE_1;
        }

        int blue2Color = selectedRoute == AutonRoute::BLUE_2 ? selectedColor : unselectedColor;
        if (button(320, 0, 80, 80, "Blue 2", blue2Color)) {
            selectedRoute = AutonRoute::BLUE_2;
        }

        int skillsColor = selectedRoute == AutonRoute::SKILLS ? selectedColor : unselectedColor;
        if (button(400, 0, 80, 80, "Skills", skillsColor)) {
            selectedRoute = AutonRoute::SKILLS;
        }

        // Set up the GUI for the next frame
        finishGUI();

        // Use screen.render to prevent screen 'flickering'
        // See https://www.vexforum.com/t/brain-screen-flashing-on-clear-screen/65574/2
        screen->render();
    }
}

void start(vex::brain& brain) {
    screen = &brain.Screen;
    autonSelectorThread = vex::thread(autonSelector);
    autonSelectorThread.detach();
}

void stop() {
    // Stop the autonomous selector thread
    autonSelectorThread.interrupt();

    // Turn off flicker prevention
    screen->renderDisable();
    screen->clearScreen();
}

} // namespace selector