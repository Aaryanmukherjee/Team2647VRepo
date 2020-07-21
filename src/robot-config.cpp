#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FL = motor(PORT16, ratio18_1, false);
controller Controller1 = controller(primary);
motor FR = motor(PORT17, ratio18_1, true);
motor BR = motor(PORT14, ratio18_1, true);
motor BL = motor(PORT11, ratio18_1, false);
motor IntakeUp = motor(PORT6, ratio6_1, false);
motor Sorter = motor(PORT3, ratio18_1, false);
motor IntakeLeft = motor(PORT7, ratio18_1, false);
motor IntakeRight = motor(PORT8, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}