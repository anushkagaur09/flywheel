#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor rightFront = motor(PORT5, ratio18_1, true);
motor leftFront = motor(PORT3, ratio18_1, true);
motor leftBack = motor(PORT8, ratio18_1, false);
motor rightBack = motor(PORT12, ratio18_1, false);
motor Intake = motor(PORT9, ratio18_1, false);
digital_out Expansion = digital_out(Brain.ThreeWirePort.A);
motor flywheel = motor(PORT11, ratio18_1, false);
motor flywheel2 = motor(PORT15, ratio18_1, false);
motor indexer = motor(PORT16, ratio18_1, true);

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