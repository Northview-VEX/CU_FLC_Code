#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor back_R = motor(PORT1, ratio18_1, true);
motor back_L = motor(PORT10, ratio18_1, false);
motor front_R = motor(PORT11, ratio18_1, true);
motor right_intake = motor(PORT16, ratio6_1, true);
motor sorter = motor(PORT6, ratio6_1, true);
motor indexer = motor(PORT5, ratio6_1, true);
motor left_intake = motor(PORT15, ratio6_1, false);
motor front_L = motor(PORT20, ratio18_1, false);
controller Controller1 = controller(primary);
line ballPos1 = line(Brain.ThreeWirePort.G);
line ballPos2 = line(Brain.ThreeWirePort.H);
inertial right_inertial = inertial(PORT18);
inertial left_inertial = inertial(PORT13);
encoder left_encoder = encoder(Brain.ThreeWirePort.A);
encoder right_encoder = encoder(Brain.ThreeWirePort.C);
encoder back_encoder = encoder(Brain.ThreeWirePort.E);

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