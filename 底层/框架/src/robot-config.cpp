#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
// VEXcode device constructors
controller Controller1 = controller(primary);

motor ChLF = motor(PORT1, ratio6_1, false);//地盘
motor ChLB = motor(PORT12, ratio6_1, false);
motor ChRF = motor(PORT6, ratio6_1,true);
motor ChRB = motor(PORT9, ratio6_1,true);
motor Intake = motor(PORT14, ratio18_1, true);
motor MogoLiftF = motor(PORT13, ratio36_1, true);//抬升
motor MogoLiftB = motor(PORT18, ratio36_1, false);
motor RingLift =  motor(PORT3, ratio36_1, true);

inertial Gyro = inertial(PORT8);
led MagValveF = led(Brain.ThreeWirePort.D);//电磁阀
led MagValveS = led(Brain.ThreeWirePort.G);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
}