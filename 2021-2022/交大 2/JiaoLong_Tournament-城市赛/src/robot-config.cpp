#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
// VEXcode device constructors
controller Controller1 = controller(primary);

#ifdef ID_0_
motor ChLF = motor(PORT1, ratio6_1, true);
motor ChLB = motor(PORT11, ratio6_1, true);
motor ChRF = motor(PORT21, ratio6_1, false);
motor ChRB = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT9, ratio18_1, true);
motor MogoLiftF = motor(PORT2, ratio36_1, true);
motor MogoLiftB = motor(PORT12, ratio36_1, false);
motor RingLift = motor(PORT10, ratio36_1, true);
inertial Gyro = inertial(PORT19);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_1_
motor ChLF = motor(PORT7, ratio6_1, true);
motor ChLB = motor(PORT11, ratio6_1, true);
motor ChRF = motor(PORT9, ratio6_1, false);
motor ChRB = motor(PORT20, ratio6_1, false);
motor RingLift = motor(PORT3, ratio36_1, true);
motor Intake = motor(PORT10, ratio18_1, true);
motor MogoLiftF = motor(PORT8, ratio36_1, true);
motor MogoLiftB = motor(PORT2, ratio36_1, false);
inertial Gyro = inertial(PORT15);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_2_
motor ChLF = motor(PORT20, ratio6_1, true);
motor ChLB = motor(PORT10, ratio6_1, true);
motor ChRF = motor(PORT2, ratio6_1, false);
motor ChRB = motor(PORT1, ratio6_1, false);
motor Intake = motor(PORT13, ratio18_1, true);
motor RingLift = motor(PORT11, ratio36_1, true);
motor MogoLiftF = motor(PORT18, ratio36_1, true);
motor MogoLiftB = motor(PORT8, ratio36_1, false);
inertial Gyro = inertial(PORT3);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_3_
motor ChLF = motor(PORT19, ratio6_1, true);
motor ChLB = motor(PORT8, ratio6_1, true);
motor ChRF = motor(PORT13, ratio6_1, false);
motor ChRB = motor(PORT3, ratio6_1, false);
motor Intake = motor(PORT11, ratio18_1, true);
motor MogoLiftF = motor(PORT12, ratio36_1, true);
motor MogoLiftB = motor(PORT9, ratio36_1, false);
motor RingLift = motor(PORT10, ratio36_1, true);
inertial Gyro = inertial(PORT19);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_8_
motor ChLF = motor(PORT20, ratio6_1, true);
motor ChLB = motor(PORT10, ratio6_1, true);
motor ChRF = motor(PORT11, ratio6_1, false);
motor ChRB = motor(PORT2, ratio6_1, false);
motor Intake = motor(PORT12, ratio18_1, true);
motor RingLift = motor(PORT13, ratio36_1, true);
motor MogoLiftF = motor(PORT8, ratio36_1, true);
motor MogoLiftB = motor(PORT9, ratio36_1, false);
inertial Gyro = inertial(PORT3);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_10_
motor ChLF = motor(PORT9, ratio6_1, true);
motor ChLB = motor(PORT2, ratio6_1, true);
motor ChRF = motor(PORT7, ratio6_1, false);
motor ChRB = motor(PORT4, ratio6_1, false);
motor Intake = motor(PORT12, ratio18_1, true);
motor RingLift = motor(PORT18, ratio36_1, true);
motor MogoLiftF = motor(PORT17, ratio36_1, true);
motor MogoLiftB = motor(PORT8, ratio36_1, false);
inertial Gyro = inertial(PORT14);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_23_
motor ChLF = motor(PORT2, ratio6_1, true);
motor ChLB = motor(PORT12, ratio6_1, true);
motor ChRF = motor(PORT19, ratio6_1, false);
motor ChRB = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT16, ratio18_1, true);
motor RingLift = motor(PORT9, ratio36_1, true);
motor MogoLiftF = motor(PORT3, ratio36_1, true);
motor MogoLiftB = motor(PORT11, ratio36_1, false);
inertial Gyro = inertial(PORT18);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_77_
motor ChLF = motor(PORT1, ratio6_1, true);
motor ChLB = motor(PORT11, ratio6_1, true);
motor ChRF = motor(PORT9, ratio6_1, false);
motor ChRB = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT10, ratio18_1, true);
motor MogoLiftF = motor(PORT14, ratio36_1, true);
motor MogoLiftB = motor(PORT12, ratio36_1, false);
motor RingLift = motor(PORT8, ratio36_1, true);
inertial Gyro = inertial(PORT15);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_88_
motor ChLF = motor(PORT1, ratio6_1, true);
motor ChLB = motor(PORT14, ratio6_1, true);
motor ChRF = motor(PORT9, ratio6_1, false);
motor ChRB = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT10, ratio18_1, true);
motor MogoLiftF = motor(PORT3, ratio36_1, true);
motor MogoLiftB = motor(PORT12, ratio36_1, false);
motor RingLift = motor(PORT8, ratio36_1, true);
inertial Gyro = inertial(PORT16);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_99_
motor ChLF = motor(PORT20, ratio6_1, true);
motor ChLB = motor(PORT2, ratio6_1, true);
motor ChRF = motor(PORT3, ratio6_1, false);
motor ChRB = motor(PORT1, ratio6_1, false);
motor Intake = motor(PORT6, ratio18_1, true);
motor MogoLiftF = motor(PORT9, ratio36_1, true);
motor MogoLiftB = motor(PORT7, ratio36_1, false);
motor RingLift = motor(PORT11, ratio36_1, true);
inertial Gyro = inertial(PORT12);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif

#ifdef ID_610_
motor ChLF = motor(PORT1, ratio6_1, true);
motor ChLB = motor(PORT11, ratio6_1, true);
motor ChRF = motor(PORT9, ratio6_1, false);
motor ChRB = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT8, ratio18_1, true);
motor RingLift = motor(PORT10, ratio36_1, true);
motor MogoLiftF = motor(PORT13, ratio36_1, true);
motor MogoLiftB = motor(PORT15, ratio36_1, false);
inertial Gyro = inertial(PORT21);
led MagValveF = led(Brain.ThreeWirePort.B);
led MagValveS = led(Brain.ThreeWirePort.A);
#endif
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