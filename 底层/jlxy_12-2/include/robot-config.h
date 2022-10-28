using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor ChRF;
extern motor ChRB;
extern motor ChLB;
extern motor ChLF;
extern motor Intake;
extern motor RingLift;
extern motor MogoLiftF;
extern motor MogoLiftB;
extern inertial Gyro;
extern led MagValveF;
extern led MagValveS;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );