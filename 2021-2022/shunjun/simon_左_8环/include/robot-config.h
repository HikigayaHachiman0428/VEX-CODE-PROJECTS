
using namespace vex;

extern brain Brain;

// VEXcode devices



extern bool PlayAuto;


extern motor LF;
extern motor RF;
extern motor LB;
extern motor RB;
extern motor_group LeftDriveSmart; 
extern motor_group RightDriveSmart;
extern motor UpClaw;
extern motor DownClaw;
extern controller Controller1;
extern inertial Inert_Constant;
extern rotation Angle;
extern motor_group Claw_Group;
extern competition Competition;
extern motor Lift;
extern motor BackClaw;



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );