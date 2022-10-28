
using namespace vex;

extern brain Brain;

// VEXcode devices
using signature = vision::signature;

extern signature Vision__BLUEBALL;
extern signature Vision__GREENBOX;
extern signature Vision__REDBALL;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern sonar Dis;

extern bool PlayAuto;




extern motor LF;
extern motor RF;
extern motor LB;
extern motor RB;
extern motor_group LeftDriveSmart; 
extern motor_group RightDriveSmart;
extern motor LClaw;
extern motor RClaw;
extern motor Gun;
extern controller Controller1;
extern inertial Inert_Constant;
extern line Line1;
extern line Line2;  
extern motor_group Claw_Group;
extern competition Competition;


//extern motor_group LeftDriveSmart;
//extern motor_group RightDriveSmart;

//extern motor_group Claw_Group;
//extern motor_group Gun_Group;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );