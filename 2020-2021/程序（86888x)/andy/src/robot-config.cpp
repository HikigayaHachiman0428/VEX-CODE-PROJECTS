#include "vex.h"
#include "cmath"
#include <iostream> 
#include <string> 
using namespace vex;
using signature = vision::signature;
using code = vision::code;

bool PlayAuto=false;
brain  Brain;
controller Controller1 = controller(primary);

motor LF = motor(PORT1, ratio18_1, false);             motor RF = motor(PORT2, ratio18_1,true);
motor LB = motor(PORT11, ratio18_1, false);             motor RB = motor(PORT12, ratio18_1,true);
motor_group LeftDriveSmart = motor_group(LF,LB);        motor_group RightDriveSmart = motor_group(RF,RB);

motor LClaw = motor(PORT5, ratio18_1, false);           motor RClaw = motor(PORT6, ratio18_1, true);
motor_group Claw_Group=motor_group(LClaw, RClaw);

motor Gun = motor(PORT13, ratio18_1, false);    

inertial Inert_Constant = inertial(PORT14);

signature Vision__BLUEBALL = signature (1, -3441, -2785, -3113, 8975, 10355, 9665, 11, 0);
signature Vision__GREENBOX = signature (2, -5767, -4965, -5366, -3803, -2861, -3332, 1.7, 0);
signature Vision__REDBALL = signature (3, 8099, 8893, 8496, -1505, -949, -1227, 11, 0);
signature Vision__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision = vision (PORT7, 50, Vision__BLUEBALL, Vision__GREENBOX, Vision__REDBALL, Vision__SIG_4, Vision__SIG_5, Vision__SIG_6);




bool RemoteControlCodeEnabled = true;
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool isMovingL = true;
bool isMovingR = true;
bool lowtorque =true;
timer drivertime;
int i=500000;

// define a task that will handle monitoring inputs from Controller1


bool first=true;

int rc_auto_loop_function_Controller1() {
  if(first){
      LClaw.setVelocity(100, percent);
      RClaw.setVelocity(100, percent);
      Gun.setVelocity(100, percent);
      LClaw.setMaxTorque(100,percent);
      RClaw.setMaxTorque(100, percent);
      Gun.setMaxTorque(100, percent);
    first=false;
  }

  while(true) {

    if(drivertime>300){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print(Inert_Constant.heading());
      Controller1.Screen.newLine();
      Controller1.Screen.print(Brain.Battery.voltage());
      Controller1.Screen.print("V");
      drivertime.reset();
    }
    if(RemoteControlCodeEnabled) {
      int Left_verticle=Controller1.Axis3.position();
      int Right_horizontal=Controller1.Axis1.position();
      if(abs(Right_horizontal)<40)
        Right_horizontal*=0.35;
      else if (abs(Right_horizontal)<70) {
        Right_horizontal*=0.55;
      }
      int drivetrainLeftSideSpeed =Left_verticle + Right_horizontal;
      int drivetrainRightSideSpeed =Left_verticle - Right_horizontal;
      /*double ratio=fmax(abs(drivetrainLeftSideSpeed),abs(drivetrainRightSideSpeed))/100.0;
      drivetrainRightSideSpeed/=ratio;
      drivetrainLeftSideSpeed/=ratio;
      drivetrainLeftSideSpeed*=0.9;*/
      if(abs(Right_horizontal)>5){
        if(!lowtorque)
        {
          LeftDriveSmart.setMaxTorque(1.5*abs(Right_horizontal)+15, percent);
          RightDriveSmart.setMaxTorque(1.5*abs(Right_horizontal)+15, percent);
          lowtorque=true;
        }
      }else{
        if(lowtorque){
          LeftDriveSmart.setMaxTorque(100,percent);
          RightDriveSmart.setMaxTorque(100,percent);
          lowtorque=true;
        }
      }
        



      if (abs(drivetrainLeftSideSpeed)<5) {
        if (isMovingL) {    
          LeftDriveSmart.stop(coast);
          isMovingL = false;
        }
      } else  isMovingL = true;
      

      if (abs(drivetrainRightSideSpeed)<5) {
        if (isMovingR) {
          RightDriveSmart.stop(coast);
          isMovingR = false;
        }
      } else  isMovingR = true;
      
      if (isMovingL) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      if (isMovingR) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }





      // check the ButtonL1/ButtonL2 status to control LClaw
      if (Controller1.ButtonL1.pressing()) {
        LClaw.spin(forward);
        RClaw.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        LClaw.spin(reverse);
        RClaw.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        LClaw.stop();
        RClaw.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }




      // check the ButtonR1/ButtonR2 status to control GUN
      if (Controller1.ButtonR1.pressing()) {
        Gun.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Gun.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Gun.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    if(Controller1.ButtonDown.pressing()&&Controller1.ButtonRight.pressing()){//陀螺仪初始化
      Inert_Constant.calibrate();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("calibrating");
      while(Inert_Constant.isCalibrating()){
        wait(100,msec);
      }
      Inert_Constant.setHeading(180,deg);
      wait(100,msec);
      Controller1.Screen.clearScreen();
    }
    if(Controller1.ButtonUp.pressing()&&!(Controller1.ButtonRight.pressing()))
      Inert_Constant.setHeading(180, deg);
    
    if(Controller1.ButtonX.pressing())//测试自动
      PlayAuto=true;
    else if(PlayAuto)
      PlayAuto=false;








    wait(20, msec);
  }

  
  return 0;
}
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  wait(200, msec);
  Inert_Constant.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  while (Inert_Constant.isCalibrating()){
    wait(25, msec);
  }
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);


  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}