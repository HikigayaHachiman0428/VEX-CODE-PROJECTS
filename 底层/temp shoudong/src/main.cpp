/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\37959                                            */
/*    Created:      Tue Sep 01 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// ChRF                 motor         19
// ChRB                 motor         11
// ChLB                 motor         20
// ChLF                 motor         12
// Intake               motor         14
// MoboFr               motor         13
// MoboBa               motor         18
// Ringlift             motor         03
// IndexR               motor         20
// Gyro                 inertial      17
// Optical              optical       18
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "parameters.h"
#include "basic-functions.h"
#include "autonomous.h"
#include "ezize.h"
using namespace vex;
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  autoMode = true;
  ChLeftLockFlag = false;
  ChRightLockFlag = false;
  runAuton();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool firstTime = true;

int Left_verticle;
int left_horizontal;
void usercontrol(void) {
  autoMode = false; 
  bool lowtorque = false;
 Left_verticle=Controller1.Axis3.position();
  left_horizontal=Controller1.Axis4.position();
  //控制左右
  int right_verticle=Controller1.Axis2.position();
  //控制前后
    int drivetrainLeftSideSpeed =Left_verticle + left_horizontal;
    int drivetrainRightSideSpeed =Left_verticle - left_horizontal;
    if (!lowtorque){
        ChRF.setMaxTorque(100, percent);
        ChRB.setMaxTorque(100, percent);
        ChLF.setMaxTorque(100, percent);
        ChLB.setMaxTorque(100, percent);
        lowtorque = true;
    } 
      double Lift_v = right_verticle; 
    if(fabs(Lift_v)>5){
      RingLift.setVelocity(Lift_v, percent);
      RingLift.spin(fwd);
    }else{
      RingLift.stop(hold);
    }
    bool isMovingL = false; bool isMovingR = false;
    if (abs(drivetrainLeftSideSpeed)<10) {
      if (isMovingL) {    
        ChLF.stop(brake);
        ChLB.stop(brake);
        isMovingL = false;
      }
    } else  
        isMovingL = true;
    

    if (abs(drivetrainRightSideSpeed)<10) {
      if (isMovingR) {
        ChRB.stop(brake);
        ChRF.stop(brake);
        isMovingR = false;
      }
    }  else  
        isMovingR = true;
    
    if (isMovingL) {
      ChLB.setVelocity(drivetrainLeftSideSpeed, percent);
      ChLB.spin(fwd);
      ChLF.setVelocity(drivetrainLeftSideSpeed, percent);
      ChLF.spin(fwd);
    }
    if (isMovingR) {
      ChRB.setVelocity(drivetrainRightSideSpeed, percent);
      ChRB.spin(fwd);
      ChRF.setVelocity(drivetrainRightSideSpeed, percent);
      ChRF.spin(fwd);
    }
    if(Controller1.ButtonR1.pressing()){
      MogoLiftB.setVelocity(100, percent);
      MogoLiftB.spin(fwd);
    }
    else if(Controller1.ButtonR2.pressing()){
      MogoLiftB.setVelocity(-100, percent);
      MogoLiftB.spin(fwd);
    }
    else
      MogoLiftB.stop(brake);

    if(Controller1.ButtonL1.pressing()){
      MogoLiftF.setVelocity(100, percent);
      MogoLiftF.spin(fwd);
    }
    else if(Controller1.ButtonL2.pressing()){
      MogoLiftF.setVelocity(-100, percent);
      MogoLiftF.spin(fwd);
    }
    else
      MogoLiftF.stop(brake);
    if (Controller1.ButtonB.pressing())
        ringLifter();
    
    // if  (Controller1.ButtonA.pressing())  c_up_lift();
    // if (Controller1.ButtonB.pressing()) c_down_lift();
    // if  (Controller1.ButtonUp.pressing()) c_fast_up_clawopen();
    // if (Controller1.ButtonDown.pressing()) c_fast_down_clawopen();
    // if (Controller1.ButtonLeft.pressing()) c_fast_up_clawclose();
    // if (Controller1.ButtonRight.pressing()) c_fast_down_clawclose();
    // wait before repeating the process
    if(Controller1.ButtonLeft.pressing()&&Controller1.ButtonRight.pressing()){//陀螺仪+爪子编码器初始化
      Gyro.calibrate();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("calibrating");
      while(Gyro.isCalibrating()){
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("calibrating");
        wait(100,msec);
      }
      Gyro.setHeading(180,deg);
      wait(100,msec);
      Controller1.Screen.clearScreen();
      RingLift.setPosition(0, degrees);
      MogoLiftB.setPosition(0, degrees);
      MogoLiftF.setPosition(0, degrees);
    }
    if(Controller1.ButtonLeft.pressing()){
      Gyro.setHeading(180, deg);
      RingLift.setPosition(0, degrees);
      MogoLiftB.setPosition(0, degrees);
      MogoLiftF.setPosition(0, degrees);
    }
    bool PlayAuto = false;
    if(Controller1.ButtonX.pressing()){//测试自动
      autoChoose=1;
      runAuton();
      PlayAuto = true;
      while(PlayAuto) 
        wait(500000,msec);
        PlayAuto = false;
    }
    wait(1,msec);
  }

//
// Main will set up the competition functions and callbacks.
//
int main() {
  bool test_mode = true;
  delay(1000);
  firstTime = true;
  Gyro.startCalibration();
  while (Gyro.isCalibrating()) {
  }
  delay(100);  
  initialize();
  // thread GyroSensor(gyroSensor);
  // thread RingLifter(ringLifter);
  // thread ChLeft(chLeft);
  // thread ChRight(chRight);
  // Set up callbacks for autonomous and driver control periods.
  Controller1.Screen.clearScreen();
  while(test_mode){
    usercontrol();
    Controller1.Screen.print(Left_verticle);
    Controller1.Screen.print(Left_verticle);
  }
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}