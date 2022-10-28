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
// ChRF                 motor         1
// ChRB                 motor         2
// ChLB                 motor         3
// ChLF                 motor         4
// Intake               motor         12
// intakeL               motor         13
// intakeR               motor         10
// IndexL               motor         19
// IndexR               motor         20
// Gyro                 inertial      5
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

void usercontrol(void) {
  autoMode = false; 
  if (firstTime) {
    autoChoose = 0;
    firstTime = false;
  } else {
  }
  while (1) {
    // int Ch1 = abbs(C1) < 10 ? 0 : C1;
    int Ch2 = abbs(C2) < 10 ? 0 : C2;
    int Ch3 = abbs(C3) < 10 ? 0 : C3;
    int Ch4 = abbs(C4) < 10 ? 0 : C4;
    if (BX){
      PIDEncoderForward(1000);
    }
    else if (BA){
      PIDGyroTurn(90);
    }
    moveLeft(Ch3 + Ch4);
    moveRight(Ch3 - Ch4);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Gryo: %.1f", getHeading());
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Encoder: %.1f", getForwardEncoder());
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("GryoRoll: %.1f", -Gyro.roll());
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(8, 1);
    delay(1);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  delay(1000);
  firstTime = true;
  Gyro.startCalibration();
  while (Gyro.isCalibrating()) {
  }
  delay(1000);
  initialize();
  thread GyroSensor(gyroSensor);
  thread ChLeft(chLeft);
  thread ChRight(chRight);
  thread Print(printInfo);
  // Set up callbacks for autonomous and driver control periods.
  // Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
