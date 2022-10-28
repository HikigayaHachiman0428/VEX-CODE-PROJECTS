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
  int autocollide = 0;
  bool mogoFState = false;
  bool mogoBState = true;
  int RLState = 1;
  bool RLInitialized = true;
  RLLockFlag = true;
  bool ClimbMode = false;
  bool L1Pressed = false;
  bool L2Pressed = false;
  bool BBPressed = false;
  bool BAPressed = false;
  bool BXPressed = false;
  bool BYPressed = false;
  bool UPPressed = false;
  bool LEFTPressed = false;
  bool RIGHTPressed = false;
  bool DOWNPressed = false;
  bool MAGSSTATE = true;
  MagValveF.on();
  MagValveS.off();
  if (firstTime) {
    autoChoose = 0;
    firstTime = false;
    RLInitialized = false;
    RLState = 0;
  } else {
    RLInitialized = true;
    RLState = 1;
  }
  printInfo();
  while (1) {
    int Ch1 = abbs(C1) < 10 ? 0 : C1;
    int Ch2 = abbs(C2) < 10 ? 0 : C2;
    int Ch3 = abbs(C3) < 10 ? 0 : C3;
    if (!ClimbMode){
      if ((abbs(Ch3) < 30 || sign(Ch3) == sign(Ch2)) && abbs(Ch2) > 80 &&
          abbs(Ch2) > abbs(Ch1) + 40) {
        autocollide = sign(Ch2);
      } else if ((abbs(Ch3) >= 50 && !(Ch3 > 0 && autocollide == 1)) ||
                (abbs(Ch2) < abbs(Ch1) && abbs(Ch1) > 40)) {
        autocollide = 0;
      }
      if (autocollide == 0) {
        moveLeft(Ch3 + Ch1);
        moveRight(Ch3 - Ch1);
      } else {
        moveLeft(autocollide * 20);
        moveRight(autocollide * 20);
      }
    } else {
      if (abbs(Ch3 + Ch1) > 20) {
        ChLeftLockFlag = false;
        moveLeft(Ch3 + Ch1 + getRoll());
      } else if (!ChLeftLockFlag){
        ChLeftLockFlag = true;
        resetChLeftTarget();
      }
      if (abbs(Ch3 - Ch1) > 20) {
        ChRightLockFlag = false;
        moveRight(Ch3 - Ch1 + getRoll());
      } else if (!ChRightLockFlag){
        ChRightLockFlag = true;
        resetChRightTarget();
      }
    }
    if (R1) intake(127);
    else if (R2) intake(-127);
    else intake(0);

    if (RLInitialized){
      if (getMLEncoderB() < mogoBTakeAngle + 10){
        if (BB) {
          RLLockFlag = false;
          liftRing(-10);
          if (getRLEncoder() < 0) resetRLEncoder();
        } else if (BBPressed) {RLLockFlag = true; RLState = 0;}
      } else {
        if (BB && RLState != 0) {RLLockFlag = true; RLState = 1;}
      }
    } else {
      if (BB) {
        RLLockFlag = false;
        liftRing(-10);
        if (getRLEncoder() < 0) resetRLEncoder();
      } else if (BBPressed) {RLLockFlag = true; RLState = 0; RLInitialized = true;}
    }
    if (!BAPressed && BA) {RLState = 1;}
    if (getMLEncoderF() < mogoFTakeAngle + 5){
      if (!BYPressed && BY) {RLState = 2;}
      else if (!BXPressed && BX) {RLState = 3;}
    }
    setRLTarget(RLState);

    if (!L1Pressed && L1) {
      if (!mogoFState && getRLEncoder() > RLWaitAngle + 15) RLState = RLState >= 2 ? 1 : RLState; 
      else mogoFState = !mogoFState;
    }
    if (!L2Pressed && L2) {
      if (!mogoBState && getRLEncoder() < RLWaitAngle - 15) RLState = RLState == 0 ? 1 : RLState;
      else mogoBState = !mogoBState;
    }
    if (getMLEncoderF() < 0)  resetMLEncoderF();
    if (getMLEncoderB() > 0)  resetMLEncoderB();
    if (mogoFState) {
      if (getMLEncoderF() < mogoFTakeAngle-5) { 
        liftMogoF(L1 ? 80 : 127);
        MagValveF.off();
      }
      else liftMogoF(10);
    } else {
      if (getMLEncoderF() > 5) {
        liftMogoF(L1 ? -50 : -127);
        if(!ClimbMode) MagValveF.on();
      }
      else liftMogoF(-5);
    }
    if (mogoBState) {
      if (getMLEncoderB() < -3)  liftMogoB(L2 ? 70 : 127);
      else liftMogoB(10);
    } else {
      if (getMLEncoderB() > mogoBTakeAngle) liftMogoB(L2 ? -60 : -127);
      else liftMogoB(-5);
    }

    // if (LEFT && !LEFTPressed && autoChoose > 0) {autoChoose--; printInfo();}
    // if (RIGHT && !RIGHTPressed && autoChoose < autoNum) {autoChoose++; printInfo();}
    if (RIGHT && !RIGHTPressed) {autoChoose++; autoChoose%=autoNum ;printInfo();}
    if (LEFT && !LEFTPressed) {MAGSSTATE=!MAGSSTATE;}
    if (MAGSSTATE) MagValveS.off();
    else MagValveS.on();
    if (UP && !UPPressed){
      ChLeftLockFlag = false;
      ChRightLockFlag = false;
      RLState = 1;
      mogoFState = true;
      mogoBState = true;
      autoMode = true;
      runAuton();

      autoMode = false;
    }
    if (!ClimbMode){
      if ((!DOWNPressed && DOWN ) || abbs(getRoll()) > 10) {
        ClimbMode = true;
        ChLeftLockFlag = false;
        ChRightLockFlag = false;
        autocollide = false;
      }
    } else {
      if (!DOWNPressed && DOWN) {
        ClimbMode = false;
        ChLeftLockFlag = false;
        ChRightLockFlag = false;
        autocollide = false;
      }
    }
    if (BB) resetGyro();
    L1Pressed = L1;
    L2Pressed = L2;
    BBPressed = BB;
    BAPressed = BA;
    BXPressed = BX;
    BYPressed = BY;
    UPPressed = UP;
    LEFTPressed = LEFT;
    RIGHTPressed = RIGHT;
    DOWNPressed = DOWN;

    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("MogoLiftFront: %.1f", getMLEncoderF());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("MogoLiftBack: %.1f", getMLEncoderB());
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("RingLift: %.1f", getRLEncoder());
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Gryo: %.1f", getHeading());
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Encoder: %.1f", getForwardEncoder());
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("GryoRoll: %.1f", -Gyro.roll());
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("MogoLiftFront_Amp: %.1f", MogoLiftF.current(amp));

    delay(1);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  MagValveF.on();
  MagValveS.off();
  delay(1000);
  firstTime = true;
  Gyro.startCalibration();
  while (Gyro.isCalibrating()) {
  }
  delay(1000);
  initialize();
  thread GyroSensor(gyroSensor);
  thread RingLifter(ringLifter);
  thread ChLeft(chLeft);
  thread ChRight(chRight);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
