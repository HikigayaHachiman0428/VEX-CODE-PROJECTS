#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "auton-functions.h"

const int autoNum = 4;
void LeftTwo();
void RightTwo();
void LeftWP();
void RightMid();

void runAuton(){
  if (autoChoose == 0){
    
  } else if (autoChoose == 1){
    LeftTwo();
  } else if (autoChoose == 2){
    RightTwo();
  } else if (autoChoose == 3){
    RightMid();
  }
}

void Unfold(){
  setKickStand(false);
  setFrontLock(false);
  setRLTarget(1);
  thread AutoMogo(autoMogo);
}

void LeftTwo(){
  auto myTimer = MyTimer();
  resetGyro();
  addGyroBias(9);
  resetChassisEncoder();
  Unfold();
  GyroTimerForward(100, 200, 9);
  autoMogoFState = false;
  GyroEncoderForward(127, 90 - getForwardEncoder(), 9, false);
  autoMogoFState = true;
  setFrontLock(true);
  PIDEncoderForward(8);
  // lockChassis(true);
  // delay(300);
  // lockChassis(false);
  GyroEncoderForward(-80, -43 + getForwardEncoder(), 9);
  GyroEncoderForward(-80, -65, 30, false); 
  bool failure = false;
  if (getMLEncoderF() > mogoFTakeAngle - 3) failure = true;
  PIDEncoderForward(-25 + failure * 10);
  PIDGyroTurn(LeftTwoTurn);
  GyroEncoderForward(-80, LeftTwoBack, LeftTwoTurn);
  PIDEncoderForward(-10 - failure * 10);
  autoMogoBState = true;
  delay(300);
  intake(127);
  GyroEncoderForward(100, 20, LeftTwoTurn);
  PIDEncoderForward(10);
  delay(150);
  intake(0);
  PIDGyroTurn(-180);
  PIDEncoderForward(-30 - failure * 15);
  intake(127);
  delay(800);
  GyroEncoderForward(30, LeftTwoForward, -180);
  PIDEncoderForward(5);
  delay(100);
  PIDEncoderForward(-40);
  delay(800);
  GyroEncoderForward(30, 35, -180);
  PIDEncoderForward(5);
  delay(100);
  while (myTimer.getTime() < 14700) {delay(1);}
  TimerForward(-80, 200);
  autoMogoFState = false;
  autoMogoBState = false;
  TimerForward(0,100);
}
void RightTwo(){
  auto myTimer = MyTimer();
  resetGyro();
  resetChassisEncoder(); 
  Unfold();
  GyroTimerForward(100, 200, 0);
  autoMogoFState = false;
  GyroEncoderForward(127, 80 - getForwardEncoder(), 0, false);
  autoMogoFState = true;
  setFrontLock(true);
  PIDEncoderForward(8);
  // lockChassis(true);
  // delay(300);
  // lockChassis(false);
  GyroEncoderForward(-100, -85 + getForwardEncoder(), 0);
  PIDEncoderForward(-10);
  bool failure = false;
  if (getMLEncoderF() > mogoFTakeAngle - 3) failure = true;
  PIDGyroTurn(-90 - failure * 15);
  GyroEncoderForward(-60, -25 - failure * 10, -90 - failure * 15);
   autoMogoBState = true;
  PIDEncoderForward(-8);
  PIDGyroTurn(-35);
  intake(127);
  
  if (failure) GyroEncoderForward(35, 15, -35);
  GyroEncoderForward(35, 90 - failure * 15, 0);
  PIDEncoderForward(10);
  GyroEncoderForward(-100, -45 + failure * 3, 0);
  PIDEncoderForward(-22);  
  PIDGyroTurn(-90);
  GyroEncoderForward(127, 60, -90);
  moveForward(25);
  while (myTimer.getTime() < 14500) {delay(1);}
  TimerForward(-127, 400);
  autoMogoFState = false;
  autoMogoBState = false;
  TimerForward(0,100);

}

void RightMid(){
  auto myTimer = MyTimer();
  resetGyro();
  addGyroBias(-31.5);
  resetChassisEncoder();
  Unfold();
  GyroTimerForward(100, 200, -31.5);
  autoMogoFState = false;
  GyroEncoderForward(127, 115 - getForwardEncoder(), -31.5, false);
  autoMogoFState = true;
  setFrontLock(true);
  PIDEncoderForward(8);
  GyroEncoderForward(-110, -90 + getForwardEncoder(), -31.5);
  bool failure = false;
  if (getMLEncoderF() > mogoFTakeAngle - 5) failure = true;
  PIDEncoderForward(-40 + 25 * failure);
  PIDGyroTurn(-90);
  GyroEncoderForward(-80, -50 - failure * 5, -90);

  autoMogoBState = true;
  PIDEncoderForward(-8);
  PIDGyroTurn(-35);
  intake(127);
  
  if (failure) GyroEncoderForward(35, 15, -35);
  GyroEncoderForward(35, 90 - failure * 15, 0);
  PIDEncoderForward(10);
  GyroEncoderForward(-100, -45, 0);
  PIDEncoderForward(-25);  
  PIDGyroTurn(-90);
  GyroEncoderForward(127, 60, -90);
  moveForward(25);
  while (myTimer.getTime() < 14500) {delay(1);}
  TimerForward(-127, 400);
  autoMogoFState = false;
  autoMogoBState = false;
  TimerForward(0,100);
}

#endif