#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "auton-functions.h"

const int autoNum = 3;
void LeftTwo();
void RightTwo();
void WinPoint();

void runAuton(){
  if (autoChoose == 0){
    
  } else if (autoChoose == 1){
    LeftTwo();
  } else if (autoChoose == 2){
    RightTwo();
  } else if (autoChoose == 3){
    WinPoint();
  } else {}
}

void Unfold(){
  setRLTarget(1);
  thread AutoMogo(autoMogo);
}

void LeftTwo(){
  resetGyro();
  addGyroBias(9);
  resetChassisEncoder();
  Unfold();
  GyroTimerForward(100, 200, 9);
  autoMogoFState = false;
  GyroEncoderForward(127, 90 - getForwardEncoder(), 9, false);
  PIDEncoderForward(18);
  autoMogoFState = true;
  lockChassis(true);
  delay(300);
  lockChassis(false);
  GyroEncoderForward(-80, -43 + getForwardEncoder(), 9);
  GyroEncoderForward(-80, -65, 30, false); 
  bool failure = false;
  if (getMLEncoderF() > -7) failure = true;
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
  PIDEncoderForward(-20);
  delay(2000);
}
void RightTwo(){
  resetGyro();
  resetChassisEncoder(); 
  Unfold();
  GyroTimerForward(100, 200, 0);
  autoMogoFState = false;
  GyroEncoderForward(127, 80 - getForwardEncoder(), 0, false);
  PIDEncoderForward(18);
  autoMogoFState = true;
  lockChassis(true);
  delay(300);
  lockChassis(false);
  GyroEncoderForward(-100, -75 + getForwardEncoder(), 0);
  bool failure = false;
  if (getMLEncoderF() > -7) failure = true;
  PIDEncoderForward(-20);
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
  PIDEncoderForward(-20);  
  PIDGyroTurn(-90);
  GyroEncoderForward(127, 60, -90);
  GyroEncoderForward(25, 60, -90, false);
  GyroTimerForward(20, 2500, -90);
}
void WinPoint(){
}

#endif