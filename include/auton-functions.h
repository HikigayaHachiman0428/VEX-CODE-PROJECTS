#ifndef AUTON_FUNCTIONS_H_
#define AUTON_FUNCTIONS_H_

#include "PID.h"
#include "basic-functions.h"
#include "ezize.h"
#include "parameters.h"

using namespace std;
bool autoMode = false;
bool RLLockFlag = false;
float RLTarget;
void setRLTarget(int RLState) {
  if (RLState == 0) RLTarget = RLStackAngle;
  else if (RLState == 1) RLTarget = RLWaitAngle;
  else if (RLState == 2) RLTarget = RLScoreMiddleAngle;
  else if (RLState == 3) RLTarget = RLScoreLowAngle;
}

int ringLifter() {
  auto pid = PID();
  pid.setCoefficient(3.0, 0, 20);
  pid.setErrorTolerance(0);
  pid.setDTolerance(20);
  while(true){
    pid.setTarget(RLTarget);
    pid.update(getRLEncoder());
    float output = pid.getOutput();
    if (RLLockFlag) liftRing (abbs(output) > 100 ? sign(output) * 100 : output);
    delay(10);
  }
  return 1;
}

bool ChLeftLockFlag = false;
float ChLeftTarget;
void resetChLeftTarget() {ChLeftTarget = getLeftEncoder();}
int chLeft() {
  auto pid = PID();
  pid.setCoefficient(3.0, 0, 20);
  pid.setErrorTolerance(0);
  pid.setDTolerance(20);
  while(true){
    pid.setTarget(ChLeftTarget);
    pid.update(getLeftEncoder());
    float output = pid.getOutput() + getRoll();
    if (ChLeftLockFlag) moveLeft(abbs(output) > 50 ? sign(output) * 50 : output);
    delay(10);
  }
  return 1;
}

bool ChRightLockFlag = false;
float ChRightTarget;
void resetChRightTarget() {ChRightTarget = getRightEncoder();}
int chRight() {
  auto pid = PID();
  pid.setCoefficient(3.0, 0, 20);
  pid.setErrorTolerance(0);
  pid.setDTolerance(20);
  while(true){
    pid.setTarget(ChRightTarget);
    pid.update(getRightEncoder());
    float output = pid.getOutput() + getRoll();
    if (ChRightLockFlag) moveRight(abbs(output) > 50 ? sign(output) * 50 : output);
    delay(10);
  }
  return 1;
}

void lockChassis(bool flag){
  if (flag){
    resetChLeftTarget();
    ChLeftLockFlag = true;
    resetChRightTarget();
    ChRightLockFlag = true;;
  } else {
    ChLeftLockFlag = false;
    ChRightLockFlag = false;
  }
}
bool autoMogoBState;
bool autoMogoFState;

int autoMogo(){
  auto pid = PID();
  pid.setCoefficient(5.0, 0, 2);
  pid.setErrorTolerance(0);
  pid.setDTolerance(20);
  autoMogoFState = true;
  autoMogoBState = false;
  while(autoMode){
    if (getMLEncoderF() < 0)  resetMLEncoderF();
    if (getMLEncoderB() > 0)  resetMLEncoderB();
    if (autoMogoFState) {
      pid.setTarget(mogoFTakeAngle);
      pid.update(getMLEncoderF());
      liftMogoF(pid.getOutput() + 5);
    } else {
      if (getMLEncoderF() > 3) liftMogoF(-127);
      else liftMogoF(-5);
    }
    if (autoMogoBState) {
      if (getMLEncoderB() < -3)  liftMogoB(127);
      else liftMogoB(10);
    } else {
      if (getMLEncoderB() > mogoBTakeAngle) liftMogoB(-127);
      else liftMogoB(-5);
    }
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("MogoLiftFront: %.1f", getMLEncoderF());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("MogoLiftBack: %.1f", getMLEncoderB());
    delay(10);
  }
  return 1;
}
void PIDEncoderForward(float target) {
  resetChassisEncoder();
  auto myTimer = MyTimer();
  auto pid = PID();
  pid.setCoefficient(6.0, 0.10, 50);
  pid.setTarget(target * kD);
  pid.setIMax(IMAX);
  pid.setIRange(60);
  pid.setErrorTolerance(1);
  pid.setDTolerance(4);
  pid.setJumpTime(0.01);
  while (!pid.targetArrived() && myTimer.getTime() < 1000 + abbs(target * 10)) {
    pid.update(getForwardEncoder());
    moveForward(pid.getOutput());
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Encoder: %.2f", getForwardEncoder());
    delay(10);
  }
  moveForward(0);
}

void EncoderForward(float forwardValue, float encoderTarget) {
  resetChassisEncoder();
  while (abbs(getForwardEncoder()) < abbs(encoderTarget)) {
    moveForward(forwardValue);
    delay(10);
  }
  moveForward(0);
}
void GyroEncoderForward(float forwardValue, float encoderTarget,
                        float gyroTarget, bool softStart = true) {
  resetChassisEncoder();
  float kp = abbs(forwardValue) > 70 ? 2.0 : 1.5;
  float PLimit = 15;
  while (abbs(getForwardEncoder()) < abbs(encoderTarget)) {
    float gyroError = gyroTarget - getHeading();
    gyroError =
        abbs(gyroError) * kp > PLimit ? sign(gyroError) * PLimit : gyroError;
    if (abbs(getForwardEncoder()) > 3 || abbs(forwardValue) < 60 || !softStart) {
      moveLeft(forwardValue + gyroError * kp);
      moveRight(forwardValue - gyroError * kp);
    } else {
      moveForward(40 * sign(forwardValue));
    }
    delay(10);
  }
  moveForward(0);
}

void GyroTimerForward(float forwardValue, int duration, float gyroTarget) {
  float kp = abbs(forwardValue) > 70 ? 2.0 : 1.5;
  float PLimit = 15;
  auto myTimer = MyTimer();
  while (myTimer.getTime() < duration) {
    float gyroError = gyroTarget - getHeading();
    gyroError =
        abbs(gyroError) * kp > PLimit ? sign(gyroError) * PLimit : gyroError;
    moveLeft(forwardValue + gyroError * kp);
    moveRight(forwardValue - gyroError * kp);
    delay(10);
  }
  moveForward(0);
}
void TimerForward(float forwardValue, int duration) {
  moveForward(forwardValue);
  delay(duration);
  moveForward(0);
}

void PIDGyroTurn(float target) {
  // resetGyro();
  auto pid = PID();
  auto myTimer = MyTimer();
  pid.setCoefficient(2.1, 0.1, 25);
  pid.setTarget(target);
  pid.setIMax(IMAX);
  pid.setIRange(60); // 60
  pid.setErrorTolerance(2);
  pid.setDTolerance(4); // 2.6 deg / sec
  pid.setJumpTime(0.01);
  while (!pid.targetArrived() &&  myTimer.getTime() < 1000 + abbs(target * 3)) {
    pid.update(getHeading());
    moveClockwise(pid.getOutput());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Gyro: %.2f", getHeading());
    delay(10);
  }
  moveClockwise(0);
}


#endif
