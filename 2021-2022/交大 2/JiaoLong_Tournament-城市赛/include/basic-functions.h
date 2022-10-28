#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "parameters.h"
#include "ezize.h"
#include "robot-config.h"

void moveLeft(float speed) {
  moveMotor(ChLF, speed);
  moveMotor(ChLB, speed);
}
// 给地盘左侧

float getLeftEncoder() {
  return ChLB.rotation(deg) / 26.317;
}

void moveRight(float speed) {
  moveMotor(ChRF, speed);
  moveMotor(ChRB, speed);
}
float getRightEncoder() {
  return ChRB.rotation(deg) / 26.317;
}
void resetLeftEncoder() {
  ChLF.resetRotation();
  ChLB.resetRotation();
}
void resetRightEncoder() {
  ChRF.resetRotation();
  ChRB.resetRotation();
}
void resetChassisEncoder() {
  resetLeftEncoder();
  resetRightEncoder();
}
void moveForward(float speed) {
  moveLeft(speed);
  moveRight(speed);
}
void moveClockwise(float speed) {
  moveLeft(speed);
  moveRight(-speed);
}
float getForwardEncoder() {
  return (getLeftEncoder() + getRightEncoder()) / 2.0;
}
void intake (float speed) {moveMotor(Intake, speed); }

void liftRing (float speed) {moveMotor(RingLift, speed);}

float getRLEncoder() {return RingLift.rotation(deg);}

void resetRLEncoder() {RingLift.resetRotation();}

void liftMogoF(float speed) {
  moveMotor(MogoLiftF, speed);
}
float getMLEncoderF() {
  return MogoLiftF.rotation(deg) * 0.2;
}
void resetMLEncoderF(){
  MogoLiftF.resetRotation();
}
void liftMogoB(float speed) {
  moveMotor(MogoLiftB, speed);
}
float getMLEncoderB() {
  return MogoLiftB.rotation(deg) * 0.2;
}
void resetMLEncoderB(){
  MogoLiftB.resetRotation();
}
vex::color teamColor;
int autoChoose;

void printTeamColor() {
  if (teamColor == red) printController("RED  ");
  else printController("BLUE ");
}

void printAuton() {
  if (autoChoose == 0){
    printController("Null     ");
  } else if (autoChoose == 1){
    printController("Left Two ");
  } else if (autoChoose == 2){
    printController("Right Two");
  } else if (autoChoose == 3){
    printController("Win Point");
  } else {}
}

void printInfo() {
  clearController(3);
  printAuton();
}
void initialize() {
  // autoChoose = 0;//3
  resetRLEncoder();
  resetMLEncoderF();
  resetMLEncoderB();
}

bool resetGyroFlag = false;
void resetGyro() { resetGyroFlag = true; }
float gyroValue = 0;
float getHeading() { return gyroValue; }
float getRoll() {return -Gyro.roll();}
float gyroBias = 0;
void addGyroBias(float bias) { gyroBias = bias; }
int gyroSensor() {
  auto myGyro = MyGyro();
  while (true) {
    myGyro.update(Gyro.heading());
    if (resetGyroFlag) {
      myGyro.reset();
      resetGyroFlag = false;
    }
    if (gyroBias != 0) {
      myGyro.addBias(gyroBias);
      gyroBias = 0;
    }
    gyroValue = myGyro.readCalculatedValue();
    delay(1);
  }
  return 1;
}


#endif