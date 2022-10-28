#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "parameters.h"
#include "ezize.h"
#include "robot-config.h"

void moveLeft(float speed) {
  moveMotor(ChLF, speed);
  moveMotor(ChLB, speed);
}
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

void setFrontLock(bool x) {
  if (x)  MagValveF.off();
  else MagValveF.on();
}

void setKickStand(bool x) {
  if (x)  MagValveS.on();
  else MagValveS.off();
}
vex::color teamColor;
int autoChoose;

void initialize() {
  autoChoose = 0;
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
    gyroValue = myGyro.readCalculatedValue() * kGyro;
    delay(1);
  }
  return 1;
}

void printAuton() {
  while (1){
  clearController();
  Controller1.Screen.setCursor(1, 1);
   printController( "Gyro = ");Controller1.Screen.print(getHeading()); 
  Controller1.Screen.newLine();
  printController( "Encoder = ");Controller1.Screen.print(getForwardEncoder()); 
   Controller1.Screen.newLine();
   Controller1.Screen.print(Brain.Battery.voltage());Controller1.Screen.print("V");
   delay(10);
  }
}

void printInfo() {
  // printTeamColor();
  printAuton();
}
#endif