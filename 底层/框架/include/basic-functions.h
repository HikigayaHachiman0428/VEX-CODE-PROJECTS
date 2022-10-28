#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "parameters.h"
#include "ezize.h"
#include "robot-config.h"
#include "my-gyro.h"

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

vex::color teamColor;
int autoChoose;



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
void printTeamColor() {
  if (teamColor == red) printController("RED  ");
  else printController("BLUE ");
}

void printAuton() {
  while (1){
  Controller1.Screen.setCursor(1, 1);
   printController( "Gyro = ");Controller1.Screen.print(getHeading()); 
  Controller1.Screen.newLine();
  printController( "Encoder = ");Controller1.Screen.print(getForwardEncoder()); 
   Controller1.Screen.newLine();
   Controller1.Screen.print(Brain.Battery.voltage());Controller1.Screen.print("V");
   delay(1);
  }
}

void printInfo() {
  clearController();
  // printTeamColor();
  // printController("\n");
  printAuton();
}
void initialize() {
  // autoChoose = 0;//3
}

#endif