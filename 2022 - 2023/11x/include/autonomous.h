#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "GYRO.h"
#include "PID.h"
#include "TIMER.h"
#include "definitions_and_declarations.h"
#include "interfaces_and_devices.h"
#include "tasks.h"
#include <iostream>

// 底盘methods
void driveForward(int v1, int v2) {
  v1 = v1 > 127 ? sgn(v1) * 127 : v1;
  v2 = v2 > 127 ? sgn(v2) * 127 : v2;
  v1 = v1 * 120;
  v2 = v2 * 120;
  LB.spin(fwd, v1, vex::voltageUnits::mV);
  LC.spin(fwd, v1, vex::voltageUnits::mV);
  RB.spin(fwd, -v2, vex::voltageUnits::mV);
  RC.spin(fwd, -v2, vex::voltageUnits::mV);
}

void intakePlate(float v) { itk.spin(fwd, v, vex::voltageUnits::mV); }

void intakeForward(float itkV, float v1, float v2, int itkE, int t) {
  // itkv: :)
  // itke: 吸的rotation
  // t: 走的路
  itk.resetPosition();
  roll1.resetPosition();
  while (fabs(itk.position(deg)) < fabs(itkE)) {
    itk.spin(fwd, itkV * 120, vex::voltageUnits::mV);
    delay(10);
  }
  LB.resetPosition();
  while (fabs(LB.position(deg)) < fabs(t)) {
    driveForward(v1, v2);
    delay(10);
  }
  driveForward(0, 0);
  itk.spin(fwd, 0, vex::voltageUnits::mV);
}

void driveRotate(int v) {
  v = v > 127 ? 127 : v;
  v = v * 120;
  LB.spin(fwd, v, vex::voltageUnits::mV);
  LC.spin(fwd, v, vex::voltageUnits::mV);
  RB.spin(fwd, v, vex::voltageUnits::mV);
  RC.spin(fwd, v, vex::voltageUnits::mV);
}

void timerForward(int v, int t) {
  float startTime = Brain.timer(msec);
  while (Brain.timer(msec) - t < startTime) {
    driveForward(v, v);
  }
  driveForward(0, 0);
}

void EncoderForward(float v1, float v2, float encoderTarget) {
  LB.resetPosition();
  while (fabs(LB.position(deg)) < fabs(encoderTarget)) {
    driveForward(v1, v2);
    delay(10);
  }
  driveForward(0, 0);
}

void timerRotate(int v, int t) {
  float startTime = Brain.timer(msec);
  while (Brain.timer(msec) - t < startTime) {
    driveRotate(v);
  }
  driveRotate(0);
}

void EncoderRotate(float forwardValue, float encoderTarget) {
  LB.resetPosition();
  while (fabs(LB.position(deg)) < fabs(encoderTarget)) {
    driveRotate(forwardValue);
    delay(10);
  }
  driveRotate(0);
}
void resetChassisEncoder() {
  LC.resetRotation();
  LB.resetRotation();
  RC.resetRotation();
  RB.resetRotation();
}

float getForwardEncoder() {
  return (fabs(LC.rotation(deg)) + fabs(LB.rotation(deg))) / 2.0;
}

void PIDForward(float target) {
  resetChassisEncoder();
  float dt = 0.01;
  float timeroffset = TIMER;
  float timeused = 0;
  float kp = 1.0;     // 3.0 , 3.95
  float ki = 12.0;    // 28.9 , 28.6
  float kd = 0.1;     // 29.9 , 29.9
  float irange = 2.0; // ji fen fan wei
  float istart = 60;  // start to integral
  float dtol = 0.1;
  float errortolerance = 0.5; // 2.5zd
  float lim = 150;
  float error = target - getForwardEncoder();
  float lasterror;
  float v = 0;
  float i = 0;
  bool arrived;
  float timetol = fabs(error) < 20 ? 700 : fabs(error) * 24;
  float pow;
  lasterror = error;
  arrived = error == 0;
  while (!arrived) {
    timeused = TIMER - timeroffset;
    error = target - getForwardEncoder();
    v = (error - lasterror) / dt;
    if ((fabs(error) < errortolerance && fabs(v) <= dtol) ||
        timeused > timetol) {
      arrived = true;
    }
    if (fabs(i) < irange && fabs(error) < istart)
      i += sgn(error) * dt;
    if (error * lasterror <= 0)
      i = 0;
    pow = kp * error + kd * v + ki * i;
    pow = fabs(pow) > lim ? sgn(pow) * lim : pow;
    driveForward(pow, pow);
    // cout<<gyroValue()<<endl;
    // cout<<timeused<<endl;
    // printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    delay(10);
  }
  driveForward(0, 0);
}
void PIDForward(float target, float vmax) {
  resetChassisEncoder();
  float dt = 0.01;
  float timeroffset = TIMER;
  float timeused = 0;
  float kp = 1.0;     // 3.0 , 3.95
  float ki = 12.0;    // 28.9 , 28.6
  float kd = 0.1;     // 29.9 , 29.9
  float irange = 2.0; // ji fen fan wei
  float istart = 60;  // start to integral
  float dtol = 0.1;
  float errortolerance = 0.5; // 2.5zd
  float lim = vmax;
  float error = target - getForwardEncoder();
  float lasterror;
  float v = 0;
  float i = 0;
  bool arrived;
  float timetol = fabs(error) < 20 ? 700 : fabs(error) * 24;
  float pow;
  lasterror = error;
  arrived = error == 0;
  while (!arrived) {
    timeused = TIMER - timeroffset;
    error = target - getForwardEncoder();
    v = (error - lasterror) / dt;
    if ((fabs(error) < errortolerance && fabs(v) <= dtol) ||
        timeused > timetol) {
      arrived = true;
    }
    if (fabs(i) < irange && fabs(error) < istart)
      i += sgn(error) * dt;
    if (error * lasterror <= 0)
      i = 0;
    pow = kp * error + kd * v + ki * i;
    pow = fabs(pow) > lim ? sgn(pow) * lim : pow;
    driveForward(pow, pow);
    // cout<<gyroValue()<<endl;
    // cout<<timeused<<endl;
    // printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    delay(10);
  }
  driveForward(0, 0);
}

// Gyro Methods
const float kGyro = 1800.0 / 1770.0;
bool resetGyroFlag = false;
void resetGyro() { resetGyroFlag = true; }
float gyroValue = 0;
float getHeading() { return gyroValue; }
float getRoll() { return -Gyro.roll(); }
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
    // Controller1.Screen.print(gyroValue);
    delay(1);
  }
  return 1;
}

void GyroRotate(int v, int deg) {
  // resetGyro();
  while (getHeading() < deg) {
    driveRotate(v);
    delay(100);
  }
  driveRotate(0);
}

void PIDGyroTurn(float target) {
  resetGyro();
  auto pid = PID();
  auto myTimer = MyTimer();
  // 觉得慢了，调第一个；觉得冲不动了，调第二个；觉得冲过头了，调大第三个
  pid.setCoefficient(2.1, 0.5, 20);
  pid.setTarget(target);
  pid.setIMax(IMAX);
  pid.setIRange(60); // 60
  pid.setErrorTolerance(1);
  pid.setDTolerance(4); // 2.6 deg / sec
  pid.setJumpTime(0.01);
  while (!pid.targetArrived() && myTimer.getTime() < 1000 + fabs(target * 3)) {
    pid.update(getHeading());
    driveRotate(pid.getOutput());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("                           ");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Gyro: %.2f", getHeading());
    delay(10);
  }
  driveRotate(0);
}

void GyroEncoderForward(float forwardValue, float encoderTarget,
                        float gyroTarget, bool softStart = true) {
  resetChassisEncoder();
  float kp = fabs(forwardValue) > 70 ? 2.0 : 1.5;
  float PLimit = 15;
  while (fabs(getForwardEncoder()) < fabs(encoderTarget)) {
    float gyroError = gyroTarget - getHeading();
    gyroError =
        fabs(gyroError) * kp > PLimit ? sgn(gyroError) * PLimit : gyroError;
    if (fabs(getForwardEncoder()) > 3 || fabs(forwardValue) < 60 ||
        !softStart) {
      driveForward((forwardValue + gyroError * kp),
                   (forwardValue - gyroError * kp));
    } else {
      driveForward(40 * sgn(forwardValue), 40 * sgn(forwardValue));
    }
    delay(10);
  }
  driveForward(0, 0);
}

void GyroTimerForward(float forwardValue, int duration, float gyroTarget) {
  float kp = fabs(forwardValue) > 70 ? 2.0 : 1.5;
  float PLimit = 15;
  auto myTimer = MyTimer();
  while (myTimer.getTime() < duration) {
    float gyroError = gyroTarget - getHeading();
    gyroError =
        fabs(gyroError) * kp > PLimit ? sgn(gyroError) * PLimit : gyroError;
    driveForward((forwardValue + gyroError * kp),
                 (forwardValue - gyroError * kp));
    delay(10);
  }
  driveForward(0, 0);
}

// 赛季特别方法
void timerIndex(float v, int encoderTarget) {
  itk.resetPosition();
  while (fabs(itk.position(deg)) < fabs(encoderTarget)) {
    itk.spin(fwd, 120 * v, voltageUnits::mV);
  }
  itk.spin(fwd, 0, voltageUnits::mV);
}

void Rindex(float v, float encoderTarget) {
  roll1.resetPosition();
  while (fabs(roll1.position(deg)) < fabs(encoderTarget)) {
    roll1.spin(fwd, 120 * v, voltageUnits::mV);
  }
  roll1.spin(fwd, 0, voltageUnits::mV);
}

void dick(bool blue) {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(rollHue);
  if (blue) {
    while (!rollRed) {
      roll1.spin(fwd, -9000, voltageUnits::mV);
    }
    while (!rollBlue) {
      roll1.spin(fwd, -9000, voltageUnits::mV);
    }
    roll1.spin(fwd, 0, voltageUnits::mV);
  } else {
    while (!rollBlue) {
      roll1.spin(fwd, -8500, voltageUnits::mV);
    }
    while (!rollRed) {
      roll1.spin(fwd, -8500, voltageUnits::mV);
    }
    roll1.spin(fwd, 0, voltageUnits::mV);
  }
}

// 自动线路
void one() {
  autoCata = 1;
  // dick();
  EncoderForward(-80, -80, 15);

  EncoderRotate(-40, 300);
  delay(200);
  EncoderForward(80, 80, 330);
  intakeForward(-100, 15, 15, 750, 750);
  EncoderRotate(40, 95);
  autoCata = 1;
  delay(300);
  intakeForward(-100, 20, 20, 500, 240);
  intakeForward(-100, 0, 0, 200, 0);
  EncoderForward(-30, -30, 120);
  intakeForward(-100, 30, -20, 50, 200);
  intakeForward(-100, 38, 38, 850, 800);
  delay(200);
  EncoderForward(-80, -100, 50);
  autoCata = 1;
}

void two() {
  opt.brightness(true);

  EncoderForward(80, 80, 250);
  delay(300);
  EncoderRotate(80, 260);
  delay(400);
  autoCata = 1;
  EncoderForward(80, 80, 150);
  delay(600);
  // dick();

  delay(400);

  EncoderForward(-40, -40, 30);
  delay(400);
  EncoderRotate(40, 320);

  intakeForward(-100, 40, 40, 400, 400);
  intakeForward(-100, 40, 40, 500, 550);
  intakeForward(-100, 45, 40, 600, 800);
  intakeForward(-100, -40, 40, 400, 250);
  delay(400);
  autoCata = 1;
}
void three() {}

#endif