#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "definitions_and_declarations.h"
#include "interfaces_and_devices.h"
#include "tasks.h"
#include <iostream>


void driveForward(int v1,int v2) {
  v1 = v1 * 120;
  v2 = v2 * 120;
  LB.spin(fwd, v1, vex::voltageUnits::mV);
  LC.spin(fwd, v1, vex::voltageUnits::mV);
  RB.spin(fwd, -v2, vex::voltageUnits::mV);
  RC.spin(fwd, -v2, vex::voltageUnits::mV);
}

void intakePlate(float v){
  itk.spin(fwd, v, vex::voltageUnits::mV);
}
void intakeForward(float itkV, float v1,float v2, int itkE,int t){
  // itkv: :)
  // itke: 吸的rotation
  // t: 走的路
  itk.resetPosition();
  roll1.resetPosition();
  while (fabs(itk.position(deg)) < fabs(itkE)) {
    itk.spin(fwd, itkV*120, vex::voltageUnits::mV);
    delay(10);
  }
  LB.resetPosition();
  while (fabs(LB.position(deg)) < fabs(t)) {
    driveForward(v1,v2);
    delay(10);
  }
  driveForward(0,0);
  itk.spin(fwd, 0, vex::voltageUnits::mV);
}

void driveRotate(int v){
  v = v * 120;
  LB.spin(fwd, v, vex::voltageUnits::mV);
  LC.spin(fwd, v, vex::voltageUnits::mV);
  RB.spin(fwd, v, vex::voltageUnits::mV);
  RC.spin(fwd, v, vex::voltageUnits::mV);
}

void timerForward(int v, int t) {
  float startTime = Brain.timer(msec);
  while (Brain.timer(msec) - t < startTime) {
    driveForward(v,v);
  }
  driveForward(0,0);
}

void EncoderForward(float v1, float v2,float encoderTarget) {
  LB.resetPosition();
  while (fabs(LB.position(deg)) < fabs(encoderTarget)) {
    driveForward(v1,v2);
    delay(10);
  }
  driveForward(0,0);
}

void timerRotate(int v,int t) {
  float startTime = Brain.timer(msec);
  while (Brain.timer(msec) - t < startTime) {
    driveRotate(v);
  }
  driveRotate(0);
}

void EncoderRotate(float forwardValue, float encoderTarget){
  LB.resetPosition();
  while (fabs(LB.position(deg)) < fabs(encoderTarget)) {
    driveRotate(forwardValue);
    delay(10);
  }
  driveRotate(0);
}

void timerIndex(float v, int encoderTarget){
  itk.resetPosition();
  while (fabs(itk.position(deg)) < fabs(encoderTarget)) {
    itk.spin(fwd, 120*v, voltageUnits::mV);
  }
  itk.spin(fwd, 0, voltageUnits::mV);
}
void Rindex(float v,float encoderTarget){
  roll1.resetPosition();
  while (fabs(roll1.position(deg)) < fabs(encoderTarget)) {
    roll1.spin(fwd, 120*v, voltageUnits::mV);
  }
  roll1.spin(fwd, 0, voltageUnits::mV);
}

void dick(){
  double colour = opt.hue();
  while(!rollBlue){
    roll1.spin(fwd, -6000, voltageUnits::mV);
  }
  roll1.spin(fwd, 0, voltageUnits::mV);
}


void one(){
  autoCata = 1;
  dick();
  EncoderForward(-80, -80,15);
  
  EncoderRotate(-40, 300);
  delay(200);
  EncoderForward(80,80, 330);
  intakeForward(-100,15, 15,750,750);
  EncoderRotate(40, 95);
  autoCata = 1;
  delay(300);
  intakeForward(-100, 20, 20,500, 240);
  intakeForward(-100, 0, 0,200,0);
  EncoderForward(-30, -30,120);
  intakeForward(-100, 30, -20,50, 200);
  intakeForward(-100, 38, 38,850, 800);
  delay(200);
  EncoderForward(-80, -100,50);
  autoCata = 1;
  }



void two(){
  opt.brightness(true);

  EncoderForward(80, 80, 250);
  delay(300);
  EncoderRotate(80, 260);
  delay(400);
  autoCata = 1;
  EncoderForward(80, 80, 150);
  delay(600);
  dick();
  
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
void three(){}

#endif