#ifndef TASKS_H
#define TASKS_H

//#include "GPS_functions.h"
#include "definitions_and_declarations.h"
// using namespace Eigen;

void Ch() {
  switch (ch_state) {
    /*
      case 0:
          LB.spin(fwd, 120*(Ch4*0.7+Ch2), voltageUnits::mV);
          LC.spin(fwd, 120*(Ch4*0.7+Ch2), voltageUnits::mV);
          RB.spin(fwd, 120*(Ch4*0.7-Ch2), voltageUnits::mV);
          RC.spin(fwd, 120*(Ch4*0.7-Ch2), voltageUnits::mV);
          break;
      case 1:
          LB.spin(fwd, 120*(Ch4*0.4+Ch2), voltageUnits::mV);
          LC.spin(fwd, 120*(Ch4*0.4+Ch2), voltageUnits::mV);
          RB.spin(fwd, 120*(Ch4*0.4-Ch2), voltageUnits::mV);
          RC.spin(fwd, 120*(Ch4*0.4-Ch2), voltageUnits::mV);
          break;
      case 2:
          LB.spin(fwd, 120*(Ch4+Ch2), voltageUnits::mV);
          LC.spin(fwd, 120*(Ch4+Ch2), voltageUnits::mV);
          RB.spin(fwd, 120*(Ch4-Ch2), voltageUnits::mV);
          RC.spin(fwd, 120*(Ch4-Ch2), voltageUnits::mV);
          break;*/
  case 0:
    LB.spin(fwd, 120 * (Ch1 * 0.7 + Ch3), voltageUnits::mV);
    LC.spin(fwd, 120 * (Ch1 * 0.7 + Ch3), voltageUnits::mV);
    RB.spin(fwd, 120 * (Ch1 * 0.7 - Ch3), voltageUnits::mV);
    RC.spin(fwd, 120 * (Ch1 * 0.7 - Ch3), voltageUnits::mV);
    break;
  case 1:
    LB.spin(fwd, 120 * (Ch1 * 0.4 + Ch3), voltageUnits::mV);
    LC.spin(fwd, 120 * (Ch1 * 0.4 + Ch3), voltageUnits::mV);
    RB.spin(fwd, 120 * (Ch1 * 0.4 - Ch3), voltageUnits::mV);
    RC.spin(fwd, 120 * (Ch1 * 0.4 - Ch3), voltageUnits::mV);
    break;
  case 2:
    LB.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
    LC.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
    RB.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
    RC.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
    break;
  }
}

void intake(float speed) { itk.spin(fwd, 120 * speed, voltageUnits::mV); }
int AutoItk() {
  while (1) {
    if (autoItk) {
      intake(-127);
    }
  }
  return 1;
}

void cata(float speed) {
  cat.spin(fwd, 120 * speed, voltageUnits::mV);
  cat2.spin(fwd, -120 * speed, voltageUnits::mV);
}

void index(float speed) { roll1.spin(fwd, 120 * speed, voltageUnits::mV); }

int AutoCata() {
  float startTime;
  int t = 3000;
  while (1) {
    if (autoCata) {
      lck = 0;
      startTime = Brain.timer(msec);
      cata(-100);
      delay(500);
      while (getLimitValue && (Brain.timer(msec) - t < startTime) &&
             !AutoCataInterrupt) {
        cata(-50);
        delay(10);
      }
      cata(0);
      lckReset = 1;
      lck = 1;
      autoCata = 0;
      manual = 1;
      AutoCataInterrupt = 0;
    }
    delay(10);
  }
}

int LCK() {
  lck = 1;
  lckReset = 1;
  float target = 0;
  float kp = 1; // 2
  float pow;
  while (1) {
    while (lck) {
      if (lckReset) {
        target = getCataEncoder;
        lckReset = 0;
      }
      pow =
          kp * (target - getCataEncoder); //+ sgn(target - getCataEncoder) * 20;
      cata(pow);
      Brain.Screen.printAt(10, 100, "Target: %f", target);
      delay(25);
    }
    delay(10);
  }
}

#endif