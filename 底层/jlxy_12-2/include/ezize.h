#ifndef EZIZE_H_
#define EZIZE_H_

#include "my-timer.h"
#include "my-gyro.h"
using namespace vex;
#define delay this_thread::sleep_for
#define printScreen Brain.Screen.printAt
#define printController Controller1.Screen.print
#define clearController Controller1.Screen.clearLine
#define    Controller1.Screen.newLine;
#define C1 Controller1.Axis1.value()
#define C2 Controller1.Axis2.value()
#define C3 Controller1.Axis3.value()
#define C4 Controller1.Axis4.value()
#define L1 Controller1.ButtonL1.pressing()
#define L2 Controller1.ButtonL2.pressing()
#define R1 Controller1.ButtonR1.pressing()
#define R2 Controller1.ButtonR2.pressing()
#define BX Controller1.ButtonX.pressing()
#define BY Controller1.ButtonY.pressing()
#define BA Controller1.ButtonA.pressing()
#define BB Controller1.ButtonB.pressing()
#define LEFT Controller1.ButtonLeft.pressing()
#define RIGHT Controller1.ButtonRight.pressing()
#define UP Controller1.ButtonUp.pressing()
#define DOWN Controller1.ButtonDown.pressing()
float sign(float x) {
  if (x == 0)
    return 0;
  else
    return x > 0 ? 1 : -1;
}
float abbs(float x) { return x >= 0 ? x : -x; }
float deg2rad(float deg){
  return deg / 180.0 * 3.1416;
}
float rad2deg(float rad){
  return rad / 3.1416 * 180.0;
}
float sqrf(float x){
  return x * x;
}

void moveMotor(motor Motor, float speed) {
  speed = abbs(speed) > 127 ? sign(speed) * 127 : speed;
  int ratio = (int)(speed * 100);
  Motor.spin(directionType::fwd, ratio, voltageUnits::mV);
}
#endif