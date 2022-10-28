#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "interfaces_and_devices.h"
#include <iostream>
using namespace std;

#define TIMER           Brain.timer(vex::timeUnits::msec)

#define Ch1             Controller1.Axis1.position(percent)
#define Ch2             Controller1.Axis2.position(percent)
#define Ch3             Controller1.Axis3.position(percent)
#define Ch4             Controller1.Axis4.position(percent)

#define BA              Controller1.ButtonA.pressing()
#define BB              Controller1.ButtonB.pressing()
#define BX              Controller1.ButtonX.pressing()
#define BY              Controller1.ButtonY.pressing()

#define L1              Controller1.ButtonL1.pressing()
#define L2              Controller1.ButtonL2.pressing()
#define R1              Controller1.ButtonR1.pressing()
#define R2              Controller1.ButtonR2.pressing()

#define UP              Controller1.ButtonUp.pressing()
#define DOWN            Controller1.ButtonDown.pressing()
#define LEFT            Controller1.ButtonLeft.pressing()
#define RIGHT           Controller1.ButtonRight.pressing()

#define getCataEncoder  (cat.position(deg))
#define getLimitValue   (lmt.value())

#define delay           vexDelay

#define sgn(x)          (x > 0 ? 1 : -1)

static int      ch_state            = 0;
static bool     lck                 = 1;
static bool     lckReset            = 0;
static bool     autoCata            = 0;
static bool     autoItk             = 0;
static bool     manual              = 1;
static bool     AutoCataInterrupt   = 0;
static int      autoRoutine         = 0;
static bool     debug               = 1;
#define rollHue (opt.hue())
#define rollMid (rollHue<10||rollHue>308)
#define rollBlue (rollHue<=220&&rollHue>=200)
#define rollRed (rollHue>=10&&rollHue<=25)
#define IMAX 30
#endif