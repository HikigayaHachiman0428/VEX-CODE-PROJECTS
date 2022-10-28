#ifndef API_H
#define API_H

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

brain Brain;
controller Controller1 = controller(primary);
competition Competition;

#define _TESTXHW103_copy


#ifdef _TESTXHW103_copy
//motor LA = motor(PORT11, ratio18_1, 1);
motor LB = motor(PORT19, ratio18_1, 1);
motor LC = motor(PORT1, ratio18_1, 1);
//motor RA = motor(PORT18, ratio18_1, 1); 
motor RB = motor(PORT18, ratio18_1, 1);
motor RC = motor(PORT16,  ratio18_1, 1);

motor itk = motor(PORT5, ratio18_1, 0); 
motor cat = motor(PORT20, ratio18_1, 0);
motor cat2 = motor(PORT15, ratio18_1, 0);
motor roll1 = motor(PORT5, ratio18_1,0);

digital_in lmt = digital_in(Brain.ThreeWirePort.B);
optical opt = optical(PORT10);
inertial Gyro = inertial(PORT3) ;
#endif

#endif