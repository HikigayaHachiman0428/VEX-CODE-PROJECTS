#include "tasks.h"
#include "interfaces_and_devices.h"
#include "autonomous.h"
#include "GYRO.h"
#include "v5.h"
#include "v5_vcs.h"
using namespace vex;
using namespace std;

void autonomous(void) {
  switch (autoRoutine) {
  case 0:
    break;
  case 1:
    one();
    break;
  case 2:
    two();
    break;
  case 3:
    three();
    break;
  }
}

int printInfo() {
  while (1) {
    bool GYRO = 1, VOLTAGE = 1, ENCODER = 1, AUTO = 0;
    Controller1.Screen.setCursor(1, 1);
    if (GYRO) {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("Heading: ");
      Controller1.Screen.print(getHeading());
      Controller1.Screen.newLine();
    }
    if (VOLTAGE) {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("Voltage: ");
      Controller1.Screen.print(Brain.Battery.capacity());
      Controller1.Screen.newLine();
    }
    if (ENCODER) {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("ForwardEncoder: ");
      Controller1.Screen.print(getForwardEncoder());
      Controller1.Screen.newLine();
    }
    if (AUTO) {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("AUTOROUTINE: ");
      Controller1.Screen.print(autoRoutine);
      Controller1.Screen.newLine();
    }
  }
  return 1;
}

void usercontrol(void) {
  // 调试时用
  bool lastL1 = 0, lastL2 = 0, lastR1 = 0, lastR2 = 0, lastLEFT = 0,
       lastRIGHT = 0;
  manual = 1;
  AutoCataInterrupt = 0;
  lck = 1;
  lckReset = 1;
  ch_state = 2;
  while (1) {
    if (BA && debug) {
      PIDForward(1000);
    }
    if (LEFT && !lastLEFT)
      autoRoutine = autoRoutine == 0 ? 3 : autoRoutine - 1;
    if (RIGHT && !lastRIGHT)
      autoRoutine = autoRoutine == 3 ? 0 : autoRoutine + 1;
    if (manual) {
      if ((L1 && L2 && !lastL1) || (L1 && L2 && !lastL2)) {
        manual = 0;
        autoCata = 1;
        lck = 0;
      }
      if ((!L1 && (R1 || R2) && lastL1) || (L1 && !R1 && lastR1) ||
          (L1 && !R2 && lastR2)) {
        lck = 1;
        lckReset = 1;
      }
      if (L1 && (R2 || R1)) {
        lck = 0;
      }
      if (!lck) {
        cata(100 * L1 * (R2 - R1));
      }

    } else {
      if ((L1 && L2 && !lastL1) || (L1 && L2 && !lastL2)) {
        // AutoCataInterrupt = 1;
      }
    }

    ch_state = abs(Ch1) < 85 ? 1 : 0;
    // if(fabs(Ch1)<0.7){
    //   ch_state = 1;
    // }else{
    //   ch_state = 0;
    // }

    Ch();
    intake(100 * L2 * (R2 - R1));
    index(100 * L2 * (R2 - R1));
    Brain.Screen.printAt(10, 20, "LCKReset: %d", lckReset);
    Brain.Screen.printAt(10, 40, "Manual: %d", manual);
    Brain.Screen.printAt(10, 60, "lck: %d", lck);
    Brain.Screen.printAt(10, 80, "Encorder: %f", getForwardEncoder());
    Brain.Screen.printAt(10, 120, "CH1: %d", Ch1);
    Brain.Screen.printAt(10, 140, "CH_state: %d", ch_state);
    Brain.Screen.printAt(10, 160, "autoRoutine: %d", autoRoutine);
    lastL1 = L1;
    lastL2 = L2;
    lastR1 = R1;
    lastR2 = R2;
    lastLEFT = LEFT;
    lastRIGHT = RIGHT;
    delay(10);
  }
}

int main() {
  delay(200);
  task LCKON(LCK);
  task AUTOCATA(AutoCata);
  task AUTOITK(AutoIntake);
  Controller1.Screen.clearScreen();
  while (Gyro.isCalibrating()) {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("CALIBRATING... DO NOT MOVE");
  }
  thread GyroSensor(gyroSensor);
  Controller1.Screen.clearScreen();
  thread PRINT(printInfo);
  delay(200);
  usercontrol();
  // Competition.autonomous(autonomous);
  // Competition.drivercontrol(usercontrol);
  while (true) {
    delay(100);
  }
}
