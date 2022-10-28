#ifndef PID_H_
#define PID_H_

#include "ezize.h"

class PID {
private:
  float errorCurt, errorPrev, errorDev, errorInt;
  float P, I, D;
  bool firstTime;
  float kp, ki, kd;
  float target, errorTol, DTol;
  float IMax, IRange;
  bool arrived;
  float output;
  float jumpTime;
  MyTimer myTimer;

public:
  PID();
  void setCoefficient(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }
  void setTarget(float t) { target = t; }
  void setIMax(float iM) { IMax = iM; }
  void setIRange(float iR) { IRange = iR; }
  void setErrorTolerance(float eT) { errorTol = eT; }
  void setDTolerance(float DT) { DTol = DT; }
  bool targetArrived() { return arrived; }
  float getOutput() { return output; }
  void setJumpTime(float jP) { jumpTime = jP; }
  void update(float input);
};

PID::PID():firstTime(true),arrived(false),errorDev(0),errorInt(0),IMax(20),IRange(60),jumpTime(0.05) {
  myTimer.reset();
}

void PID::update(float input) {
  errorCurt = target - input;
  P = kp * errorCurt;
  if (firstTime) {
    firstTime = false;
    errorPrev = errorCurt;
  }
  errorDev = errorCurt - errorPrev;
  errorPrev = errorCurt;
  D = kd * errorDev;
  if (abbs(P) >= IRange) {
    errorInt = 0;
  } else {
    errorInt += errorCurt;
    if (abbs(errorInt) * ki > IMax)
      errorInt = sign(errorInt) * IMax / ki;
  }
  if (sign(errorInt) != sign(errorCurt) || (abbs(errorCurt) <= errorTol))
    errorInt = 0;
  I = ki * errorInt;
  if (abbs(errorCurt) <= errorTol && abbs(D) <= DTol) {
    arrived = myTimer.getTime() >= jumpTime;
  } else {
    myTimer.reset();
  }
  output = P + I + D;
  
}
#endif