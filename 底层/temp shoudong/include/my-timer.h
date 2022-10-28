#ifndef MYTIMER_H_
#define MYTIMER_H_
#include "vex.h"
using namespace vex;
class MyTimer{
  private:
  float startTime;
  public:
  MyTimer();
  MyTimer(float offset);
  void reset();
  float getTime() const;
};

MyTimer::MyTimer(){
  startTime = Brain.Timer.value();
}

MyTimer::MyTimer(float offset){//value starts from offset
  startTime = Brain.Timer.value() - offset / 1000.0;
}

void MyTimer::reset(){
  startTime = Brain.Timer.value();
}

float MyTimer::getTime() const{
  return (Brain.Timer.value() - startTime) * 1000.0;
}
#endif