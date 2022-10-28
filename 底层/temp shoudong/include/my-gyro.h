#ifndef MY_GYRO_H_
#define MY_GYRO_H_

#include "vex.h"
using namespace vex;
class MyGyro{
  private:
    float originalValueCurt = 0;
    float originalValuePrev = 0;
    int turnCount = 0;// clockwise is positive 
    float offset = 0;
    float calculatedValue = 0;
  public:
    MyGyro();
    void update(float originalValue);
    void reset();
    void addBias(float bias);
    float readCalculatedValue() const;
};

MyGyro::MyGyro():originalValueCurt(0),originalValuePrev(0),turnCount(0),offset(0),calculatedValue(0){}

void MyGyro::update(float originalValue){
  originalValueCurt = originalValue;
  if (originalValueCurt - originalValuePrev > 300)  turnCount--;
  else if (originalValueCurt - originalValuePrev < -300) turnCount++;
  calculatedValue = turnCount * 360 + originalValueCurt - offset;
  originalValuePrev = originalValueCurt;
}

void MyGyro::reset(){
  offset += calculatedValue;
}
void MyGyro::addBias(float bias){
  offset -= bias;
}
float MyGyro::readCalculatedValue() const{
  return calculatedValue;
}

#endif