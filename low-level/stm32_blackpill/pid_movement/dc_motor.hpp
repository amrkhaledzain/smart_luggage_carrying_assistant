#ifndef DC
#define DC

#include <Arduino.h>

class DCMotor {
  public: 
    virtual void Setup() = 0 ;
    virtual void Move(int Speed) = 0;
    virtual void Stop() = 0;
    virtual ~DCMotor() {}
};

#endif