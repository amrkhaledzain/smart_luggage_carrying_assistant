#ifndef MOTOR_DRIVER_2
#define MOTOR_DRIVER_2

#include <Arduino.h>
#include "dc_motor.hpp"

class CYTRON_MDD10A: public DCMotor {
  private:
    int DIR;
    int PWM;

  public:
    CYTRON_MDD10A(int dir, int pwm);
    void Setup() override;
    void Move(int Speed) override;
    void Stop() override;
};

#endif