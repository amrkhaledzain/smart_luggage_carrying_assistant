#ifndef MOTOR_DRIVER_1
#define MOTOR_DRIVER_1

#include <Arduino.h>
#include "dc_motor.hpp"

class L298N: public DCMotor {
  private:
    int Pin1;
    int Pin2;
    int ENA;

  public:
    L298N(int ena, int pin1, int pin2);
    void Setup() override;
    void Move(int Speed) override;
    void Stop() override;
};

#endif 