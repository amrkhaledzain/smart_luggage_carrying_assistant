#ifndef WHEELS_H
#define WHEELS_H

#include "Timer_class.h"
#include "encoder.h"
#include "L298N.hpp"
#include "PID.hpp"
#include "dc_motor.hpp"

class Wheels {
private:
    int wheel_num;
    timer* tim_motor;  
    timer* tim_encoder;
    Encoder* enc;
    L298N* motor;
    PID* pid;

public:
    Wheels(int wheel_n);    //constructor

    float get_speed_rpm() ;
    void move(int speed) ;
    void stop();
    void pid_move(float target) ;
    void setup();
};

#endif 
