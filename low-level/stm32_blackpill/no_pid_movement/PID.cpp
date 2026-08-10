#include "PID.hpp"




PID::PID(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0), error(0), P(0), I(0), D(0){
    }



float PID::compute(float setpoint, float current, float dt=1) {
    error = setpoint - current;

    if ( -2 < error && error < 2) error= 0.0;  // Add Deadzone
    
    P = Kp * error;
    integral += error * dt;

    if (setpoint == 0) integral = 0;
    I = Ki * integral;

    I = ((I)<(-255)?(-255):((I)>(255)?(255):(I)));  // Add Anti Windup

    float derivative = (error - prevError) / dt;
    D = Kd * derivative;
    prevError = error;
    return P + I + D;
}


void PID::reset() {
    prevError = 0;
    integral = 0;
}

float PID::GetError(){
    return error;
}

float PID::GetP(){
    return P;
}

float PID::GetI(){
    return I;
}

float PID::GetD(){
    return D;
}

void PID::set_params(float kp ,float ki ,float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;
}


