#include "PID.hpp"
#include "math.h"
PID::PID(float kp, float ki, float kd, int max_pid_output)
    : Kp(kp), Ki(ki), Kd(kd), max_pid_output(max_pid_output),prevError(0), integral(0), error(0), P(0), I(0), D(0){
    }

float PID::compute(float setpoint, float current, float dt=1) {
    error = setpoint - current;
    // if ( -0.01*setpoint < error && error < 0.01*setpoint) error= 0.0;  // Add Deadzone
    if(abs(error)<dead_zone && abs(setpoint) < 1) {
        error =0; 
        integral=0;

    }
    P = Kp * error;
    integral += error * dt;

    if (setpoint == 0) integral = 0;
    I = Ki * integral;

    I = ((I)<(-max_pid_output)?(-max_pid_output):((I)>(max_pid_output)?(max_pid_output):(I)));  // Add Anti Windup

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