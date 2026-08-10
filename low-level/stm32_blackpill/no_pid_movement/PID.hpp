#ifndef PID_HPP
#define PID_HPP

class PID {
private:
    float Kp, Ki, Kd;
    float prevError;
    float integral;
    float error;
    float P,I,D;

public:
    PID(float kp, float ki, float kd);
    float compute(float setpoint, float current, float dt);
    float GetError();
    float GetP();
    float GetI();
    float GetD();
    void reset();
    void set_params(float kp ,float ki ,float kd);
};

#endif