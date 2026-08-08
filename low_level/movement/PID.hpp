#ifndef PID_HPP
#define PID_HPP
#define dead_zone 5

class PID {
private:
    float Kp, Ki, Kd;
    float prevError;
    float integral;
    float error;
    float P,I,D;
    int max_pid_output;

public:
    PID(float kp, float ki, float kd, int max_pid_output);
    float compute(float setpoint, float current, float dt);
    float GetError();
    float GetP();
    float GetI();
    float GetD();
    void reset();
    void set_params(float kp ,float ki ,float kd);
};

#endif