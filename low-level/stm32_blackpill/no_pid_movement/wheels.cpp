#include "wheels.hpp"
#include "pin_confg.hpp"   

// constructor
Wheels::Wheels(int wheel_n)
    : wheel_num(wheel_n)
{
    switch (wheel_num) {
        case 1:
            tim_motor   = new timer(TIM10);
            tim_encoder = new timer(TIM2);
            enc   = new Encoder(*tim_encoder, RESOLUTION, CIRCUM);
            motor = new L298N(MOTOR_1_EN, MOTOR_1_IN1, MOTOR_1_IN2);
            pid   = new PID(1.0, 0.1, 0.01);
            break;

        case 2:
            tim_motor   = new timer(TIM1);
            tim_encoder = new timer(TIM3);
            enc   = new Encoder(*tim_encoder, RESOLUTION, CIRCUM);
            motor = new L298N(MOTOR_2_EN, MOTOR_2_IN1, MOTOR_2_IN2);
            pid   = new PID(1.2, 0.1, 0.02);
            break;

        case 3:
            tim_motor   = new timer(TIM1);
            tim_encoder = new timer(TIM4);
            enc   = new Encoder(*tim_encoder, RESOLUTION, CIRCUM);
            motor = new L298N(MOTOR_3_EN, MOTOR_3_IN1, MOTOR_3_IN2);
            pid   = new PID(1.0, 0.15, 0.02);
            break;

        case 4:
            tim_motor   = new timer(TIM1);
            tim_encoder = new timer(TIM5);
            enc   = new Encoder(*tim_encoder, RESOLUTION, CIRCUM);
            motor = new L298N(MOTOR_4_EN, MOTOR_4_IN1, MOTOR_4_IN2);
            pid   = new PID(1.1, 0.1, 0.03);
            break;

        default:
            //fallback if invalid number
            tim_motor = nullptr;
            tim_encoder = nullptr;
            enc = nullptr;
            motor = nullptr;
            pid = nullptr;
            break;
    }
}

float Wheels::get_speed_rpm()
{
  enc->update_speed();
  return enc->get_speed_rpm();
}

void Wheels::move(int speed)
{
  motor->Move(speed);
}

void Wheels::stop()
{
  motor->Stop();
}

void Wheels::pid_move(float target)
{
  enc->update_speed();
  float current_rpm =enc->get_speed_rpm();
  float output = pid->compute(target , current_rpm , dT);
  float pwm_output = constrain(output,-255,255);

  motor->Move(pwm_output);
}

void Wheels::setup()
{
  motor->Setup();
}



