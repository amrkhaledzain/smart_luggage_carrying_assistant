#ifndef pin_config
#define pin_config

// A2, 
#define MOTOR_1_EN  PB13 
#define MOTOR_1_IN1 PB14 
#define MOTOR_1_IN2 PB12 
#define MOTOR_2_EN  PB8
#define MOTOR_2_IN1 PA3
#define MOTOR_2_IN2 PA2
#define MOTOR_3_EN  PA9 
#define MOTOR_3_IN1 PB15
#define MOTOR_3_IN2 PA8 
#define MOTOR_4_EN  PB1_ALT1
#define MOTOR_4_IN1 PB9
#define MOTOR_4_IN2 PB0

#define RESOLUTION 5050
#define CIRCUM  0.22

#define Kp 8
#define Kd 0
#define Ki 1.5
#define dT 0.1

#endif