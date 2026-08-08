#ifndef pin_config
#define pin_config

#include "Timer_class.h"
#include "encoder.h"
#include "CYTRON_MDD10A.hpp"
#include "PID.hpp"
#include "dc_motor.hpp"

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>  
#include <std_msgs/msg/int32.h>
rcl_timer_t pub_timer;
// subscribers object
rcl_subscription_t subscriber;
rcl_subscription_t pid_values_subscriber;
rcl_subscription_t pid_mode_subscriber;
// publishers object
rcl_publisher_t pid_publisher;
rcl_publisher_t encoder_rpm_publisher;
rcl_publisher_t encoder_count_publisher;
// messages types
std_msgs__msg__Float32MultiArray pid_msg;
std_msgs__msg__Float32MultiArray encoder_rpm_msg; 
std_msgs__msg__Float32MultiArray encoder_count_msg; 
std_msgs__msg__Float32MultiArray setpoints_msg;
std_msgs__msg__Float32MultiArray pid_values_msg; 
std_msgs__msg__Int32 pid_mode_msg;
// handles callbacks
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial.printf("RCCHECK failed: %d\n", temp_rc); while(1); }}  // Halt on error
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial.printf("RCSOFTCHECK failed: %d\n", temp_rc); } }

// 0 for normal, 1 for precision
int tune_mode = 0;
int prev_tune_mode = 0;
//r2
// ,330,10200,3.2, 300,10000,3.5 ,250,5000,5 ,270,5000,5
// normal tune 
#define KP_1_normal 150
#define KI_1_normal 5000
#define KD_1_normal 1.75

#define KP_2_normal 175
#define KI_2_normal 5250
#define KD_2_normal 0.6

#define KP_3_normal 172
#define KI_3_normal 5300
#define KD_3_normal 0.55

#define KP_4_normal 148
#define KI_4_normal 5250
#define KD_4_normal 1.5



// precision tune 
// {"data:[100, 2.0, 0.28, 90.0, 2.25, 0.42, 140.0, 2.0, 0.42, 100.0, 2.2, 0.4]"}
#define KP_1_percision 100
#define KI_1_percision 2.0
#define KD_1_percision 0.28

#define KP_2_percision 350
#define KI_2_percision 10500
#define KD_2_percision 1.2

#define KP_3_percision 350
#define KI_3_percision 10660
#define KD_3_percision 1.1

#define KP_4_percision 346
#define KI_4_percision 10500
#define KD_4_percision 0.22



#define DT 0.0333 // seconds

#define RESOLUTION 550
#define CIRCUM  0.22

#define MOTOR_1_PWM PB13
#define MOTOR_1_DIR PA10

#define MOTOR_2_PWM PA9
#define MOTOR_2_DIR PA8

#define MOTOR_3_PWM PA2
#define MOTOR_3_DIR PA5

#define MOTOR_4_PWM PA3
#define MOTOR_4_DIR PA4



#define MAX_PID_OUTPUT 65536

PID pid1(KP_1_normal , KI_1_normal ,KD_1_normal, MAX_PID_OUTPUT) ;
PID pid2(KP_2_normal , KI_2_normal ,KD_2_normal, MAX_PID_OUTPUT) ;
PID pid3(KP_3_normal , KI_3_normal ,KD_3_normal, MAX_PID_OUTPUT) ;
PID pid4(KP_4_normal , KI_4_normal, KD_4_normal, MAX_PID_OUTPUT) ;

timer tim_encoder_1(TIM2);    
timer tim_encoder_2(TIM4);
timer tim_encoder_3(TIM5);
timer tim_encoder_4(TIM3);

// //initializing encoders
Encoder enc1(tim_encoder_1 , RESOLUTION , CIRCUM) ;
Encoder enc2(tim_encoder_2 , RESOLUTION , CIRCUM) ;
Encoder enc3(tim_encoder_3 , RESOLUTION , CIRCUM) ;
Encoder enc4(tim_encoder_4 , RESOLUTION , CIRCUM) ;

// //initialize motors
CYTRON_MDD10A motor_1(MOTOR_1_DIR, MOTOR_1_PWM);
CYTRON_MDD10A motor_2(MOTOR_2_DIR, MOTOR_2_PWM);
CYTRON_MDD10A motor_3(MOTOR_3_DIR, MOTOR_3_PWM);
CYTRON_MDD10A motor_4(MOTOR_4_DIR, MOTOR_4_PWM);

//targets
float wheels_targets[4] = {0.0, 0.0, 0.0, 0.0};  // Initialized to defaults
float pid_outputs[4] = {0.0, 0.0, 0.0, 0.0};
float encoder_rpm[4] = {0.0, 0.0, 0.0, 0.0};
float encoder_count[4] = {0.0, 0.0, 0.0, 0.0};
float Kp_params[4] = {0.0, 0.0, 0.0,0.0};
float Ki_params[4] = {0.0, 0.0, 0.0,0.0};
float Kd_params[4] = {0.0, 0.0, 0.0,0.0};


void subscription_callback(const void * msgin) { 
  // type casting to use generic pointer
  const std_msgs__msg__Float32MultiArray * setpoints_msg = (const std_msgs__msg__Float32MultiArray *) msgin;
  if (setpoints_msg->data.size != 4) {
    return;  // Ignore invalid messages
  }


  wheels_targets[0]=setpoints_msg->data.data[0];
  wheels_targets[1]=setpoints_msg->data.data[1];
  wheels_targets[2]=setpoints_msg->data.data[2];
  wheels_targets[3]=setpoints_msg->data.data[3];
}


void subscription_pid_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * pid_values_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (pid_values_msg->data.size != 12) {
    return;  // Ignore invalid messages
  }

  Kp_params[0] = pid_values_msg->data.data[0];Ki_params[0] = pid_values_msg->data.data[1];Kd_params[0] = pid_values_msg->data.data[2];
  Kp_params[1] = pid_values_msg->data.data[3];Ki_params[1] = pid_values_msg->data.data[4];Kd_params[1] = pid_values_msg->data.data[5];
  Kp_params[2] = pid_values_msg->data.data[6];Ki_params[2] = pid_values_msg->data.data[7];Kd_params[2] = pid_values_msg->data.data[8];
  Kp_params[3] = pid_values_msg->data.data[9];Ki_params[3] = pid_values_msg->data.data[10];Kd_params[3] = pid_values_msg->data.data[11];

  pid1.set_params(Kp_params[0],Ki_params[0],Kd_params[0]);
  pid2.set_params(Kp_params[1],Ki_params[1],Kd_params[1]);
  pid3.set_params(Kp_params[2],Ki_params[2],Kd_params[2]);
  pid4.set_params(Kp_params[3],Ki_params[3],Kd_params[3]);
}

void subscription_pid_mode(const void * msgin)
{  
  if (msgin == NULL) return;

  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int value = msg->data;

  if (value == prev_tune_mode) return;
  prev_tune_mode = value;
  tune_mode = value;

  if (value == 1) {
    // precision mode
    pid1.set_params(KP_1_percision, KI_1_percision, KD_1_percision);
    pid2.set_params(KP_2_percision, KI_2_percision, KD_2_percision);
    pid3.set_params(KP_3_percision, KI_3_percision, KD_3_percision);
    pid4.set_params(KP_4_percision, KI_4_percision, KD_4_percision);
  } else {
    // normal mode
    pid1.set_params(KP_1_normal, KI_1_normal, KD_1_normal);
    pid2.set_params(KP_2_normal, KI_2_normal, KD_2_normal);
    pid3.set_params(KP_3_normal, KI_3_normal, KD_3_normal);
    pid4.set_params(KP_4_normal, KI_4_normal, KD_4_normal);
  }
}

void publish_pid_outputs() {  
  pid_msg.data.data[0] = pid_outputs[0];
  pid_msg.data.data[1] = pid_outputs[1];
  pid_msg.data.data[2] = pid_outputs[2];
  pid_msg.data.data[3] = pid_outputs[3];

  RCSOFTCHECK(rcl_publish(&pid_publisher, &pid_msg, NULL));
}

void publish_encoder_rpm() {  
  encoder_rpm_msg.data.data[0] = encoder_rpm[0];
  encoder_rpm_msg.data.data[1] = encoder_rpm[1];
  encoder_rpm_msg.data.data[2] = encoder_rpm[2];
  encoder_rpm_msg.data.data[3] = encoder_rpm[3];

  RCSOFTCHECK(rcl_publish(&encoder_rpm_publisher, &encoder_rpm_msg, NULL));
}

void publish_encoder_count() {  
  encoder_count_msg.data.data[0] = encoder_count[0];
  encoder_count_msg.data.data[1] = encoder_count[1];
  encoder_count_msg.data.data[2] = encoder_count[2];
  encoder_count_msg.data.data[3] = encoder_count[3];

  // encoder_count_msg.data.data[0] = wheels_targets[0];
  // encoder_count_msg.data.data[1] = wheels_targets[1];
  // encoder_count_msg.data.data[2] = wheels_targets[2];
  // encoder_count_msg.data.data[3] = wheels_targets[3];

  RCSOFTCHECK(rcl_publish(&encoder_count_publisher, &encoder_count_msg, NULL));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    
    enc1.update_speed(DT*1000);
    enc2.update_speed(DT*1000);
    enc3.update_speed(DT*1000);
    enc4.update_speed(DT*1000);
    
    encoder_count[0] = enc1.get_count();
    encoder_count[1] = enc2.get_count();
    encoder_count[2] = enc3.get_count();
    encoder_count[3] = enc4.get_count();
 
    encoder_rpm[0] = enc1.get_speed_rpm();
    encoder_rpm[1] = enc2.get_speed_rpm();
    encoder_rpm[2] = enc3.get_speed_rpm();
    encoder_rpm[3] = enc4.get_speed_rpm();

    pid_outputs[0] = pid1.compute(wheels_targets[0] , encoder_rpm[0] , DT);
    pid_outputs[1] = pid2.compute(wheels_targets[1] , encoder_rpm[1] , DT);
    pid_outputs[2] = pid3.compute(wheels_targets[2] , encoder_rpm[2] , DT);
    pid_outputs[3] = pid4.compute(wheels_targets[3] , encoder_rpm[3] , DT);

    pid_outputs[0] = constrain(pid_outputs[0],-MAX_PID_OUTPUT,MAX_PID_OUTPUT);
    pid_outputs[1] = constrain(pid_outputs[1],-MAX_PID_OUTPUT,MAX_PID_OUTPUT);
    pid_outputs[2] = constrain(pid_outputs[2],-MAX_PID_OUTPUT,MAX_PID_OUTPUT);
    pid_outputs[3] = constrain(pid_outputs[3],-MAX_PID_OUTPUT,MAX_PID_OUTPUT);

    motor_1.Move(pid_outputs[0]);
    motor_2.Move(pid_outputs[1]);
    motor_3.Move(pid_outputs[2]);
    motor_4.Move(pid_outputs[3]);

    publish_pid_outputs();
    publish_encoder_rpm();
    publish_encoder_count();
}

#endif