#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>  // <-- changed to Float32MultiArray
#include <wheels.hpp>
#include <pin_confg.hpp>

rcl_subscription_t subscriber;      // subscriber object
rcl_subscription_t pid_values_subscriber;      // subscriber object
rcl_publisher_t pid_publisher;    // pid publisher object
rcl_publisher_t encoder_rpm_publisher;    // pid publisher object
rcl_publisher_t encoder_count_publisher;    // pid publisher object
std_msgs__msg__Float32MultiArray pid_msg;  // message type for pid 
std_msgs__msg__Float32MultiArray encoder_rpm_msg;  // message type for pid 
std_msgs__msg__Float32MultiArray encoder_count_msg;  // message type for pid 
std_msgs__msg__Float32MultiArray setpoints_msg;     // message type for subscriber
std_msgs__msg__Float32MultiArray pid_values_msg;  // message type for pid 
rclc_executor_t executor;           // handles callbacks
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define Kp 38
#define Ki 0.2
#define Kd 0.016

#define dT 0.1

PID pid1(Kp , Ki ,Kd) ;
PID pid2(Kp , Ki ,Kd) ;
PID pid3(Kp , Ki ,Kd) ;
PID pid4(Kp , Ki ,Kd) ;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial.printf("RCCHECK failed: %d\n", temp_rc); while(1); }}  // Halt on error
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial.printf("RCSOFTCHECK failed: %d\n", temp_rc); } }

timer tim_encoder_1(TIM2);//     TIM5
timer tim_encoder_2(TIM3);//TIM5
timer tim_encoder_3(TIM4);//TIM2
timer tim_encoder_4(TIM5);//TIM3

//initializing encoders
Encoder enc1(tim_encoder_1 , RESOLUTION , CIRCUM) ;
Encoder enc2(tim_encoder_2 , RESOLUTION , CIRCUM) ;
Encoder enc3(tim_encoder_3 , RESOLUTION , CIRCUM) ;
Encoder enc4(tim_encoder_4 , RESOLUTION , CIRCUM) ;

//initialize motors
L298N motor_1(MOTOR_1_EN , MOTOR_1_IN1 , MOTOR_1_IN2);
L298N motor_2(MOTOR_2_EN , MOTOR_2_IN1 , MOTOR_2_IN2);
L298N motor_3(MOTOR_3_EN , MOTOR_3_IN1 , MOTOR_3_IN2);
L298N motor_4(MOTOR_4_EN , MOTOR_4_IN1 , MOTOR_4_IN2);


//targets
float wheels_targets[4] = {0.0, 0.0, 0.0, 0.0};  // Initialized to defaults
float pid_outputs[4] = {0.0, 0.0, 0.0, 0.0};
float encoder_rpm[4] = {0.0, 0.0, 0.0, 0.0};
float encoder_count[4] = {0.0, 0.0, 0.0, 0.0};
float pid_tunning_pararms[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};


void subscription_callback(const void * msgin) { 
  // type casting to use generic pointer
  const std_msgs__msg__Float32MultiArray * setpoints_msg = (const std_msgs__msg__Float32MultiArray *) msgin;
  if (setpoints_msg->data.size != 4) {
    return;  // Ignore invalid messages
  }
  wheels_targets[0] = setpoints_msg->data.data[0];
  wheels_targets[1] = setpoints_msg->data.data[1];
  wheels_targets[2] = setpoints_msg->data.data[2];
  wheels_targets[3] = setpoints_msg->data.data[3];
}


void subscription_pid_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * pid_values_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (pid_values_msg->data.size != 12) {
    return;  // Ignore invalid messages
  }

  pid_tunning_pararms[0] = pid_values_msg->data.data[0];
  pid_tunning_pararms[1] = pid_values_msg->data.data[1];
  pid_tunning_pararms[2] = pid_values_msg->data.data[2];
  pid_tunning_pararms[3] = pid_values_msg->data.data[3];

  pid1.set_params(pid_values_msg->data.data[0],pid_values_msg->data.data[1],pid_values_msg->data.data[2]);
  pid2.set_params(pid_values_msg->data.data[3],pid_values_msg->data.data[4],pid_values_msg->data.data[5]);
  pid3.set_params(pid_values_msg->data.data[6],pid_values_msg->data.data[7],pid_values_msg->data.data[8]);
  pid4.set_params(pid_values_msg->data.data[9],pid_values_msg->data.data[10],pid_values_msg->data.data[11]);
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

void setup() {
  //Serial.begin(115200);  // For debugging
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Initialize subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/wheel_setpoints"
  ));


  // Initialize subscriber
  RCCHECK(rclc_subscription_init_default(
    &pid_values_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/pid_tunning_values"
  ));

  // Initialize pid publisher
  RCCHECK(rclc_publisher_init_default(
    &pid_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "pid_outputs"
  ));

  // Initialize pid publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_rpm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_speed"
  ));

  // Initialize pid publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_count_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_counts"
  ));


  motor_1.Setup();
  motor_2.Setup();
  motor_3.Setup();
  motor_4.Setup();

  pid_values_msg.data.data = (float *)malloc(12 * sizeof(float));
  pid_values_msg.data.size = 12;  
  pid_values_msg.data.capacity = 12;

  pid_msg.data.data = (float *)malloc(4 * sizeof(float));
  pid_msg.data.size = 4;  
  pid_msg.data.capacity = 4;

  encoder_rpm_msg.data.data = (float *)malloc(4 * sizeof(float));
  encoder_rpm_msg.data.size = 4;  
  encoder_rpm_msg.data.capacity = 4;

  encoder_count_msg.data.data = (float *)malloc(4 * sizeof(float));
  encoder_count_msg.data.size = 4;  
  encoder_count_msg.data.capacity = 4;

  setpoints_msg.data.data = (float *)malloc(4 * sizeof(float));
  setpoints_msg.data.size = 4;
  setpoints_msg.data.capacity = 4;

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &setpoints_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_values_subscriber, &pid_values_msg, &subscription_pid_callback, ON_NEW_DATA));
  
}
void loop() {
  static uint32_t last_print = 0;

  // run motor control every 100 ms
  if (millis() - last_print >= 30) {


    enc1.update_speed();
    enc2.update_speed();
    enc3.update_speed();
    enc4.update_speed();

    encoder_count[0] = enc1.get_count();
    encoder_count[1] = enc2.get_count();
    encoder_count[2] = enc3.get_count();
    encoder_count[4] = enc4.get_count();

    encoder_rpm[0] = enc1.get_speed_rpm();
    encoder_rpm[1] = enc2.get_speed_rpm();
    encoder_rpm[2] = enc3.get_speed_rpm();
    encoder_rpm[3] = enc4.get_speed_rpm();

    // pid_outputs[0] = pid1.compute(wheels_targets[0] , encoder_rpm[0] , 30);
    // pid_outputs[1] = pid2.compute(wheels_targets[1] , encoder_rpm[1] , 30);
    // pid_outputs[2] = pid3.compute(wheels_targets[2] , encoder_rpm[2] , 30);
    // pid_outputs[3] = pid4.compute(wheels_targets[3] , encoder_rpm[3] , 30);

    pid_outputs[0] = constrain(pid_outputs[0],-255,255);
    pid_outputs[1] = constrain(pid_outputs[1],-255,255);
    pid_outputs[2] = constrain(pid_outputs[2],-255,255);
    pid_outputs[3] = constrain(pid_outputs[3],-255,255);

    // motor_1.Move(pid_outputs[0]);
    // motor_2.Move(pid_outputs[1]);
    // motor_3.Move(pid_outputs[2]);
    // motor_4.Move(pid_outputs[3]);


    motor_1.Move(wheels_targets[0]);
    motor_2.Move(wheels_targets[1]);
    motor_3.Move(wheels_targets[2]);
    motor_4.Move(wheels_targets[3]);



    publish_encoder_rpm();  
    publish_pid_outputs();
    publish_encoder_count();
  
    last_print = millis();
  }

  // handle ROS messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
}