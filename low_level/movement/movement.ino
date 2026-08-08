#include "pin_config.hpp"

void setup() {

  analogWriteResolution(16);  // makes anologue write take range from 0 to 65535

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Initialize subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/wheel_setpoints"));

  // Initialize subscriber
  RCCHECK(rclc_subscription_init_default(
    &pid_values_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/pid_tunning_values"));

  // Initialize pid mode subscriber
  RCCHECK(rclc_subscription_init_default(
    &pid_mode_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/movement_pid_mode"));

  // Initialize pid publisher
  RCCHECK(rclc_publisher_init_default(
    &pid_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "pid_outputs"));

  // Initialize pid publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_rpm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_speed"));

  // Initialize pid publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_count_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_counts"));

  const unsigned int timer_timeout = DT * 1000;
  rclc_timer_init_default(
    &pub_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);

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

  pid_mode_msg.data = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &setpoints_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_values_subscriber, &pid_values_msg, &subscription_pid_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_mode_subscriber, &pid_mode_msg, &subscription_pid_mode, ON_NEW_DATA));
  rclc_executor_add_timer(&executor, &pub_timer);
}
void loop() {

  // handle ROS messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}