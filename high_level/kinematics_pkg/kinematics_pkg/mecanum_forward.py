#!/usr/bin/env python3

import math
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MecanumKinematicsForward(Node):

  def __init__(self):
    super().__init__('mecanum_kinematics_forward')

    # Physical dimensions of the robot (in meters)
    self.L = 0.23  # Half of wheel-base length
    self.W = 0.15  # Half of wheel track width
    self.R = 0.044  # Wheel radius

    # State variables
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0
    self.total_distance = 0.0

    self.last_time = self.get_clock().now()

    self.subscription = self.create_subscription(
        Float32MultiArray, '/encoder_speed', self.wheel_callback, 10
    )
    self.publisher = self.create_publisher(Odometry, '/odom', 10)

    # Standard Mecanum Forward Kinematics Matrix
    # Robot frame: X = Forward, Y = Left, Z = Up (ROS REP 103)
    # Wheel ordering: [FL, FR, BL, BR]
    lx_ly = self.L + self.W
    self.M_forward = (self.R / 4.0) * np.array([
        [1.0, 1.0, 1.0, 1.0],  # Vx
        [-1.0, 1.0, 1.0, -1.0],  # Vy
        [-1.0 / lx_ly, 1.0 / lx_ly, -1.0 / lx_ly, 1.0 / lx_ly],  # Wz
    ])

    self.get_logger().info('Mecanum Odometry Node Initialized.')

  def wheel_callback(self, msg: Float32MultiArray):
    if len(msg.data) < 4:
      return

    current_time = self.get_clock().now()
    dt = (current_time - self.last_time).nanoseconds / 1e9
    self.last_time = current_time

    if dt <= 0.0 or dt > 1.0:  # Ignore zero or invalid large jumps
      return

    # 1. Calculate body velocities (vx, vy, wz) in base_link frame
    w = np.array(msg.data[:4]).reshape((4, 1))
    v = self.M_forward @ w

    vx = float(v[0][0])
    vy = float(v[1][0])
    wz = float(v[2][0])

    # 2. Midpoint integration for accurate pose update
    delta_theta = wz * dt
    mid_theta = self.theta + (delta_theta / 2.0)

    # Transform velocities from body frame to odom frame using mid_theta
    delta_x = (vx * math.cos(mid_theta) - vy * math.sin(mid_theta)) * dt
    delta_y = (vx * math.sin(mid_theta) + vy * math.cos(mid_theta)) * dt

    self.x += delta_x
    self.y += delta_y
    self.theta = self.normalize_angle(self.theta + delta_theta)

    self.total_distance += math.hypot(delta_x, delta_y)

    # 3. Publish Odometry
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time.to_msg()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'

    odom_msg.pose.pose.position.x = self.x
    odom_msg.pose.pose.position.y = self.y
    odom_msg.pose.pose.position.z = 0.0

    # Quaternion representation for ROS 2
    odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

    # Velocities MUST remain in base_link frame
    odom_msg.twist.twist.linear.x = vx
    odom_msg.twist.twist.linear.y = vy
    odom_msg.twist.twist.angular.z = wz

    self.publisher.publish(odom_msg)

  @staticmethod
  def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
  rclpy.init(args=args)
  node = MecanumKinematicsForward()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()