#!/usr/bin/env python3

import math
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MecanumKinematicsForward(Node):

  def __init__(self):
    super().__init__('mecanum_kinematics_forward')

    # Physical dimensions of the robot (in meters)
    self.L = 0.23  # Half of the wheel-base length
    self.W = 0.15  # Half of the wheel track width
    self.R = 0.044  # Wheel radius

    # State variables (Position & Orientation)
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0  # Heading angle in radians
    self.total_distance = 0.0  # Cumulative distance traveled in meters

    # Timestamp tracking for numerical integration (dt)
    self.last_time = self.get_clock().now()

    # Subscriber to wheel speeds array [w1, w2, w3, w4] in rad/s
    self.subscription = self.create_subscription(
        Float32MultiArray, '/wheel_setpoints', self.wheel_callback, 10
    )

    # Publisher for complete Odometry data
    self.publisher = self.create_publisher(Odometry, '/odom', 10)

    # Forward Kinematics Transformation Matrix
    self.M_forward = (self.R / 4.0) * np.array([
        [1.0, 1.0, 1.0, 1.0],
        [-1.0, 1.0, -1.0, 1.0],
        [
            -1.0 / (self.L + self.W),
            1.0 / (self.L + self.W),
            1.0 / (self.L + self.W),
            -1.0 / (self.L + self.W),
        ],
    ])

    self.get_logger().info('Mecanum Odometry & Angle Estimator Started.')

  def wheel_callback(self, msg: Float32MultiArray):
    if len(msg.data) < 4:
      return

    # 1. Calculate elapsed time (dt)
    current_time = self.get_clock().now()
    dt = (current_time - self.last_time).nanoseconds / 1e9
    self.last_time = current_time

    if dt <= 0:
      return

    # 2. Transform wheel angular velocities to robot body velocities (Vx, Vy, Wz)
    w = np.array(msg.data[:4]).reshape((4, 1))
    v = self.M_forward @ w

    vx = float(v[0][0])  # Linear speed forward/backward
    vy = float(v[1][0])  # Linear speed strapping sideways
    wz = float(v[2][0])  # Angular velocity (rad/s)

    # 3. Integrate angular velocity to update heading angle (Theta)
    self.theta += wz * dt
    # Normalize theta to remain within [-pi, pi]
    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    # 4. Transform velocities from robot frame to global odom frame
    delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
    delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

    self.x += delta_x
    self.y += delta_y

    # 5. Compute cumulative distance traveled
    self.total_distance += math.sqrt(delta_x**2 + delta_y**2)

    # Log orientation and distance for monitoring
    angle_deg = math.degrees(self.theta)
    self.get_logger().info(
        f'Angle: {angle_deg:.2f}° | Dist: {self.total_distance:.2f}m | Pos:'
        f' ({self.x:.2f}, {self.y:.2f})'
    )

    # 6. Construct and publish Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time.to_msg()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'

    # Position
    odom_msg.pose.pose.position.x = self.x
    odom_msg.pose.pose.position.y = self.y

    # Convert Euler angle (theta) to Quaternion orientation for ROS 2
    odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

    # Velocities
    odom_msg.twist.twist.linear.x = vx
    odom_msg.twist.twist.linear.y = vy
    odom_msg.twist.twist.angular.z = wz

    self.publisher.publish(odom_msg)


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