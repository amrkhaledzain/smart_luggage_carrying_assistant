#!/usr/bin/env python3

import math
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class TargetTrackerNode(Node):

  def __init__(self):
    super().__init__('target_tracker_node')

    # Parameters
    self.declare_parameter('stop_distance', 0.50)
    self.stop_distance = (
        self.get_parameter('stop_distance').get_parameter_value().double_value
    )

    # Publishers & Subscribers
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.create_subscription(
        Point, '/aruco/target_pose', self.pose_callback, 10
    )

    self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    # State Machine Variables
    self.current_yaw = 0.0
    self.search_state = (
        'TRACKING'  # States: TRACKING, SWEEP_RIGHT, SWEEP_LEFT, RETURN_CENTER
    )
    self.search_start_yaw = 0.0
    self.sweep_angle = math.radians(45.0)  # 45 degrees = 0.785 rad
    self.search_angular_speed = 14.0  # rad/s

    self.no_target_count=0

    self.get_logger().info('Target Tracker Node with Fixed Sweep Search Started!')

  def odom_callback(self, msg: Odometry):
    # Extract Yaw from Quaternion
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    self.current_yaw = 2.0 * math.atan2(qz, qw)

  def pose_callback(self, msg: Point):
    cmd = Twist()

    x_m = msg.x
    z_m = msg.z
    status = msg.y

    if status == 1.0 : # Target Found
      self.search_state = 'TRACKING'

      self.no_target_count=0
      # Distance control
      depth_error = z_m - self.stop_distance
      if depth_error > 0.05:
        cmd.linear.x = ( depth_error * 10)
      elif depth_error < -0.10:
        cmd.linear.x = ( depth_error * 4)
      else:
        cmd.linear.x = 0.0

      # Lateral and angular adjustment
      if abs(x_m) > 0.1:
        cmd.linear.y = -x_m * 1.2
        cmd.angular.z = -x_m * 0.8

    else:  # Target Lost -> Execute 45-degree Sweep Search
      self.no_target_count +=1
      if self.no_target_count >= 50:
        if self.search_state == 'TRACKING':
          self.search_state = 'SWEEP_RIGHT'
          self.search_start_yaw = self.current_yaw
          self.get_logger().info('Target lost! Starting right sweep...')

        # Relative angle from initial heading: Positive = Left, Negative = Right
        rel_yaw = self.normalize_angle(self.current_yaw - self.search_start_yaw)

        if self.search_state == 'SWEEP_RIGHT':
          # Turn right until angle reaches -45 degrees (-0.785 rad)
          if rel_yaw > -self.sweep_angle:
            cmd.angular.z = -self.search_angular_speed  # Negative = Clockwise (Right)
          else:
            self.search_state = 'SWEEP_LEFT'
            self.get_logger().info('Reached -45°. Sweeping left to +45°...')

        elif self.search_state == 'SWEEP_LEFT':
          # Turn left until angle reaches +45 degrees (+0.785 rad)
          if rel_yaw < self.sweep_angle:
            cmd.angular.z = self.search_angular_speed  # Positive = Counter-Clockwise (Left)
          else:
            self.search_state = 'RETURN_CENTER'
            self.get_logger().info('Reached +45°. Returning to center...')

        elif self.search_state == 'RETURN_CENTER':
          # Return to 0 relative heading
          if rel_yaw > 0.05:
            cmd.angular.z = -self.search_angular_speed
          elif rel_yaw < -0.05:
            cmd.angular.z = self.search_angular_speed
          else:
            cmd.angular.z = 0.0
            self.search_state = 'SWEEP_RIGHT'  # Restart sweep cycle if still lost

    self.cmd_vel_pub.publish(cmd)

  @staticmethod
  def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
  rclpy.init(args=args)
  node = TargetTrackerNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    stop_cmd = Twist()
    node.cmd_vel_pub.publish(stop_cmd)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()