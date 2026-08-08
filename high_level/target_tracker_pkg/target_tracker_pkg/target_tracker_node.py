#!/usr/bin/env python3

from geometry_msgs.msg import Point, Twist
import rclpy
from rclpy.node import Node


class TargetTrackerNode(Node):

  def __init__(self):
    super().__init__('target_tracker_node')

    self.declare_parameter('stop_distance', 0.50)
    self.stop_distance = (
        self.get_parameter('stop_distance').get_parameter_value().double_value
    )

    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.create_subscription(
        Point, '/aruco/target_pose', self.pose_callback, 10
    )

    self.get_logger().info('Target Tracker Node Started!')

  def pose_callback(self, msg):
    cmd = Twist()

    x_m = msg.x
    z_m = msg.z
    status = msg.y

    if status == 1.0:  # Target Found
      depth_error = z_m - self.stop_distance
      if depth_error > 0.05:
        cmd.linear.x = min(0.4, depth_error * 0.6)
      elif depth_error < -0.10:
        cmd.linear.x = max(-0.2, depth_error * 0.4)
      else:
        cmd.linear.x = 0.0

      if abs(x_m) > 0.1:
        cmd.linear.y = -x_m * 1.2
        cmd.angular.z = -x_m * 0.8
    else:  # Searching
      cmd.linear.x = 0.0
      cmd.linear.y = 0.0
      cmd.angular.z = 0.0

    self.cmd_vel_pub.publish(cmd)


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