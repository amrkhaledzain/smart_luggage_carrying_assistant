#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MecanumKinematics_inverse(Node):

  def __init__(self):
    super().__init__('mecanum_kinematics_inverse')
    # Subscribe to target velocity (/cmd_vel)
    self.subscription = self.create_subscription(
        Twist, '/cmd_vel', self.cmd_vel_callback, 10
    )
    # Publish wheel speeds array [v1, v2, v3, v4]
    self.publisher = self.create_publisher(
        Float32MultiArray, '/wheel_setpoints', 10
    )

    # Robot Physical Dimensions (in meters)
    self.L = 0.23  # Half length (Lx)
    self.W = 0.15  # Half width (Ly)
    self.R = 0.044  # Wheel radius (4.4 cm)

    self.get_logger().info('Mecanum Kinematics Node has been started')

  def cmd_vel_callback(self, msg: Twist):
    vx = msg.linear.x
    vy = msg.linear.y
    omega = msg.angular.z

    # Inverse Kinematics Formulae (Output in rad/s)
    v1 = (vx - vy - (self.L + self.W) * omega) / self.R
    v2 = (vx + vy + (self.L + self.W) * omega) / self.R
    v3 = (vx + vy - (self.L + self.W) * omega) / self.R
    v4 = (vx - vy + (self.L + self.W) * omega) / self.R

    speeds_msg = Float32MultiArray()
    speeds_msg.data = [v1, v2, v3, v4]
    self.publisher.publish(speeds_msg)


def main(args=None):
  rclpy.init(args=args)
  kinematics_node = MecanumKinematics_inverse()

  try:
    rclpy.spin(kinematics_node)
  except KeyboardInterrupt:
    pass
  finally:
    stop_msg = Float32MultiArray()
    stop_msg.data = [0.0, 0.0, 0.0, 0.0]
    kinematics_node.publisher.publish(stop_msg)
    kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()