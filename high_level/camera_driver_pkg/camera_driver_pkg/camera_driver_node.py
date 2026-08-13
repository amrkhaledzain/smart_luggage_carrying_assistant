#!/usr/bin/env python3

import cv2
from geometry_msgs.msg import Twist  # noqa: F401
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraDriverNode(Node):

  def __init__(self):
    super().__init__('camera_driver_node')
    self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
    self.cap = cv2.VideoCapture(1)
    self.timer = self.create_timer(0.033, self.timer_callback)
    self.get_logger().info('Camera Driver Node Started!')

  def timer_callback(self):
    ret, frame = self.cap.read()
    if not ret:
      return

    # Create ROS 2 Image msg directly without cv_bridge
    msg = Image()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_frame'
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()

    self.publisher_.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  node = CameraDriverNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()