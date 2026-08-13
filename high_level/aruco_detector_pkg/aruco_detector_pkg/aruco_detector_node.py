#!/usr/bin/env python3

import cv2
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ArucoDetectorNode(Node):

  def __init__(self):
    super().__init__('aruco_detector_node')

    # 1. Publisher for Target Position
    self.pose_pub = self.create_publisher(Point, '/aruco/target_pose', 10)

    # 2. Parameter for Target ID
    self.declare_parameter('target_id', 0)

    # 3. Subscriber to raw camera image
    self.create_subscription(
        Image, '/camera/image_raw', self.image_callback, 10
    )

    # 4. ArUco Dictionary Setup (Supports up to 1000 IDs)
    self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    self.parameters = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    # 5. Constants
    self.focal_length = 600.0
    self.marker_real_width = 0.20  # 10 cm

    self.get_logger().info('ArUco Detector Node Started!')

  def image_callback(self, msg: Image):
    # Convert ROS Image msg to NumPy Array
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        (msg.height, msg.width, 3)
    )

    # Read dynamically updated target_id parameter
    target_id = (
        self.get_parameter('target_id').get_parameter_value().integer_value
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = self.detector.detectMarkers(gray)

    pose = Point()

    if ids is not None and target_id in ids:
      idx = np.where(ids == target_id)[0][0]
      pts = corners[idx][0]

      # Distance and Offset calculation
      pixel_width = np.linalg.norm(pts[0] - pts[1])
      z = (self.marker_real_width * self.focal_length) / pixel_width
      center_x = np.mean(pts[:, 0])
      x = (center_x - (msg.width / 2.0)) / self.focal_length * z

      pose.x = float(x)
      pose.z = float(z)
      pose.y = 1.0  # Found

      # Draw Bounding Box and Info on Frame
      pts_int = pts.astype(int)
      cv2.polylines(frame, [pts_int], True, (0, 255, 0), 3)
      cv2.putText(
          frame,
          f'TARGET ID: {target_id} | Z: {z:.2f}m | X: {x:.2f}m',
          (20, 40),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.6,
          (0, 255, 0),
          2,
      )
    else:
      pose.x = 0.0
      pose.z = 0.0
      pose.y = 0.0  # Not Found

      cv2.putText(
          frame,
          f'SEARCHING FOR TARGET ID: {target_id}...',
          (20, 40),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.6,
          (0, 0, 255),
          2,
      )

    self.pose_pub.publish(pose)

    # Show Live Frame Window
    cv2.imshow('Robot Camera View', frame)
    cv2.waitKey(1)


def main(args=None):
  rclpy.init(args=args)
  node = ArucoDetectorNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()