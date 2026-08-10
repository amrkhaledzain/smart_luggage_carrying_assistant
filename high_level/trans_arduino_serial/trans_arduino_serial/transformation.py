#!/usr/bin/env python3

import serial  
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SimpleSerialBridge(Node):

  def __init__(self):
    super().__init__('simple_serial_bridge')

 
    self.arduino = serial.Serial('/dev/ttyACM0', 9600)


    self.create_subscription(
        Float32MultiArray, '/wheel_setpoints', self.receive_speeds, 10
    )

    self.get_logger().info('done')

  def receive_speeds(self, msg):

    fl = int((msg.data[0] / 30.0) * 255)
    fr = int((msg.data[1] / 30.0) * 255)
    rl = int((msg.data[2] / 30.0) * 255)
    rr = int((msg.data[3] / 30.0) * 255)


    text_to_send = f'{fl},{fr},{rl},{rr}\n'

    self.arduino.write(text_to_send.encode('utf-8'))


def main(args=None):
  rclpy.init(args=args)
  node = SimpleSerialBridge()

  try:
    rclpy.spin(node)  
  except KeyboardInterrupt:
    pass
  finally:
    
    node.arduino.write(b'0,0,0,0\n')
    node.arduino.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()