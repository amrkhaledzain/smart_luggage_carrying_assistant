import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 1. Declare the target_id launch argument
  target_id_arg = DeclareLaunchArgument(
      'target_id',
      default_value='0',
      description='ArUco Marker ID to track and follow',
  )

  # 2 read the target_id from the launch argument
  target_id = LaunchConfiguration('target_id')

  # 3. Camera Driver Node
  camera_driver_node = Node(
      package='camera_driver_pkg',
      executable='camera_driver_node',
      name='camera_driver_node',
      output='screen',
  )

    # 4. ArUco Detector Node
  aruco_detector_node = Node(
      package='aruco_detector_pkg',
      executable='aruco_detector_node',
      name='aruco_detector_node',
      parameters=[{'target_id': target_id}],
      output='screen',
  )

  # 5. Target Tracker Node
  target_tracker_node = Node(
      package='target_tracker_pkg',
      executable='target_tracker_node',
      name='target_tracker_node',
      output='screen',
  )

  # 6. Mecanum Kinematics Node
  mecanum_inverse_node = Node(
      package='kinematics_pkg',
      executable='mecanum_inverse_node',
      name='mecanum_inverse_node',
      output='screen',
  )
  # 7. Micro-ROS Agent Node
  micro_ros_agent = ExecuteProcess(
      cmd=[
          'ros2',
          'run',
          'micro_ros_agent',
          'micro_ros_agent',
          'serial',
          '--dev',
          '/dev/ttyACM0',
      ],
      output='screen',
  )
  trasform_serial_arduino = Node(
        package='trans_arduino_serial',
        executable='trans_arduino_serial',
        name='trans_arduino_serial',
        output='screen',    
  )
  odom_node = Node(
        package='kinematics_pkg',
        executable='mecanum_forward_node',
        name='mecanum_forward_node',
        output='screen',
    )

  return LaunchDescription([
      target_id_arg,
      camera_driver_node,
      aruco_detector_node,
      target_tracker_node,
      mecanum_inverse_node,
      micro_ros_agent,
      # trasform_serial_arduino,
      odom_node,
  ])