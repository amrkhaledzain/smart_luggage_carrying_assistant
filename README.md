# Smart Luggage Carrying Assistant 🧳🤖

An autonomous vision-guided assistant designed to follow users and transport luggage in indoor environments such as airports and stations. The system utilizes **ROS 2**, **OpenCV**, and **Micro-ROS** for target detection, dynamic tracking, and motion control.

---

## 🏗️ Repository Architecture

- **`high_level/`**: ROS 2 packages responsible for image acquisition, ArUco marker detection, target tracking, and kinematics computation.
- **`low_level/`**: Embedded firmware and motor control interface using Micro-ROS on the microcontroller platform.

---

## 🚀 Quick Start (ROS 2 High-Level)

### 1. Build the Workspace
Navigate to the workspace root and build:
```bash
'colcon build'
'source install/setup.bash'

### 2. Launch the System
Run the bringup launch file:
'ros2 launch bringup_pkg robot_bringup.launch.py target_id:=0'
