# Smart Luggage Carrying Assistant 🧳🤖

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FJazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](#)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Embedded-A200FF?style=for-the-badge&logo=ros&logoColor=white)](#)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](#)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](#)

An enterprise-grade autonomous vision-guided mobile robot designed for real-time human-following and luggage transport in complex indoor environments. This project serves as a comprehensive implementation of full-stack robotics, bridging high-level computer vision and autonomous tracking (ROS 2) with deterministic, real-time closed-loop motor control (micro-ROS).

---

## 📑 Table of Contents
1. [System Architecture Overview](#-system-architecture-overview)
2. [Mathematical Modeling & Kinematics](#-mathematical-modeling--kinematics)
3. [Control Theory & PID Implementation](#-control-theory--pid-implementation)
4. [Software Stack (High-Level ROS 2)](#-software-stack-high-level-ros-2)
5. [Firmware & Embedded Systems (Low-Level)](#-firmware--embedded-systems-low-level)
6. [Project Directory Tree](#-project-directory-tree)
7. [Installation & Bringup](#-installation--bringup)

---

## 🏗️ System Architecture Overview

The system operates on a dual-layer computing architecture to ensure both high computational throughput for image processing and strict real-time deterministic behavior for motor control.

1. **High-Level Processing Unit (Microprocessor/SBC):** Runs **Ubuntu & ROS 2**. Responsible for interfacing with the camera, detecting ArUco markers via OpenCV, estimating 3D poses, and calculating the required velocity vectors (`Twist` messages) to keep the target centered at a fixed distance.
2. **Low-Level Control Unit (Microcontroller):** Runs **micro-ROS** over an RTOS or bare-metal loop. It acts as a node in the ROS 2 graph, subscribing to wheel velocity commands, reading quadrature encoders via hardware interrupts, and applying discrete PID control to drive the motor drivers (Cytron MDD10A / L298N) via hardware PWM.

---

## 🧮 Mathematical Modeling & Kinematics

To achieve holonomic (omnidirectional) motion, the robot utilizes a four-wheel **Mecanum Drive System**. Mecanum wheels have rollers mounted at a $45^\circ$ angle to the wheel's circumference, allowing the chassis to move in 3 Degrees of Freedom (X, Y, $\theta$) independently.

### System Parameters
* $R$: Radius of the Mecanum wheel ($m$).
* $L_x$: Distance from the robot's geometric center to the front/rear axle ($m$).
* $L_y$: Distance from the robot's geometric center to the left/right wheel center ($m$).
* $V_x, V_y, \omega_z$: Desired linear and angular velocities of the robot chassis.
* $\omega_1, \omega_2, \omega_3, \omega_4$: Angular velocities of the individual wheels (Front-Left, Front-Right, Rear-Right, Rear-Left).

### Inverse Kinematics (Chassis Velocity $\rightarrow$ Wheel Speeds)
When the tracking node requests a chassis movement, the `kinematics_pkg` converts the desired body twist into individual wheel speeds. The derived Jacobian matrix for our configuration is:

$$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix} = \frac{1}{R}  \begin{bmatrix}  1 & -1 & -(L_x + L_y) \\  1 & 1 & (L_x + L_y) \\  1 & -1 & (L_x + L_y) \\  1 & 1 & -(L_x + L_y)  \end{bmatrix} \begin{bmatrix} V_x \\ V_y \\ \omega_z \end{bmatrix}$$

*Note: In our human-following use case, $V_y$ (strafing) is typically constrained or set to $0$ to prioritize natural forward/rotational following behavior, but the model supports full holonomic drift.*

### Forward Kinematics (Odometry Estimation)
To estimate the robot's actual movement in the world frame (for future SLAM/Odometry integration), the system can calculate chassis velocities from the actual measured wheel speeds (via encoders):

$$\begin{bmatrix} V_x \\ V_y \\ \omega_z \end{bmatrix} = \frac{R}{4}  \begin{bmatrix}  1 & 1 & 1 & 1 \\  -1 & 1 & -1 & 1 \\  -\frac{1}{L_x + L_y} & \frac{1}{L_x + L_y} & \frac{1}{L_x + L_y} & -\frac{1}{L_x + L_y}  \end{bmatrix}  \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix}$$

---

## 🎛️ Control Theory & PID Implementation

The system employs a **Cascaded Control Loop** strategy:

### 1. Outer Loop (Spatial Tracking - ROS 2)
Located in `pid_follower_node.cpp`. This loop regulates the spatial distance and angle between the robot and the human target.

* **Distance Error:** $e_d = \text{Target Distance} - \text{Measured Distance}_Z$
* **Angle Error:** $e_\theta = 0 - \text{Measured Angle}_X$
* **Output:** $V_x$ (Linear Velocity) and $\omega_z$ (Angular Velocity).

### 2. Inner Loop (Motor Velocity - micro-ROS/MCU)
Located in `PID.cpp` on the microcontroller. Physical DC motors suffer from non-linearities, battery voltage drops, and friction. A discrete PID controller ensures the actual wheel RPM matches the commanded RPM from the kinematics node.

The discrete PID difference equation used with Backward Euler integration:
$$u(k) = K_p e(k) + K_i \sum_{i=0}^{k} e(i) \Delta t + K_d \frac{e(k) - e(k-1)}{\Delta t}$$

**Key Firmware Features:**
* **Anti-Windup:** Prevents the integral term from accumulating massive errors when the motor reaches its physical limit (100% PWM saturation).
* **Precise $\Delta t$:** Uses hardware timers (`Timer_class.cpp`) to measure the exact microseconds passed between loop iterations, rather than relying on blocking `delay()` functions, ensuring mathematically accurate derivative and integral responses.

---

## 🧠 Software Stack (High-Level ROS 2)

The `high_level` directory contains modular, decoupled ROS 2 packages:

1. **`vision_pkg` (Perception):**
   * Streams video frames.
   * Utilizes OpenCV `aruco` module.
   * Solves the **Perspective-n-Point (PnP)** problem using the camera's intrinsic calibration matrix to extract the 6D pose of the target marker relative to the camera frame.
2. **`tracking_pkg` (Decision Making):**
   * Subscribes to the target's pose.
   * Filters out high-frequency noise.
   * Computes spatial errors and publishes smooth `geometry_msgs/Twist` commands.
3. **`kinematics_pkg` (Actuation Mapping):**
   * Implements the Inverse Kinematics math described above.
   * Converts $m/s$ and $rad/s$ into RPM targets for each specific wheel, publishing them to the micro-ROS topics.
4. **`bringup_pkg` (Orchestration):**
   * Centralizes all `launch` files and `params.yaml` (PID gains, camera matrices, robot dimensions) for single-command system startups.

---

## ⚡ Firmware & Embedded Systems (Low-Level)

The `low_level` directory contains the robust C++ firmware designed for the microcontroller, bridging micro-ROS with bare-metal hardware peripherals.

* **micro-ROS Executor (`movement.ino`):** Initializes the ROS 2 node natively on the MCU. Handles Serial transport, topic subscriptions, and the main non-blocking control loop.
* **Hardware Timers (`Timer_class` & `timer_peripherals`):** Configures MCU hardware registers to generate precise interrupts for the PID loops and RPM calculations, independent of the main CPU loop.
* **Quadrature Encoders (`encoder`):** Utilizes external hardware interrupts (RISING/FALLING edges) on Channel A and B to determine direction and accumulate tick counts with zero pulse loss.
* **Motor Drivers (`CYTRON_MDD10A` & `L298N`):** Object-oriented abstractions. The system supports heavy-duty Cytron drivers (Sign-Magnitude PWM) for high-torque applications, and L298N (Locked-Antiphase/Dual-Pin) for development testing, all inheriting from a generic `dc_motor` interface.

---

## 📂 Project Directory Tree

```text
smart_luggage_carrying_assistant/
│
├── high_level/                               # ROS 2 Processing Layer (Ubuntu)
│   ├── vision_pkg/                           # OpenCV ArUco detection & PnP pose estimation
│   ├── tracking_pkg/                         # Spatial PID following & target locking
│   ├── kinematics_pkg/                       # Mecanum inverse kinematics matrix solver
│   ├── trans_arduino_serial/                 # Serial bridge node (Subscribes to wheel setpoints & sends to Serial)
│   └── bringup_pkg/                          # Launch files and params.yaml
│
└── low-level/                                # Embedded Firmware (MCU)
    ├── arduino_code/                         # Simple Arduino Uno/Nano implementation
    │   ├── arduino_code.ino                  # Serial receiver & direct L298N drive loop
    │   ├── L298N.cpp/.hpp                    # L298N H-Bridge driver
    │   └── dc_motor.hpp                      # Abstract Base Class for motor drivers
    │
    └── stm32_blackpill/                      # STM32 F401 Firmware implementations
        ├── PeripheralPins_BLACKPILL_F401Cx.c # STM32 pinout register definitions
        ├── no_pid_movement/                  # Open-loop control pipeline (No feedback)
        │   ├── no_pid_movement.ino
        │   ├── L298N.cpp/.hpp
        │   ├── PID.cpp/.hpp
        │   ├── Timer_class.cpp/.h
        │   ├── dc_motor.hpp
        │   ├── encoder.cpp/.h
        │   ├── pin_confg.hpp
        │   └── wheels.cpp/.hpp
        │
        └── pid_movement/                     # Closed-loop control pipeline (Encoder PID control)
            ├── pid_movement.ino
            ├── L298N.cpp/.hpp
            ├── PID.cpp/.hpp
            ├── Timer_class.cpp/.h
            ├── dc_motor.hpp
            ├── encoder.cpp/.h
            ├── pin_confg.hpp
            └── wheels.cpp/.hpp
```

---

## 🚀Installation & Bringup

### 1. ROS 2 Workspace Setup

```text
  cd ~/ieee_ws
  rosdep update && rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install
  source install/setup.bash
```

### 2. Flashing the Firmware

-Open the low_level/movement directory in your preferred IDE (Arduino IDE / PlatformIO).
-Ensure the micro-ROS client library is installed for your specific board.
-Compile and flash the code to the microcontroller.

### 3. Running the System

Terminal : Launch the High-Level Intelligence
(Starts the camera, vision pipelines, tracking algorithms, and kinematics).

```text
    ros2 launch bringup_pkg robot_bringup.launch.py target_id:=0
```

###L aunch Serial Bridge to Hardware

```text
ros2 run trans_arduino_serial trans_arduino_serial
```
