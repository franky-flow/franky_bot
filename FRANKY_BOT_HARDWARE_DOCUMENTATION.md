# Franky_Bot - ROS2 Jazzy Differential Drive Robot
## Real Hardware Implementation Documentation

## Project Overview

**Franky_Bot** is a differential drive mobile robot project built on **ROS2 Jazzy**, now successfully deployed on **real hardware** with Raspberry Pi 5, Arduino Mega, and physical sensors. The robot evolved from a Gazebo Harmonic simulation into a fully functional physical robot with encoder feedback, real-time motor control, LiDAR navigation, and camera vision.

### Key Features
- **Real Hardware Platform**: Raspberry Pi 5 running ROS2 Jazzy
- **Motor Control**: Arduino Mega with L298N motor drivers controlling differential drive
- **Encoder Feedback**: Hall effect encoders with 6000 counts per revolution
- **Sensor Suite**: 
  - 2D LiDAR (360° scanning, 12m range)
  - USB/Pi Camera (640x480 resolution, RGB)
- **Teleoperation**: Gamepad control with turbo mode via joy_linux
- **ROS2 Control**: Custom hardware interface `diffdrive_arduino` for real-time motor control
- **Dual Operation Modes**: Simulation (Gazebo Harmonic) and Real Hardware

### Platform Specifications
- **ROS Distribution**: ROS2 Jazzy
- **Single Board Computer**: Raspberry Pi 5 (running Ubuntu/ROS2)
- **Microcontroller**: Arduino Mega 2560
- **Motor Driver**: L298N Dual H-Bridge
- **Encoders**: Hall effect encoders (6000 CPR)
- **Communication**: USB Serial (57600 baud)
- **Simulator**: Gazebo Harmonic (for development/testing)
- **Build System**: CMake (ament_cmake)
- **Package Format**: Format 3

---

## Hardware Architecture

### System Overview
```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5 (ROS2 Jazzy)              │
│                                                             │
│  ┌─────────────────┐    ┌──────────────┐   ┌────────────┐  │
│  │ diff_drive_     │    │  Camera      │   │  LiDAR     │  │
│  │ controller      │    │  Node        │   │  Node      │  │
│  └────────┬────────┘    └──────┬───────┘   └─────┬──────┘  │
│           │                    │                  │         │
│  ┌────────▼────────┐    ┌──────▼───────┐   ┌─────▼──────┐  │
│  │ diffdrive_      │    │ v4l2_camera  │   │ rplidar_   │  │
│  │ arduino         │    │ (USB/CSI)    │   │ ros        │  │
│  │ (HW Interface)  │    └──────────────┘   └────────────┘  │
│  └────────┬────────┘                                        │
│           │ Serial                                          │
└───────────┼─────────────────────────────────────────────────┘
            │ USB (/dev/ttyACM0, 57600 baud)
            │
┌───────────▼─────────────────────────────────────────────────┐
│                    Arduino Mega 2560                        │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Serial Communication Protocol                       │   │
│  │  - Command: "m L R" (motor velocities -255 to 255)  │   │
│  │  - Command: "e" (request encoder values)            │   │
│  │  - Response: "L_ticks R_ticks"                       │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌────────────┐         ┌────────────┐                     │
│  │  Encoder   │         │  Encoder   │                     │
│  │  Interrupt │         │  Interrupt │                     │
│  │  Handler   │         │  Handler   │                     │
│  └─────┬──────┘         └──────┬─────┘                     │
│        │                       │                           │
└────────┼───────────────────────┼───────────────────────────┘
         │                       │
    ┌────▼────┐             ┌────▼────┐
    │ L298N   │             │ L298N   │
    │ Motor   │             │ Motor   │
    │ Driver  │             │ Driver  │
    └────┬────┘             └────┬────┘
         │                       │
    ┌────▼────┐             ┌────▼────┐
    │  Left   │             │ Right   │
    │  Motor  │             │ Motor   │
    │  +Enc   │             │ +Enc    │
    └─────────┘             └─────────┘
```

### Hardware Components

#### 1. Raspberry Pi 5
- **Role**: Main computing unit running ROS2 Jazzy
- **Responsibilities**:
  - ROS2 node management
  - Differential drive controller
  - Sensor data processing
  - Navigation algorithms
  - Joystick teleoperation
  - Hardware interface to Arduino

#### 2. Arduino Mega 2560
- **Role**: Real-time motor control and encoder reading
- **Pin Configuration**:
  ```
  Motor Control (L298N):
  - ENA (Pin 5):  Left motor PWM
  - IN1 (Pin 9):  Left motor direction 1
  - IN2 (Pin 10): Left motor direction 2
  - ENB (Pin 6):  Right motor PWM
  - IN3 (Pin 11): Right motor direction 1
  - IN4 (Pin 12): Right motor direction 2
  
  Encoders (Hall Effect):
  - Pin 2:  Left encoder A (interrupt)
  - Pin 3:  Left encoder B
  - Pin 18: Right encoder A (interrupt)
  - Pin 19: Right encoder B
  ```

#### 3. L298N Motor Driver
- **Specifications**:
  - Dual H-bridge for independent wheel control
  - PWM speed control (0-255)
  - Bidirectional control
  - Current capacity: Up to 2A per channel

#### 4. Encoders
- **Type**: Hall effect encoders
- **Resolution**: 6000 counts per revolution
- **Connection**: Interrupt-based reading on Arduino
- **Purpose**: Closed-loop velocity control and odometry

#### 5. Sensors
- **LiDAR**: 2D scanning laser (e.g., RPLIDAR A1/A2)
  - Range: 0.3m to 12m
  - Field of view: 360°
  - Update rate: 10 Hz
  
- **Camera**: USB or Pi Camera Module
  - Resolution: 640x480
  - Frame rate: 15-30 fps
  - Interface: v4l2_camera node

---

## Package Structure

```
franky_ws/src/
│
├── franky_bot/                      # Main robot package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── LICENSE.md
│   ├── README.md
│   │
│   ├── config/                      # Configuration files
│   │   ├── joystick.yaml           # Gamepad configuration
│   │   ├── my_controllers.yaml     # ros2_control controllers
│   │   └── view_bot.rviz           # RViz configuration
│   │
│   ├── description/                 # Robot URDF/Xacro models
│   │   ├── robot.urdf.xacro        # Main robot description
│   │   ├── robot_core.xacro        # Chassis and wheels
│   │   ├── ros2_control.xacro      # Hardware interface config
│   │   ├── lidar.xacro             # LiDAR sensor
│   │   ├── camera.xacro            # Camera sensor
│   │   ├── inertial_macros.xacro   # Inertial properties
│   │   └── gazebo_control.xacro    # Legacy (unused)
│   │
│   ├── launch/                      # Launch files
│   │   ├── launch_robot.launch.py  # **Real hardware launcher**
│   │   ├── launch_sim.launch.py    # Simulation launcher
│   │   ├── rsp.launch.py           # Robot State Publisher
│   │   ├── joystick.launch.py      # Gamepad node
│   │   ├── joystick_rpi.launch.py  # Gamepad for RPi (joy_linux)
│   │   ├── pi_camera.launch.py     # Camera launcher (basic)
│   │   └── pi_grok_camera.launch..py # Camera launcher (advanced)
│   │
│   ├── worlds/                      # Gazebo worlds (simulation)
│   │   ├── empty.world
│   │   └── empty-with-shapes1.world
│   │
│   └── arduino/                     # Arduino firmware
│       ├── arduino_ros_control_basic/
│       │   └── arduino_ros_control_basic.ino     # Basic motor control
│       ├── arduino_ros_control_encoders/
│       │   └── arduino_ros_control_encoders.ino  # **With encoder feedback**
│       ├── arduino_ros_control_gemini/
│       │   └── arduino_ros_control_gemini.ino    # Advanced version
│       └── test_motores/
│           └── test_motores.ino                  # Motor testing
│
├── diffdrive_arduino/               # Custom ros2_control hardware interface
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── LICENSE
│   │
│   ├── include/diffdrive_arduino/
│   │   ├── diffdrive_arduino.h     # Main hardware interface
│   │   ├── arduino_comms.h         # Serial communication
│   │   ├── wheel.h                 # Wheel state management
│   │   ├── config.h                # Configuration struct
│   │   └── fake_robot.h            # Simulation fallback
│   │
│   ├── src/
│   │   ├── diffdrive_arduino.cpp   # HW interface implementation
│   │   ├── arduino_comms.cpp       # Serial protocol
│   │   ├── wheel.cpp               # Wheel calculations
│   │   ├── diffdrive_robot.cpp     # Main plugin export
│   │   └── fake_robot.cpp          # Simulation version
│   │
│   ├── launch/
│   │   ├── test_robot.launch.py    # Hardware testing
│   │   └── fake_robot.launch.py    # Simulation testing
│   │
│   └── controllers/
│       └── robot_controller_example.yaml
│
└── serial/                          # Serial communication library
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/serial/
    │   └── serial.h
    └── src/
        ├── serial.cc
        └── impl/
            ├── unix.cc              # Linux/Unix implementation
            └── list_ports/
                └── list_ports_linux.cc
```

---

## Detailed Component Analysis

### 1. Arduino Firmware

#### **arduino_ros_control_encoders.ino** (Recommended for Real Hardware)

**Purpose**: Real-time motor control with encoder feedback for ROS2 integration

**Serial Protocol** (57600 baud):
```
ROS2 → Arduino Commands:
- "e\r"         → Request encoder values
- "m L R\r"     → Set motor velocities (L, R: -255 to 255)
- "u P D I O\r" → Set PID values (currently unused)

Arduino → ROS2 Responses:
- "OK\n"              → Command acknowledged
- "L_ticks R_ticks\n" → Encoder values (response to "e")
```

**Key Features**:
- Interrupt-based encoder counting (accurate, non-blocking)
- Bidirectional motor control with PWM
- Request-response protocol matching diffdrive_arduino expectations
- Encoder direction based on B channel reading
- Safety: Motor constraining to ±255 PWM

**Encoder ISR (Interrupt Service Routine)**:
```cpp
void leftISR() {
  if (digitalRead(LEFT_ENCODER_B) == LOW) left_ticks--;
  else left_ticks++;
}

void rightISR() {
  if (digitalRead(RIGHT_ENCODER_B) == LOW) right_ticks--;
  else right_ticks++;
}
```

**Motor Control**:
- PWM range: 0-255
- Direction control via H-bridge (IN1-IN4)
- Independent left/right wheel control

---

### 2. ROS2 Hardware Interface: diffdrive_arduino

#### **Purpose**
Custom `ros2_control` hardware interface that bridges ROS2's `diff_drive_controller` with the Arduino motor controller via serial communication.

#### **Architecture**
```
diff_drive_controller (ROS2)
         ↓
    [Command Interface: velocity]
         ↓
DiffDriveArduino (hardware_interface)
         ↓
    ArduinoComms (Serial)
         ↓
    USB Serial (/dev/ttyACM0)
         ↓
    Arduino Mega
         ↓
    Motors + Encoders
         ↑
    [State Interface: position, velocity]
```

#### **Key Files**

**diffdrive_arduino.h/cpp**:
- Implements `hardware_interface::SystemInterface`
- **configure()**: Sets up wheel parameters, encoder counts, serial device
- **export_state_interfaces()**: Provides position and velocity state for each wheel
- **export_command_interfaces()**: Accepts velocity commands for each wheel
- **read()**: Reads encoder values from Arduino, updates wheel positions/velocities
- **write()**: Sends velocity commands to Arduino as PWM values

**arduino_comms.h/cpp**:
- Serial communication wrapper using `serial` library
- **readEncoderValues()**: Sends "e" command, parses response
- **setMotorValues()**: Sends "m L R" command with PWM values
- **setPidValues()**: Sends PID tuning (optional)

**wheel.h/cpp**:
- Wheel state management
- Encoder tick to radian conversion
- Velocity calculation from encoder deltas
- Radian/sec to PWM conversion

#### **Configuration Parameters** (ros2_control.xacro)
```xml
<param name="left_wheel_name">left_wheel_joint</param>
<param name="right_wheel_name">right_wheel_joint</param>
<param name="loop_rate">30</param>
<param name="device">/dev/ttyACM0</param>
<param name="baud_rate">57600</param>
<param name="timeout_ms">1000</param>
<param name="enc_counts_per_rev">6000</param>
```

---

### 3. Launch Files

#### **launch_robot.launch.py** - Real Hardware Launcher

**Purpose**: Launch the complete real robot system

**Components Launched**:
1. **Robot State Publisher (rsp)**:
   - Processes URDF/xacro
   - `use_sim_time: false`
   - Publishes TF tree and robot description

2. **Controller Manager**:
   - Node: `ros2_control_node`
   - Loads robot description and controller config
   - Uses `diffdrive_arduino` hardware interface
   - 3-second delay to ensure proper initialization

3. **Differential Drive Controller Spawner**:
   - Spawns `diff_cont` controller
   - Provides `/diff_cont/cmd_vel` interface
   - Publishes odometry to `/odom`

4. **Joint State Broadcaster Spawner**:
   - Spawns `joint_broad` broadcaster
   - Publishes joint states to `/joint_states`

5. **Joystick Node** (`joy_linux`):
   - Package: `joy_linux` (Linux-specific joystick driver)
   - Loads config from `joystick.yaml`
   - Publishes to `/joy` topic

6. **Teleoperation Node**:
   - Package: `teleop_twist_joy`
   - Converts `/joy` to `/diff_cont/cmd_vel`
   - Enable button: Right Bumper (RB)
   - Turbo button: Left Bumper (LB)

**Usage**:
```bash
ros2 launch franky_bot launch_robot.launch.py
```

#### **joystick_rpi.launch.py** - Standalone Joystick

**Purpose**: Launch only the joystick node (useful for debugging or modular launch)

**Features**:
- Uses `joy_linux` package (optimized for Linux/Raspberry Pi)
- Standalone operation

**Usage**:
```bash
ros2 launch franky_bot joystick_rpi.launch.py
```

#### **pi_camera.launch.py** - Basic Camera Launcher

**Purpose**: Launch USB or Pi Camera using `camera_ros`

**Configuration**:
- Frame ID: `camera_link_optical`
- Resolution: 640x480

**Usage**:
```bash
ros2 launch franky_bot pi_camera.launch.py
```

#### **pi_grok_camera.launch..py** - Advanced Camera Launcher

**Purpose**: Comprehensive camera launch using `v4l2_camera` with full configurability

**Parameters**:
- `video_device`: `/dev/video0` (default)
- `camera_frame_id`: `camera_link_optical`
- Resolution: 640x480 (configurable)
- Frame rate: 15 fps (configurable)
- Pixel format: YUYV or MJPG
- Output encoding: rgb8

**Topic Remapping**:
```
/image_raw    → /camera/image_raw
/camera_info  → /camera/camera_info
```

**Usage**:
```bash
ros2 launch franky_bot pi_grok_camera.launch..py

# With custom resolution
ros2 launch franky_bot pi_grok_camera.launch..py \
    image_width:=1280 image_height:=720 framerate:=30.0
```

---

### 4. Configuration Files

#### **my_controllers.yaml** - Controller Configuration

**Controller Manager Settings**:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50              # Hz
    use_sim_time: false          # Real hardware mode
```

**Differential Drive Controller (diff_cont)**:
```yaml
diff_cont:
  ros__parameters:
    base_frame_id: base_link
    use_stamped_vel: true        # Timestamped velocity commands
    cmd_vel_timeout: 0.5         # Safety timeout (seconds)
    publish_rate: 50.0           # Odometry publish rate
    open_loop: false             # Closed-loop with encoder feedback
    
    left_wheel_names: ['left_wheel_joint']
    right_wheel_names: ['right_wheel_joint']
    
    # CRITICAL: Must match physical robot dimensions
    wheel_separation: 0.35       # 350mm between wheel centers
    wheel_radius: 0.05          # 50mm wheel radius
```

**Joint State Broadcaster**:
- Publishes `/joint_states` for TF tree updates

#### **joystick.yaml** - Gamepad Teleoperation

**Joy Node Parameters**:
```yaml
joy_node:
  ros__parameters:
    device_id: 0                # First connected gamepad
    deadzone: 0.05              # 5% stick deadzone
    autorepeat_rate: 20.0       # 20 Hz publishing
```

**Teleop Node Parameters**:
```yaml
teleop_node:
  ros__parameters:
    # Linear motion (left stick vertical - axis 1)
    axis_linear:
      x: 1
    scale_linear:
      x: 0.5                    # Normal: 0.5 m/s
    scale_linear_turbo:
      x: 1.0                    # Turbo: 1.0 m/s
    
    # Angular motion (left stick horizontal - axis 0)
    axis_angular:
      yaw: 0
    scale_angular:
      yaw: 0.5                  # Normal: 0.5 rad/s
    scale_angular_turbo:
      yaw: 1.0                  # Turbo: 1.0 rad/s
    
    # Button mapping
    enable_button: 5            # Right Bumper (RB)
    enable_turbo_button: 4      # Left Bumper (LB)
```

**Safety Feature**: Enable button (RB) must be held for robot movement.

---

### 5. Robot Description (URDF/Xacro)

#### **ros2_control.xacro** - Hardware Interface Configuration

**Real Hardware Mode** (Currently Active):
```xml
<ros2_control name="RealFrankyRobot" type="system">
    <hardware>
        <plugin>diffdrive_arduino/DiffDriveArduinoHardware</plugin>
        
        <!-- Wheel Configuration -->
        <param name="left_wheel_name">left_wheel_joint</param>
        <param name="right_wheel_name">right_wheel_joint</param>
        
        <!-- Control Loop -->
        <param name="loop_rate">30</param>
        
        <!-- Serial Communication -->
        <param name="device">/dev/ttyACM0</param>
        <param name="baud_rate">57600</param>
        <param name="timeout_ms">1000</param>
        
        <!-- Encoder Configuration -->
        <param name="enc_counts_per_rev">6000</param>
    </hardware>
    
    <!-- Joint Interfaces -->
    <joint name="left_wheel_joint">
        <command_interface name="velocity">
            <parameter name="min" value="-10.0"/>
            <parameter name="max" value="10.0"/>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    
    <joint name="right_wheel_joint">
        <command_interface name="velocity">
            <parameter name="min" value="-10.0"/>
            <parameter name="max" value="10.0"/>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
</ros2_control>
```

**Simulation Mode** (Commented Out):
- Uses `gz_ros2_control/GazeboSimSystem` plugin
- For Gazebo Harmonic simulation testing

---

## ROS2 Topics and Interfaces

### Published Topics (from Robot)
| Topic | Type | Publisher | Rate | Description |
|-------|------|-----------|------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster | 50 Hz | Joint positions and velocities |
| `/odom` | `nav_msgs/msg/Odometry` | diff_drive_controller | 50 Hz | Robot odometry (encoder-based) |
| `/tf` | `tf2_msgs/msg/TFMessage` | diff_drive_controller | 50 Hz | Transform: odom→base_link |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | Static | Static transforms |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | Latched | URDF robot model |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver | 10 Hz | 360° laser scan |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | v4l2_camera | 15 Hz | RGB camera feed |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | v4l2_camera | 15 Hz | Camera calibration |
| `/joy` | `sensor_msgs/msg/Joy` | joy_linux | 20 Hz | Raw gamepad inputs |

### Subscribed Topics (to Robot)
| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/diff_cont/cmd_vel` | `geometry_msgs/msg/Twist` | diff_drive_controller | Velocity commands (linear, angular) |

### Services
| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/controller_manager/*` | Various | controller_manager | Controller management services |

---

## TF Frame Tree

```
map (if using SLAM/Nav2)
 └── odom (published by diff_drive_controller from encoders)
      └── base_link
           ├── chassis_link
           │    ├── laser_frame_link (LiDAR sensor)
           │    └── camera_link
           │         └── camera_link_optical (ROS camera convention)
           ├── left_wheel_link
           ├── right_wheel_link
           └── caster_wheel_link
```

**Key Transforms**:
- **map → odom**: Published by SLAM/localization (e.g., slam_toolbox)
- **odom → base_link**: Published by diff_drive_controller (encoder odometry)
- **base_link → chassis_link**: Static offset (-0.1m in x)
- **chassis_link → sensors**: Static mounting positions

---

## System Dependencies

### Hardware Requirements
- Raspberry Pi 4/5 (4GB+ RAM recommended)
- Arduino Mega 2560 or compatible
- L298N or similar dual H-bridge motor driver
- 2x DC motors with encoders (6000 CPR Hall effect)
- 2D LiDAR (RPLIDAR A1/A2 or compatible)
- USB camera or Raspberry Pi Camera Module
- Gamepad (Xbox-style controller)
- Power supply (12V for motors, 5V for Pi/Arduino)
- Appropriate wiring and chassis

### Software Dependencies

**ROS2 Packages**:
```bash
# Core ROS2
ros-jazzy-ros2-control
ros-jazzy-ros2-controllers
ros-jazzy-controller-manager
ros-jazzy-diff-drive-controller
ros-jazzy-joint-state-broadcaster

# Robot description
ros-jazzy-robot-state-publisher
ros-jazzy-xacro

# Teleoperation
ros-jazzy-teleop-twist-joy
ros-jazzy-joy-linux

# Sensors
ros-jazzy-v4l2-camera
ros-jazzy-rplidar-ros

# Simulation (optional, for testing)
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge
ros-jazzy-gz-ros2-control

# Build tools
ros-jazzy-ament-cmake
```

**Custom Packages** (included in workspace):
- `diffdrive_arduino` - Hardware interface
- `serial` - Serial communication library
- `franky_bot` - Main robot package

**System Libraries**:
```bash
sudo apt install -y \
    python3-pip \
    python3-serial \
    libserial-dev \
    joystick \
    jstest-gtk
```

---

## Usage Instructions

### Initial Setup

#### 1. Hardware Assembly
1. Mount motors with encoders on chassis
2. Connect L298N motor driver to Arduino:
   - Motor outputs to motors
   - Enable/Input pins to Arduino (5, 6, 9-12)
   - Power supply to L298N
3. Connect encoders to Arduino (pins 2, 3, 18, 19)
4. Connect Arduino to Raspberry Pi via USB
5. Mount LiDAR and camera
6. Connect gamepad to Raspberry Pi

#### 2. Arduino Firmware Upload
```bash
# Open Arduino IDE
# Load: arduino/arduino_ros_control_encoders/arduino_ros_control_encoders.ino
# Select: Board → Arduino Mega 2560
# Select: Port → /dev/ttyACM0
# Upload firmware

# Test serial communication
screen /dev/ttyACM0 57600
# Send: e
# Expected response: 0 0 (encoder values)
```

#### 3. Build ROS2 Workspace
```bash
cd /opt/franky_ws
colcon build --symlink-install
source install/setup.bash
```

#### 4. Serial Port Permissions
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Logout and login again, or:
newgrp dialout

# Verify Arduino connection
ls -l /dev/ttyACM*
# Should show: crw-rw---- 1 root dialout ...
```

### Running the Real Robot

#### Complete System Launch
```bash
# Source workspace
source /opt/franky_ws/install/setup.bash

# Launch robot with all components
ros2 launch franky_bot launch_robot.launch.py
```

This launches:
- Robot State Publisher
- Controller Manager with diffdrive_arduino
- Differential Drive Controller
- Joint State Broadcaster
- Joystick node (joy_linux)
- Teleoperation node

#### Launch Camera (separate terminal)
```bash
# Option 1: Advanced camera with v4l2
ros2 launch franky_bot pi_grok_camera.launch..py

# Option 2: Basic camera
ros2 launch franky_bot pi_camera.launch.py

# List available cameras
v4l2-ctl --list-devices
```

#### Launch LiDAR (if using RPLIDAR)
```bash
# Start RPLIDAR node
ros2 launch rplidar_ros rplidar.launch.py

# Or create custom launch file for your LiDAR model
```

### Teleoperation

**Gamepad Controls**:
1. **Connect gamepad** (wired or Bluetooth)
2. **Verify detection**:
   ```bash
   ls /dev/input/js*
   # Should show: /dev/input/js0
   
   # Test joystick
   jstest /dev/input/js0
   ```
3. **Control the robot**:
   - Hold **Right Bumper (RB)** to enable movement
   - **Left Stick**:
     - Vertical axis: Forward/backward
     - Horizontal axis: Left/right rotation
   - Hold **Left Bumper (LB)** for turbo mode (2x speed)

**Speed Reference**:
- Normal mode: 0.5 m/s linear, 0.5 rad/s angular
- Turbo mode: 1.0 m/s linear, 1.0 rad/s angular

### Monitoring and Debugging

#### Check System Status
```bash
# List active nodes
ros2 node list

# List controllers
ros2 control list_controllers

# Expected output:
# diff_cont[diff_drive_controller/DiffDriveController] active
# joint_broad[joint_state_broadcaster/JointStateBroadcaster] active

# Check hardware interface
ros2 control list_hardware_interfaces
```

#### Monitor Topics
```bash
# View odometry
ros2 topic echo /odom

# View joint states
ros2 topic echo /joint_states

# View velocity commands
ros2 topic echo /diff_cont/cmd_vel

# View joystick input
ros2 topic echo /joy

# View LiDAR data
ros2 topic echo /scan

# View camera (requires image processing)
ros2 run rqt_image_view rqt_image_view
```

#### Check Communication with Arduino
```bash
# Monitor serial communication (advanced)
sudo apt install minicom
minicom -D /dev/ttyACM0 -b 57600

# Send test command: e
# Should receive encoder values: 0 0
```

#### Visualize in RViz
```bash
# Launch RViz with config
rviz2 -d /opt/franky_ws/src/config/view_bot.rviz

# Or create custom config:
rviz2

# Add displays:
# - RobotModel (topic: /robot_description)
# - TF
# - LaserScan (topic: /scan)
# - Image (topic: /camera/image_raw)
# - Odometry (topic: /odom)
```

### Running Simulation (for Testing)
```bash
# Launch Gazebo simulation instead of real hardware
ros2 launch franky_bot launch_sim.launch.py

# Useful for:
# - Testing navigation algorithms
# - Verifying URDF changes
# - Developing without hardware access
```

---

## Technical Deep Dive

### Motor Control Loop

**Control Flow**:
```
1. User Input (Joystick)
    ↓
2. teleop_twist_joy → /diff_cont/cmd_vel
    ↓
3. diff_drive_controller receives Twist message
    ↓
4. Converts to left/right wheel velocities (rad/s)
    ↓
5. Sends to hardware interface (DiffDriveArduino::write())
    ↓
6. Wheel velocity → PWM value conversion
    ↓
7. arduino_comms sends "m L R" command
    ↓
8. Arduino sets motor PWM and direction
    ↓
9. Motors rotate, encoders increment
    ↓
10. DiffDriveArduino::read() polls Arduino with "e"
    ↓
11. Arduino responds with encoder ticks
    ↓
12. Tick delta → velocity calculation
    ↓
13. Position integration (odometry)
    ↓
14. Published to /odom and /tf
```

**Update Frequency**:
- Controller manager loop: 50 Hz
- Arduino polling: 30 Hz (configurable)
- Encoder reading: Continuous (interrupt-based)
- Odometry publishing: 50 Hz

### Velocity to PWM Conversion

**Algorithm** (in wheel.cpp):
```cpp
// Convert rad/s command to PWM (-255 to 255)
double rps = vel_cmd / (2.0 * M_PI);  // revolutions per second
double rpm = rps * 60.0;              // revolutions per minute
int pwm = (int)(rpm / max_rpm * 255); // scale to PWM range
```

**Example**:
- Command: 1.0 rad/s
- RPS: 1.0 / (2π) = 0.159 rps
- RPM: 0.159 * 60 = 9.55 rpm
- PWM (assuming max 100 RPM): (9.55 / 100) * 255 ≈ 24

### Encoder Odometry Calculation

**Position Update**:
```cpp
// Read encoder delta
long delta_ticks = current_ticks - previous_ticks;

// Convert to radians
double delta_angle = (2.0 * M_PI * delta_ticks) / enc_counts_per_rev;

// Update position
position += delta_angle;
```

**Velocity Calculation**:
```cpp
// Time delta
double dt = (current_time - previous_time).seconds();

// Velocity (rad/s)
velocity = delta_angle / dt;
```

**Differential Drive Odometry**:
```cpp
// Left and right wheel displacements
double dl = position_left * wheel_radius;
double dr = position_right * wheel_radius;

// Robot displacement and rotation
double dc = (dr + dl) / 2.0;  // center displacement
double dtheta = (dr - dl) / wheel_separation;

// Update pose
x += dc * cos(theta);
y += dc * sin(theta);
theta += dtheta;
```

### Serial Communication Protocol

**Frame Format**:
```
ROS2 → Arduino:
[Command][Space][Arg1][Space][Arg2]\r

Arduino → ROS2:
[Response][Newline]
```

**Examples**:
```
# Read encoders
Send: "e\r"
Receive: "1234 5678\n"

# Set motors
Send: "m 100 -50\r"
Receive: "OK\n"

# Set PID (optional)
Send: "u 30 20 0 100\r"
Receive: "OK\n"
```

**Timeout Handling**:
- Serial timeout: 1000ms (configurable)
- If no response: Hardware interface reports error
- Controller may enter safe mode (stop motors)

### PID Control

**Current Implementation**:
- Arduino firmware includes PID placeholder
- Currently using open-loop PWM control
- PID values sent from ROS2: `setPidValues(30, 20, 0, 100)`

**Future Enhancement**:
- Implement closed-loop PID on Arduino
- Use encoder feedback for accurate velocity control
- Tune Kp, Ki, Kd for optimal performance

---

## Calibration and Tuning

### Wheel Parameters

**Critical Parameters in `my_controllers.yaml`**:
```yaml
wheel_separation: 0.35  # Distance between wheel centers (meters)
wheel_radius: 0.05     # Wheel radius (meters)
```

**How to Measure**:
1. **Wheel Separation**: Measure center-to-center distance between wheels
2. **Wheel Radius**: Measure diameter, divide by 2 (or use r = circumference / 2π)

**Why Accuracy Matters**:
- Affects odometry accuracy
- Incorrect values → robot drifts or circles
- Used for velocity to wheel speed conversion

### Encoder Calibration

**Verify Encoder Counts Per Revolution**:
```bash
# Manually rotate wheel 10 full revolutions
# Monitor encoder topic
ros2 topic echo /joint_states

# Expected delta: 10 * 6000 = 60,000 ticks
# If different, update enc_counts_per_rev in ros2_control.xacro
```

**Direction Calibration**:
- Forward command should increase encoder counts
- If inverted, swap encoder A/B wires or modify ISR logic

### Motor Direction

**Test and Correct**:
```bash
# Send positive velocity
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Both wheels should rotate forward
# If one is backward, swap IN1/IN2 wires for that motor
```

### Odometry Validation

**Straight Line Test**:
1. Mark robot starting position
2. Drive 1 meter forward
3. Check `/odom` reported distance
4. If error > 5%, adjust `wheel_radius`

**Rotation Test**:
1. Mark starting orientation
2. Rotate 360° (full circle)
3. Check `/odom` reported angle (should be 2π radians)
4. If error > 5%, adjust `wheel_separation`

**Formula for Correction**:
```
wheel_radius_new = wheel_radius_old * (measured_distance / reported_distance)
wheel_separation_new = wheel_separation_old * (measured_angle / reported_angle)
```

### Joystick Button Mapping

**Find Button/Axis Indices**:
```bash
# Test joystick
jstest /dev/input/js0

# Note which axis/button corresponds to:
# - Left stick vertical (typically axis 1)
# - Left stick horizontal (typically axis 0)
# - Right bumper (typically button 5)
# - Left bumper (typically button 4)

# Update joystick.yaml if needed
```

---

## Advanced Features

### SLAM (Simultaneous Localization and Mapping)

**Using slam_toolbox**:
```bash
# Install
sudo apt install ros-jazzy-slam-toolbox

# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/map
```

**Integration**:
- Uses `/scan` (LiDAR) topic
- Uses `/odom` for initial pose estimate
- Publishes `/map` and `map→odom` transform

### Autonomous Navigation (Nav2)

**Install Nav2**:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

**Launch Navigation**:
```bash
# With existing map
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map:=/path/to/map.yaml

# Set initial pose in RViz (2D Pose Estimate)
# Set navigation goal (2D Goal Pose)
```

**Requirements**:
- Map (from SLAM)
- Localization (AMCL or SLAM)
- `/scan` topic (LiDAR)
- `/odom` topic (encoders)
- `/cmd_vel` subscribed by diff_drive_controller

### Camera-Based Object Detection

**Option 1: OpenCV**:
```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
pip3 install opencv-python
```

**Option 2: Deep Learning (YOLOv8)**:
```bash
pip3 install ultralytics
# Use /camera/image_raw as input
# Publish detected objects
```

**Example Node**:
```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Process image, detect objects
        # Publish results
```

### Multi-Robot Coordination

**Using Namespaces**:
```bash
# Robot 1
ros2 launch franky_bot launch_robot.launch.py \
  namespace:=robot1

# Robot 2
ros2 launch franky_bot launch_robot.launch.py \
  namespace:=robot2
```

**Topics become**:
- `/robot1/diff_cont/cmd_vel`
- `/robot2/diff_cont/cmd_vel`

### Remote Operation (WiFi/Internet)

**Setup ROS2 Domain**:
```bash
# On robot (Raspberry Pi)
export ROS_DOMAIN_ID=42

# On remote computer (same network)
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

# Discover robot topics
ros2 topic list
```

**Use ROS2 Bridge for Internet**:
```bash
# Install
sudo apt install ros-jazzy-ros-bridge-suite

# Run bridge on robot
ros2 run rosapi rosapi_node
rosbridge_websocket
```

### Data Logging and Playback

**Record ROS2 Bag**:
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /odom /scan /camera/image_raw

# Playback
ros2 bag play rosbag2_<date>
```

**Useful for**:
- Debugging issues
- Algorithm development offline
- Performance analysis

---

## Troubleshooting Guide

### Hardware Issues

#### Motors Not Moving
**Symptoms**: No motor response, joystick input detected
**Solutions**:
1. Check Arduino power supply
2. Verify L298N connections
3. Check enable jumpers on L298N
4. Test motors directly (bypass Arduino)
5. Check serial communication:
   ```bash
   ros2 topic echo /joint_states
   # Should show changing positions
   ```

#### Encoders Not Counting
**Symptoms**: Odometry stuck at 0, wheels reported as stationary
**Solutions**:
1. Verify encoder wiring (A, B, GND, VCC)
2. Check encoder power (3.3V or 5V depending on model)
3. Monitor Arduino Serial:
   ```bash
   minicom -D /dev/ttyACM0 -b 57600
   # Send: e
   # Should show changing numbers when wheels rotate
   ```
4. Swap A/B wires if counts don't change
5. Check interrupt pins (must use 2, 3, 18, 19 on Mega)

#### Robot Drifts or Circles
**Symptoms**: Straight command causes curved path
**Solutions**:
1. **Check motor directions**: Ensure both motors rotate forward
2. **Calibrate wheel parameters**:
   - Verify `wheel_separation` measurement
   - Verify `wheel_radius` measurement
3. **Check encoder polarity**: Both should increase when moving forward
4. **Mechanical issues**: Check wheel alignment, ensure no binding

#### Serial Communication Errors
**Symptoms**: "Failed to read encoders", "Timeout", controller inactive
**Solutions**:
1. Check USB cable connection
2. Verify Arduino port:
   ```bash
   ls -l /dev/ttyACM*
   # Should exist and be readable
   ```
3. Check permissions:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   # Or add user to dialout group (permanent fix)
   ```
4. Verify baud rate matches (57600)
5. Re-upload Arduino firmware
6. Check dmesg for USB errors:
   ```bash
   dmesg | grep ttyACM
   ```

### Software Issues

#### Controllers Not Loading
**Symptoms**: `ros2 control list_controllers` shows inactive or missing
**Solutions**:
1. Check controller manager logs:
   ```bash
   ros2 run controller_manager ros2_control_node --ros-args --log-level debug
   ```
2. Verify hardware interface plugin:
   ```bash
   ros2 control list_hardware_interfaces
   # Should show left_wheel_joint and right_wheel_joint
   ```
3. Check `ros2_control.xacro` syntax
4. Rebuild workspace:
   ```bash
   cd /opt/franky_ws
   colcon build --packages-select franky_bot diffdrive_arduino
   ```

#### Joystick Not Working
**Symptoms**: Robot doesn't respond to gamepad
**Solutions**:
1. Verify joystick detection:
   ```bash
   ls /dev/input/js*
   jstest /dev/input/js0
   ```
2. Check joy topic:
   ```bash
   ros2 topic echo /joy
   # Move sticks, press buttons - should show changes
   ```
3. Check enable button (RB) is pressed
4. Verify teleop node is running:
   ```bash
   ros2 node list | grep teleop
   ```
5. Check button/axis mapping in `joystick.yaml`

#### Camera Not Publishing
**Symptoms**: No `/camera/image_raw` topic
**Solutions**:
1. Check camera device:
   ```bash
   v4l2-ctl --list-devices
   ls -l /dev/video*
   ```
2. Test camera:
   ```bash
   v4l2-ctl -d /dev/video0 --all
   # Should show camera capabilities
   ```
3. Check camera node logs
4. Try different video device (`/dev/video0` vs `/dev/video1`)
5. For Pi Camera, enable CSI interface:
   ```bash
   sudo raspi-config
   # Interface Options → Camera → Enable
   ```

#### LiDAR Not Scanning
**Symptoms**: No `/scan` topic or empty scans
**Solutions**:
1. Check LiDAR power (motor spinning?)
2. Verify USB connection
3. Check permissions:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```
4. Launch with correct serial port:
   ```bash
   ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0
   ```
5. Check RPLIDAR health:
   ```bash
   ros2 topic echo /rplidar/scan_mode
   ```

#### High Latency / Lag
**Symptoms**: Slow response to commands, delayed sensor data
**Solutions**:
1. Reduce update rates in configs
2. Disable unnecessary nodes/sensors
3. Check Raspberry Pi CPU usage:
   ```bash
   htop
   ```
4. Improve WiFi signal (if using remote operation)
5. Use Ethernet instead of WiFi
6. Overclock Raspberry Pi (carefully)

### Common Error Messages

**"Failed to load controller: diff_cont"**
- Check `my_controllers.yaml` syntax
- Verify diff_drive_controller is installed
- Ensure hardware interface is active

**"Hardware interface not available"**
- Arduino not connected or not responding
- Wrong serial port in `ros2_control.xacro`
- Arduino firmware not loaded

**"Transform timeout: odom to base_link"**
- diff_drive_controller not publishing odometry
- Check encoders are working
- Verify controller is active

**"No joy messages received"**
- Joystick not connected
- Wrong device_id in `joystick.yaml`
- joy_linux node not running

---

## Performance Optimization

### Raspberry Pi Optimization

**CPU Governor** (increase performance):
```bash
# Check current governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Set to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

**Disable Unnecessary Services**:
```bash
sudo systemctl disable bluetooth
sudo systemctl disable cups  # Printing service
sudo systemctl disable avahi-daemon  # Bonjour/mDNS
```

**Increase Swap** (if low RAM):
```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
# Set CONF_SWAPSIZE=2048 (2GB)
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

### ROS2 Optimization

**QoS Settings** (reduce latency):
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Lower latency
    durability=DurabilityPolicy.VOLATILE,       # Don't store history
    depth=1                                      # Minimal queue
)

self.create_subscription(Image, '/camera/image_raw', callback, qos)
```

**Disable Logging** (production mode):
```bash
ros2 run <package> <node> --ros-args --log-level ERROR
```

**Use Composition** (reduce overhead):
- Combine multiple nodes into single process
- Reduces inter-process communication

### Network Optimization

**For Remote Operation**:
```bash
# Compress image transport
sudo apt install ros-jazzy-compressed-image-transport

# Use compressed topics
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/compressed
```

---

## Future Enhancements

### Planned Hardware Upgrades
- [ ] IMU (Inertial Measurement Unit) for improved odometry
- [ ] Ultrasonic sensors for close-range obstacle avoidance
- [ ] GPS module for outdoor navigation
- [ ] Gripper or manipulator arm
- [ ] LED indicators for robot status
- [ ] Emergency stop button (hardware kill switch)
- [ ] Battery monitoring system with ROS2 publishing
- [ ] Second LiDAR for 3D scanning (elevated mount)

### Software Roadmap
- [ ] Full Nav2 integration with path planning
- [ ] SLAM mapping with saved maps
- [ ] Autonomous waypoint navigation
- [ ] Object detection and tracking (camera-based)
- [ ] Voice control integration
- [ ] Web dashboard for monitoring and control
- [ ] Automatic docking/charging station
- [ ] Multi-robot fleet management
- [ ] ROS2 bag recording for ML training data
- [ ] PID auto-tuning for motor controllers

### Documentation Improvements
- [ ] Video tutorials for hardware assembly
- [ ] Detailed wiring diagrams (Fritzing)
- [ ] PCB design for electronics integration
- [ ] 3D printable parts (chassis, sensor mounts)
- [ ] Calibration procedures with videos
- [ ] Troubleshooting flowcharts

---

## Project Evolution Timeline

### Phase 1: Simulation (Completed)
- ✅ Gazebo Harmonic simulation
- ✅ URDF/Xacro robot model
- ✅ ros2_control with gazebo_sim plugin
- ✅ Virtual sensors (LiDAR, camera)
- ✅ Joystick teleoperation in sim

### Phase 2: Hardware Interface (Completed)
- ✅ Arduino firmware development
- ✅ Serial communication protocol
- ✅ diffdrive_arduino hardware interface
- ✅ Encoder feedback integration
- ✅ Motor PWM control

### Phase 3: Real Robot Deployment (Current)
- ✅ Raspberry Pi setup with ROS2 Jazzy
- ✅ Real motor control with encoders
- ✅ Real camera integration (v4l2)
- ⏳ Real LiDAR integration (in progress)
- ⏳ Odometry calibration and tuning

### Phase 4: Autonomous Navigation (Future)
- ⏳ SLAM implementation
- ⏳ Nav2 path planning
- ⏳ Obstacle avoidance
- ⏳ Waypoint navigation

### Phase 5: Advanced Features (Future)
- ⏳ Object detection and tracking
- ⏳ Multi-robot coordination
- ⏳ Cloud connectivity
- ⏳ Machine learning integration

---

## References and Resources

### Official Documentation
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [ros2_control Documentation](https://control.ros.org/jazzy/index.html)
- [Differential Drive Controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)

### Hardware Documentation
- [Arduino Mega Pinout](https://www.arduino.cc/en/Main/Arduino_Board_Mega)
- [L298N Motor Driver Datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)

### Tutorials and Guides
- [Articulated Robotics YouTube](https://www.youtube.com/c/ArticulatedRobotics) - Excellent ROS2 tutorials
- [Articulated Robotics Robot Template](https://github.com/joshnewans/my_bot)
- [Josh Newans' diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino)
- [ROS2 Control Demos](https://github.com/ros-controls/ros2_control_demos)

### Community and Support
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [Raspberry Pi Forums](https://forums.raspberrypi.com/)
- [Arduino Forums](https://forum.arduino.cc/)

### Recommended Components
- **LiDAR**: RPLIDAR A1 (~$99), A2 (~$250), or Slamtec RPLIDAR C1
- **Camera**: Raspberry Pi Camera Module 3, Logitech C270/C920
- **Motors**: DC motors with 1:34 gearbox, 12V, 200 RPM
- **Encoders**: Hall effect, 6000 CPR (or calculate from PPR * gear ratio * 4)
- **SBC**: Raspberry Pi 5 (8GB), Raspberry Pi 4 (4GB+)
- **Gamepad**: Xbox One/Series controller, PS4/PS5 controller
- **Battery**: 11.1V LiPo (3S) or 12V Lead Acid, 2000+ mAh

---

## Acknowledgments

### Project Credits
- **Original Template**: [Articulated Robotics Robot Template](https://github.com/joshnewans/my_bot)
- **Hardware Interface**: Based on [Josh Newans' diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino)
- **Maintainer**: Franky (franky.bigdata@gmail.com)

### Open Source Libraries
- **ROS2**: Open Robotics Foundation
- **serial**: William Woodall, John Harrison (cross-platform serial library)
- **ros2_control**: ROS2 Control Working Group
- **Gazebo**: Open Source Robotics Foundation

### Special Thanks
- Josh Newans (Articulated Robotics) for comprehensive ROS2 tutorials
- ROS2 community for excellent documentation and support
- Open source robotics community

---

## License and Usage

**Package License**: TODO (See LICENSE.md)

**Third-Party Licenses**:
- `diffdrive_arduino`: BSD License (Josh Newans)
- `serial`: MIT License
- ROS2 packages: Apache 2.0 License

**Usage Terms**:
- Free for educational and personal use
- Commercial use: Contact maintainer
- Attribution appreciated

---

## Contact and Contribution

### Maintainer
- **Name**: Franky
- **Email**: franky.bigdata@gmail.com
- **Project**: Franky_Bot ROS2 Differential Drive Robot

### Contributing
Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly (simulation and hardware)
4. Submit pull request with detailed description

### Reporting Issues
- **Hardware Issues**: Include wiring diagram, component specs, error logs
- **Software Issues**: Include ROS2 logs, node graph, topic list
- **Documentation**: Suggest improvements or corrections

### Community
- Share your Franky_Bot build!
- Post improvements and modifications
- Help other builders in forums

---

## Appendix

### A. Quick Reference Commands

```bash
# Build workspace
cd /opt/franky_ws && colcon build && source install/setup.bash

# Launch real robot
ros2 launch franky_bot launch_robot.launch.py

# Launch simulation
ros2 launch franky_bot launch_sim.launch.py

# Launch camera
ros2 launch franky_bot pi_grok_camera.launch..py

# Monitor odometry
ros2 topic echo /odom

# List controllers
ros2 control list_controllers

# Visualize
rviz2

# Test joystick
jstest /dev/input/js0

# Check serial port
ls -l /dev/ttyACM*

# Monitor CPU
htop
```

### B. Configuration File Locations

```
/opt/franky_ws/src/
├── config/my_controllers.yaml        # Controller parameters
├── config/joystick.yaml             # Gamepad configuration
├── description/ros2_control.xacro   # Hardware interface config
└── arduino/arduino_ros_control_encoders/
    └── arduino_ros_control_encoders.ino  # Arduino firmware
```

### C. Critical Parameters

| Parameter | Location | Value | Notes |
|-----------|----------|-------|-------|
| wheel_separation | my_controllers.yaml | 0.35 m | Measure carefully! |
| wheel_radius | my_controllers.yaml | 0.05 m | Affects odometry |
| enc_counts_per_rev | ros2_control.xacro | 6000 | From encoder specs |
| baud_rate | ros2_control.xacro | 57600 | Must match Arduino |
| device | ros2_control.xacro | /dev/ttyACM0 | May vary |
| loop_rate | ros2_control.xacro | 30 Hz | Encoder polling rate |
| update_rate | my_controllers.yaml | 50 Hz | Controller update rate |

### D. Pin Reference (Arduino Mega)

| Function | Pin | Type | Notes |
|----------|-----|------|-------|
| Left Motor PWM | 5 (ENA) | PWM Out | 0-255 speed |
| Left Motor Dir 1 | 9 (IN1) | Digital Out | Forward/reverse |
| Left Motor Dir 2 | 10 (IN2) | Digital Out | Forward/reverse |
| Right Motor PWM | 6 (ENB) | PWM Out | 0-255 speed |
| Right Motor Dir 1 | 11 (IN3) | Digital Out | Forward/reverse |
| Right Motor Dir 2 | 12 (IN4) | Digital Out | Forward/reverse |
| Left Encoder A | 2 | Interrupt In | Rising edge trigger |
| Left Encoder B | 3 | Digital In | Direction sensing |
| Right Encoder A | 18 | Interrupt In | Rising edge trigger |
| Right Encoder B | 19 | Digital In | Direction sensing |

### E. Topic Reference

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| /diff_cont/cmd_vel | Twist | Variable | Velocity commands |
| /odom | Odometry | 50 Hz | Robot pose estimate |
| /joint_states | JointState | 50 Hz | Wheel positions/velocities |
| /scan | LaserScan | 10 Hz | LiDAR data |
| /camera/image_raw | Image | 15 Hz | Camera feed |
| /joy | Joy | 20 Hz | Joystick inputs |
| /tf | TFMessage | 50 Hz | Transform tree |

---

*Document Version: 2.0*  
*Last Updated: January 13, 2026*  
*ROS2 Distribution: Jazzy*  
*Status: Real Hardware Operational*  

**From Simulation to Reality - The Franky_Bot Journey Continues!** 🤖
