# Franky_Bot - ROS2 Jazzy Differential Drive Robot

## Project Overview

**Franky_Bot** is a differential drive mobile robot project built on **ROS2 Jazzy** and simulated in **Gazebo Harmonic**. This project serves as a comprehensive simulation platform that will be used as a blueprint for building a physical robot. The robot is equipped with a 2D LiDAR sensor, a camera, and gamepad teleoperation capabilities.

### Key Features
- **Differential Drive System**: Two-wheeled mobile robot with independent wheel control
- **Sensor Suite**: 
  - GPU-accelerated LiDAR (360° scanning, 12m range)
  - RGB Camera (640x480 resolution)
- **Teleoperation**: Gamepad control with turbo mode
- **ROS2 Control**: Full integration with ros2_control framework
- **Gazebo Harmonic**: Modern simulation environment with gz_ros2_control

### Platform Specifications
- **ROS Distribution**: ROS2 Jazzy
- **Simulator**: Gazebo Harmonic (formerly Ignition Gazebo)
- **Build System**: CMake (ament_cmake)
- **Package Format**: Format 3

---

## Package Structure

```
franky_bot/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── LICENSE.md                  # License file
├── README.md                   # Basic project information
│
├── config/                     # Configuration files
│   ├── empty.yaml             # Empty configuration template
│   ├── joystick.yaml          # Gamepad mapping and parameters
│   ├── my_controllers.yaml    # ros2_control controller configuration
│   └── view_bot.rviz          # RViz configuration
│
├── description/                # Robot model definitions (URDF/Xacro)
│   ├── robot.urdf.xacro       # Main robot description (includes all components)
│   ├── robot_core.xacro       # Core robot structure (chassis, wheels)
│   ├── ros2_control.xacro     # ros2_control hardware interface
│   ├── lidar.xacro            # LiDAR sensor definition
│   ├── camera.xacro           # Camera sensor definition
│   ├── inertial_macros.xacro  # Reusable inertial property macros
│   └── gazebo_control.xacro   # (Unused) Legacy Gazebo control
│
├── launch/                     # Launch files
│   ├── launch_sim.launch.py   # Main simulation launcher
│   ├── rsp.launch.py          # Robot State Publisher launcher
│   └── joystick.launch.py     # Standalone joystick node launcher
│
└── worlds/                     # Gazebo world files
    ├── empty.world            # Empty world
    ├── empty-with-shapes1.world  # World with objects for testing
    └── rviz2-config-lidar-camera.rviz  # RViz configuration
```

---

## Detailed File Analysis

### 1. Robot Description (URDF/Xacro Files)

#### **robot.urdf.xacro** - Main Robot Description
- **Purpose**: Top-level robot definition that includes all components
- **Includes**:
  - `robot_core.xacro` - Base chassis and wheels
  - `ros2_control.xacro` - Hardware interfaces
  - `lidar.xacro` - LiDAR sensor
  - `camera.xacro` - Camera sensor
- **Note**: `gazebo_control.xacro` is commented out (replaced by ros2_control)

#### **robot_core.xacro** - Robot Mechanical Structure
Defines the robot's physical structure:

**Links**:
- **base_link**: Robot's reference frame (empty link)
- **chassis_link**: Main body (300x300x150mm box, 0.5kg)
- **left_wheel_link**: Left drive wheel (50mm radius, 40mm width, 0.1kg)
- **right_wheel_link**: Right drive wheel (50mm radius, 40mm width, 0.1kg)
- **caster_wheel_link**: Front caster wheel for stability

**Joints**:
- **chassis_joint**: Fixed joint offsetting chassis from base_link (-0.1m in x)
- **left_wheel_joint**: Continuous joint for left wheel (y=0.175m)
- **right_wheel_joint**: Continuous joint for right wheel (y=-0.175m)
- **caster_wheel_joint**: Fixed joint for caster wheel

**Key Specifications**:
- Wheel separation: 350mm (0.35m)
- Wheel radius: 50mm (0.05m)
- Uses `inertial_macros.xacro` for proper mass and inertia calculations
- Includes collision and visual geometries for simulation

#### **ros2_control.xacro** - Hardware Interface
Configures the ros2_control framework:

**Hardware Interface**:
- **Plugin**: `gz_ros2_control/GazeboSimSystem`
- **Type**: System interface
- **Joints Controlled**:
  - `left_wheel_joint` (velocity command, position/velocity state)
  - `right_wheel_joint` (velocity command, position/velocity state)
- **Velocity Limits**: ±10.0 rad/s

**Gazebo Plugin**:
- Uses `gz_ros2_control::GazeboSimROS2ControlPlugin`
- Loads controller parameters from `config/my_controllers.yaml`

#### **lidar.xacro** - LiDAR Sensor Configuration
**Physical Properties**:
- Mounted on top of chassis: x=0.1m, z=0.175m
- Cylindrical shape: 50mm radius, 40mm height

**Sensor Specifications**:
- **Type**: GPU LiDAR (gpu_lidar sensor)
- **Update Rate**: 10 Hz
- **Topic**: `/scan`
- **Horizontal Scan**: 360 samples, 360° coverage (-π to +π)
- **Range**: 0.3m to 12.0m
- **Resolution**: 0.01m
- **Frame ID**: `laser_frame_link`
- **Visualization**: Enabled in Gazebo

#### **camera.xacro** - Camera Sensor Configuration
**Physical Properties**:
- Mounted at front of chassis: x=0.305m, z=0.08m
- Visual representation: 10x30x30mm box

**Optical Frame**:
- Includes `camera_optical_joint` with proper ROS camera convention rotation
- Transforms from robot frame to optical frame (X-right, Y-down, Z-forward)

**Sensor Specifications**:
- **Type**: RGB Camera
- **Update Rate**: 10 Hz
- **Topic**: `/camera`
- **Resolution**: 640x480 pixels
- **Format**: R8G8B8
- **Field of View**: 1.089 rad (~62.4°)
- **Clip Range**: 0.05m to 8.0m
- **Visualization**: Enabled in Gazebo

---

### 2. Configuration Files

#### **my_controllers.yaml** - ros2_control Controllers
Configures the controller manager and specific controllers:

**Controller Manager**:
- Update rate: 30 Hz
- Simulation time enabled

**diff_cont (Differential Drive Controller)**:
- **Type**: `diff_drive_controller/DiffDriveController`
- **Publish Rate**: 30 Hz
- **Base Frame**: `base_link`
- **Wheel Parameters**:
  - Left wheel: `left_wheel_joint`
  - Right wheel: `right_wheel_joint`
  - Wheel separation: 0.35m
  - Wheel radius: 0.05m
- **Velocity Mode**: Unstamped (use_stamped_vel: false)

**joint_broad (Joint State Broadcaster)**:
- **Type**: `joint_state_broadcaster/JointStateBroadcaster`
- Publishes joint states to `/joint_states` topic

#### **joystick.yaml** - Gamepad Configuration
**Joy Node Parameters**:
- Device ID: 0 (first connected gamepad)
- Dead zone: 0.05 (5% stick deadzone)
- Auto-repeat rate: 20 Hz
- Publishes stamped twist messages

**Teleop Node Parameters**:
- **Linear Motion**: Axis 1 (left stick vertical)
  - Normal scale: 0.5 m/s
  - Turbo scale: 1.0 m/s
- **Angular Motion**: Axis 0 (left stick horizontal)
  - Normal scale: 0.5 rad/s
  - Turbo scale: 1.0 rad/s
- **Enable Button**: Button 5 (RB - Right Bumper)
- **Turbo Button**: Button 4 (LB - Left Bumper)
- **Safety**: Requires enable button to be held

**Note**: Button mapping appears to be for Xbox-style controllers

---

### 3. Launch Files

#### **launch_sim.launch.py** - Main Simulation Launch
This is the primary launch file that brings up the complete simulation:

**Components Launched** (in order):
1. **World Configuration**:
   - Declares `world` argument with default to `empty-with-shapes1.world`
   - Allows custom world file specification

2. **Robot State Publisher** (`rsp`):
   - Includes `rsp.launch.py`
   - Forces `use_sim_time: true`

3. **Gazebo Simulator** (`gazebo`):
   - Launches `gz_sim.launch.py` from `ros_gz_sim`
   - Arguments: `-r -v4` (run immediately, verbose level 4)
   - Shutdown on exit enabled

4. **Entity Spawner** (`spawn_entity`):
   - Spawns robot from `/robot_description` topic
   - Robot name: `franky_robot`
   - Initial height: z=0.1m (prevents ground collision)

5. **ROS-Gazebo Bridge** (`bridge`):
   - Bridges between ROS2 and Gazebo topics
   - **Bridged Topics**:
     - `/cmd_vel`: ROS2→Gazebo (Twist messages)
     - `/odom`: Gazebo→ROS2 (Odometry)
     - `/tf`: Gazebo→ROS2 (TF messages)
     - `/clock`: Gazebo→ROS2 (Clock for simulation time)
     - `/scan`: Gazebo→ROS2 (LaserScan)
     - `/camera`: Gazebo→ROS2 (Image)

6. **Controller Spawners**:
   - `diff_drive_spawner`: Spawns `diff_cont` controller
   - `joint_broad_spawner`: Spawns `joint_broad` broadcaster

7. **Teleoperation Node** (`teleop_node`):
   - Package: `teleop_twist_joy`
   - Loads parameters from `joystick.yaml`
   - **Remapping**: `/cmd_vel` → `/diff_cont/cmd_vel`

**Usage**:
```bash
ros2 launch franky_bot launch_sim.launch.py
```

#### **rsp.launch.py** - Robot State Publisher
Processes the robot URDF and publishes robot state:

**Functionality**:
1. Accepts `use_sim_time` argument (default: false)
2. Processes `robot.urdf.xacro` using xacro
3. Launches `robot_state_publisher` node with:
   - Processed robot description
   - Simulation time configuration

**Purpose**: 
- Publishes static transforms (`/tf_static`)
- Converts joint states to TF transforms
- Makes robot description available on `/robot_description` topic

**Usage** (standalone):
```bash
ros2 launch franky_bot rsp.launch.py use_sim_time:=true
```

#### **joystick.launch.py** - Standalone Joy Node
Simple launcher for the joystick node:

**Functionality**:
- Launches `joy_node` from `joy` package
- Loads parameters from `joystick.yaml`

**Usage**:
```bash
ros2 launch franky_bot joystick.launch.py
```

**Note**: This is separate from the main simulation launch for testing or standalone use

---

### 4. World Files

#### **empty-with-shapes1.world**
- Currently used world (set as default in launch_sim.launch.py)
- Contains geometric shapes for sensor testing
- Useful for validating LiDAR and camera functionality

#### **empty.world**
- Minimal empty world
- Useful for focused robot testing without environmental distractions

---

## ROS2 Topics and Interfaces

### Published Topics (from Robot)
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster | Joint positions and velocities |
| `/odom` | `nav_msgs/msg/Odometry` | diff_drive_controller | Robot odometry |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR sensor (via bridge) | 360° laser scan data |
| `/camera` | `sensor_msgs/msg/Image` | Camera sensor (via bridge) | RGB image stream |
| `/tf` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | Transform tree |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | Static transforms |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | URDF/xacro robot model |

### Subscribed Topics (to Robot)
| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/diff_cont/cmd_vel` | `geometry_msgs/msg/Twist` | diff_drive_controller | Velocity commands |
| `/joy` | `sensor_msgs/msg/Joy` | teleop_twist_joy | Gamepad inputs |

---

## TF Frame Tree

```
odom
 └── base_link
      ├── chassis_link
      │    ├── laser_frame_link (LiDAR)
      │    └── camera_link
      │         └── camera_link_optical
      ├── left_wheel_link
      ├── right_wheel_link
      └── caster_wheel_link
```

**Key Frames**:
- **odom**: Odometry reference frame (published by diff_drive_controller)
- **base_link**: Robot's base reference point
- **chassis_link**: Physical chassis (offset -0.1m in x from base_link)
- **laser_frame_link**: LiDAR sensor frame
- **camera_link_optical**: Camera optical frame (ROS convention)

---

## Dependencies

### Build Dependencies
- `ament_cmake` - ROS2 build system

### Runtime Dependencies (Implicit)
Based on the launch files and configurations, the following packages are required:
- `robot_state_publisher` - Publishes robot state and TF
- `ros_gz_sim` - Gazebo simulator integration
- `ros_gz_bridge` - ROS2-Gazebo communication
- `controller_manager` - ros2_control controller management
- `diff_drive_controller` - Differential drive controller
- `joint_state_broadcaster` - Joint state publishing
- `teleop_twist_joy` - Joystick teleoperation
- `joy` - Joystick driver
- `xacro` - URDF macro processing
- `gz_ros2_control` - Gazebo-ros2_control integration

### Test Dependencies
- `ament_lint_auto`
- `ament_lint_common`

---

## Usage Instructions

### Building the Package
```bash
cd /opt/franky_ws
colcon build --packages-select franky_bot
source install/setup.bash
```

### Launching the Simulation
```bash
# Launch complete simulation with robot, sensors, and teleoperation
ros2 launch franky_bot launch_sim.launch.py

# Launch with custom world
ros2 launch franky_bot launch_sim.launch.py world:=/path/to/custom.world
```

### Teleoperation
1. Connect your gamepad (Xbox-style controller recommended)
2. Hold the **Right Bumper (RB)** button to enable movement
3. Use the **Left Stick** to control the robot:
   - Vertical axis: Forward/backward
   - Horizontal axis: Rotation
4. Hold **Left Bumper (LB)** for turbo mode (2x speed)

### Monitoring Topics
```bash
# View LiDAR data
ros2 topic echo /scan

# View camera images (requires image_view or RViz)
ros2 run image_view image_view image:=/camera

# View odometry
ros2 topic echo /odom

# View joint states
ros2 topic echo /joint_states
```

### Visualization in RViz
```bash
rviz2 -d /opt/franky_ws/src/franky_bot/config/view_bot.rviz
```

---

## Technical Considerations for Physical Robot Implementation

### Hardware Requirements
Based on the simulation configuration:

1. **Motors and Encoders**:
   - 2x DC motors with encoders
   - Velocity control capability (±10 rad/s → ~±3000 RPM @ 50mm radius)
   - Minimum 350mm wheel separation

2. **Sensors**:
   - 2D LiDAR (12m range, 360° coverage)
   - RGB Camera (minimum 640x480 resolution)
   - IMU (recommended for improved odometry)

3. **Microcontroller/SBC**:
   - Capable of running ROS2 Jazzy
   - GPIO for motor control
   - USB/Serial interfaces for sensors
   - Recommended: Raspberry Pi 4/5, Jetson Nano, or similar

### ros2_control Hardware Interface
The current configuration uses `gz_ros2_control/GazeboSimSystem`. For the physical robot:

**Required Changes**:
1. Replace in `ros2_control.xacro`:
   ```xml
   <hardware>
       <plugin>your_hardware_interface/YourRobotSystem</plugin>
   </hardware>
   ```

2. Common hardware interface options:
   - Custom hardware interface (recommended for learning)
   - `ros2_control` packages for specific hardware (e.g., Arduino, Raspberry Pi GPIO)
   - Commercial motor controller interfaces

3. Implement velocity command interfaces for wheel motors
4. Implement position/velocity feedback from encoders

### Sensor Integration
1. **LiDAR**: Most 2D LiDARs have ROS2 drivers (e.g., rplidar_ros, urg_node)
2. **Camera**: Use `usb_cam` or vendor-specific ROS2 packages
3. **Bridge Configuration**: Remove `ros_gz_bridge`, sensors publish directly to ROS2

### Controller Configuration
The `my_controllers.yaml` parameters (wheel separation, radius) must match the physical robot dimensions precisely for accurate odometry.

### Power Considerations
- Battery system with voltage regulation
- Estimated power budget: Motors (bulk), SBC (5-15W), Sensors (5-10W)
- Consider battery monitoring for safety

---

## Advanced Features and Extensibility

### Adding New Sensors
1. Create new xacro file in `description/`
2. Include in `robot.urdf.xacro`
3. Add Gazebo plugin with appropriate topic
4. Add bridge entry in `launch_sim.launch.py` if needed

### Navigation Stack Integration
The robot is ready for:
- **Nav2** integration (already has odometry, LaserScan)
- **SLAM** (e.g., slam_toolbox, cartographer)
- **Autonomous navigation** with proper map and path planning

### Simulation Testing Workflow
1. Test in simulation with `empty.world` for basic movement
2. Add obstacles using `empty-with-shapes1.world` for sensor testing
3. Create custom worlds for specific scenarios
4. Validate controller parameters before physical implementation

---

## Common Issues and Troubleshooting

### Simulation Won't Start
- Verify Gazebo Harmonic is installed: `gz sim --version`
- Check that all ROS2 dependencies are installed
- Ensure proper sourcing: `source install/setup.bash`

### Robot Falls Through Ground
- Initial spawn height set to z=0.1m in launch file
- Increase if still experiencing issues
- Check collision geometries in xacro files

### Gamepad Not Working
- Verify device detection: `ros2 run joy joy_enumerate_devices`
- Check device ID in `joystick.yaml`
- Ensure joy node is running: `ros2 node list | grep joy`
- Test raw joy output: `ros2 topic echo /joy`

### Controllers Not Loading
- Check controller manager status: `ros2 control list_controllers`
- Verify controller configuration: `ros2 param dump /controller_manager`
- Review controller spawner output in launch terminal

### Bridge Topics Not Publishing
- List Gazebo topics: `gz topic -l`
- Verify bridge is running: `ros2 node list | grep bridge`
- Check bridge configuration syntax in launch file

---

## Future Enhancements

### Planned for Physical Robot
- [ ] IMU integration for improved odometry
- [ ] Ultrasonic sensors for close-range obstacle detection
- [ ] Battery monitoring system
- [ ] Emergency stop mechanism
- [ ] LED status indicators
- [ ] Custom PCB for electronics integration

### Software Enhancements
- [ ] Nav2 integration for autonomous navigation
- [ ] SLAM for mapping
- [ ] Object detection using camera
- [ ] Multi-robot coordination
- [ ] Web-based control interface
- [ ] Data logging and analysis tools

---

## References and Resources

### Official Documentation
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ros2_control Documentation](https://control.ros.org/)
- [Nav2 Documentation](https://navigation.ros.org/)

### Tutorials
- [Articulated Robotics YouTube Channel](https://www.youtube.com/c/ArticulatedRobotics)
- [Articulated Robotics Robot Package Template](https://articulatedrobotics.github.io/robot-package-template/)

### Hardware Recommendations
- RPLIDAR A1/A2 for 2D LiDAR
- Logitech C270/C920 for camera
- Arduino/Raspberry Pi for motor control
- Xbox One controller for teleoperation

---

## License and Acknowledgments

### Project Information
- **Package**: franky_bot
- **Version**: 0.0.0
- **Maintainer**: Franky (franky.bigdata@gmail.com)
- **License**: TODO (See LICENSE.md)

### Acknowledgments
This project structure is based on the Articulated Robotics robot package template, an excellent resource for learning ROS2 robot development.

---

## Contact and Support

For questions, issues, or contributions:
- **Maintainer**: Franky 
- **Email**: franky.bigdata@gmail.com
- **Repository**: TODO (add when available)

---

*Last Updated: December 11, 2025*
*ROS2 Distribution: Jazzy*
*Simulator: Gazebo Harmonic*
