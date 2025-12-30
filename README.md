# Autonomous Navigation Package

Autonomous navigation system integrating GPS-based odometry, Nav2, LIDAR obstacle avoidance, and direct motion control for Quantum Robotics Autonomous Mission .

## Table of Contents
- [Features](#features)
- [System Requirements](#system-requirements)
- [Hardware Requirements](#hardware-requirements)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Nodes Description](#nodes-description)
- [Topics](#topics)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features

- **GPS-based Odometry**: Converts GPS latitude/longitude to local odometry frame
- **Nav2 Integration**: Full autonomous navigation with dynamic obstacle avoidance
- **LIDAR Filtering**: 180° frontal scan filtering for optimized navigation
- **Direct Motion Control**: Specialized behaviors for centering and approaching targets
- **Finite State Machine**: Coordinated state management for complex behaviors
- **ZED Camera Support**: Integration with Stereolabs ZED camera for vision tasks
- **IMU Integration**: Orientation correction and sensor fusion

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Memory**: 4GB RAM minimum (8GB recommended)
- **Storage**: 20GB free space

## Hardware Requirements

### Required Hardware

1. **Mobile Robot Platform**
   - Differential drive or skid-steering configuration
   - Motor controllers compatible with ROS 2

2. **LIDAR**
   - SLAMTEC RPLIDAR A1/A2/A3 or compatible
   - USB connection
   - 360° scanning capability

3. **GPS Module**
   - Must publish to ROS 2 topics (see [Hardware Setup](#hardware-setup))
   - Minimum 5Hz update rate recommended
   - GPS accuracy: <5m (RTK recommended for precision applications)

4. **IMU Sensor**
   - 9-DOF IMU (accelerometer, gyroscope, magnetometer)
   - Must publish orientation data to ROS 2 topics
   - Examples: BNO055, MPU9250, or similar

5. **ZED Camera** (Optional for vision features)
   - ZED, ZED Mini, or ZED 2/2i
   - USB 3.0 connection
   - CUDA-compatible GPU recommended

6. **Onboard Computer**
   - Jetson series (Orin recommended)
   - NVIDIA GPU (for ZED camera processing)
   - Multiple USB ports

## Dependencies

### ROS 2 Packages
- `navigation2` - Nav2 navigation stack
- `nav2_bringup` - Nav2 launch files and configurations
- `tf2_ros` - Transform library
- `tf2_geometry_msgs` - Geometry message transforms
- `sllidar_ros2` - SLAMTEC RPLIDAR driver (must be cloned from source)
- `laser_filters` - Laser scan filtering tools
- `rviz2` - Visualization tool
- `robot_state_publisher` - Robot state broadcasting
- `cv_bridge` - OpenCV-ROS bridge
- `vision_opencv` - Vision processing tools

### System Packages
- `python3-numpy` - Numerical computing
- `python3-opencv` - Computer vision library
- `python3-colcon-common-extensions` - Build tools
- `python3-rosdep` - Dependency management

### External SDKs

#### ZED SDK (Required for camera features)
- **Version**: 4.0+ recommended
- **Download**: [Stereolabs ZED SDK](https://www.stereolabs.com/developers/release/)

Ensure ZED SDK is installed

- **ZED ROS 2 Wrapper (optional)**:
```bash
  cd ~/ros2_ws/src
  git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
  cd ~/ros2_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

## Installation

### 1. Prerequisites

Ensure ROS 2 Humble is installed and sourced:
```bash
source /opt/ros/humble/setup.bash
```

### 2. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone Required Repositories

#### Clone autonomous_nav package
```bash
git clone https://github.com/QuantumRoboticsURC/autonomous_nav.git
```

**Note:** Make sure you're on the correct branch for your robot configuration.

#### Clone SLAMTEC LIDAR driver
```bash
git clone https://github.com/Slamtec/sllidar_ros2.git
```

**Important:** After cloning, you must modify the LIDAR launch file:
```bash
# Open the launch file
nano ~/ros2_ws/src/sllidar_ros2/launch/sllidar_a1_launch.py
```

Find this line:
```python
frame_id = LaunchConfiguration('frame_id', default='laser')
```

Change it to:
```python
frame_id = LaunchConfiguration('frame_id', default='laser_frame')
```

Save and exit (Ctrl+X, then Y, then Enter).

### 4. Install Dependencies
```bash
cd ~/ros2_ws/src/autonomous_nav
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### 5. Build Packages
```bash
cd ~/ros2_ws
colcon build --packages-select sllidar_ros2 autonomous_nav
source install/setup.bash
```

## Hardware Setup

### GPS Node Setup

Your GPS module must publish to the following topics:
```yaml
Required Topics:
  /latitude:
    Type: std_msgs/Float64
    Description: Current latitude in decimal degrees
    Rate: ≥5 Hz recommended
    Note: Topic must be at root level (not /gps/latitude)
    
  /longitude:
    Type: std_msgs/Float64
    Description: Current longitude in decimal degrees
    Rate: ≥5 Hz recommended
    Note: Topic must be at root level (not /gps/longitude)
```

**Example GPS node launch** (adapt to your GPS hardware):
```bash
# Example for NMEA GPS via serial
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=9600 \
  -r /fix:=/gps/fix

# Then remap to Float64 messages if needed
# (Create a simple converter node if your GPS publishes sensor_msgs/NavSatFix)
```

### IMU Node Setup

Your IMU must publish orientation data:
```yaml
Required Topics:
  /imu/data:
    Type: sensor_msgs/Imu
    Description: IMU orientation, angular velocity, linear acceleration
    Rate: ≥20 Hz recommended
    Frame: base_link or imu_link
```

**Example IMU node launch** (BNO055):
```bash
# Example for BNO055 IMU
ros2 run bno055 bno055 --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p frame_id:=imu_link
```

### LIDAR Setup

**Note:** The main launch file configures the LIDAR automatically. These commands are only for testing independently.

#### Configure LIDAR port permissions:
```bash
# One-time setup
sudo chmod 666 /dev/ttyUSB0

# Or permanently add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again
```

#### Test LIDAR independently:
```bash
# Launch LIDAR node
ros2 launch sllidar_ros2 sllidar_a1_launch.py

# In another terminal, check scan output
ros2 topic hz /scan
ros2 topic echo /scan --once
```

**Note:** Make sure you modified the `frame_id` to `'laser_frame'` in the launch file as described in the installation section.

### ZED Camera Setup

**Note:** The main launch file can include ZED camera setup. These commands are for testing independently.

Launch ZED camera node:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

Verify camera is publishing:
```bash
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/image_rect_color
```

## Configuration

### Main Configuration File

Edit `config/config_nav2.yaml` to customize Nav2 parameters:
```yaml
# Key parameters to adjust:

controller_server:
  ros__parameters:
    controller_frequency: 20.0          # Control loop frequency (Hz)
    min_vel_x: 0.1                      # Minimum linear velocity (m/s)
    max_vel_x: 0.5                      # Maximum linear velocity (m/s)
    max_vel_theta: 1.0                  # Maximum angular velocity (rad/s)

local_costmap:
  local_costmap:
    ros__parameters:
      width: 6                          # Costmap width (meters)
      height: 6                         # Costmap height (meters)
      resolution: 0.05                  # Cell resolution (meters)
      robot_radius: 0.25                # Robot radius for collision checking
```

### Transform Configuration

Adjust robot dimensions in `launch/nav2_launch.py`:
```python
# base_footprint to base_link (robot height)
arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link']

# base_link to laser_frame (LIDAR position on robot)
arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
```

## Usage

### Launch Complete System
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch navigation system
ros2 launch autonomous_nav nav2_launch.py
```

### Launch Without RViz
```bash
ros2 launch autonomous_nav nav2_launch.py use_rviz:=false
```

### Send Navigation Goal

**Option 1: Using RViz**
1. Click "2D Goal Pose" button in RViz toolbar
2. Click and drag on map to set goal position and orientation

**Option 2: Using Command Line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### Monitor System Status
```bash
# Check all nodes are running
ros2 node list

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Check transform tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitor GPS origin
ros2 topic echo /gps_origin

# Check LIDAR filtering
ros2 topic hz /scan
ros2 topic hz /scan_filtered
```

## Package Structure
```
autonomous_nav/
├── autonomous_nav/
│   ├── __init__.py
│   ├── odom_by_gps.py           # GPS to odometry converter
│   ├── nav2_controller.py        # Nav2 interface controller
│   ├── move.py                   # Direct motion controller
│   ├── main_controller.py        # Finite state machine
│   └── laser_filter_180.py       # LIDAR 180° filter
├── config/
│   └── config_nav2.yaml          # Nav2 configuration
├── launch/
│   └── nav2_launch.py            # Main system launch file
├── install_dependencies.sh       # Dependency installation script
├── package.xml                   # Package manifest
├── setup.py                      # Python package setup
└── README.md                     # This file
```

## Nodes Description

### `odom_by_gps`
Converts GPS coordinates (latitude/longitude) to local Cartesian odometry.

**Subscribed Topics:**
- `/latitude` (std_msgs/Float64)
- `/longitude` (std_msgs/Float64)

**Published Topics:**
- `/odom` (nav_msgs/Odometry)
- `/gps_origin` (std_msgs/Float64MultiArray)

**Published TF:**
- `odom` → `base_footprint`

### `laser_filter_180`
Filters 360° LIDAR scan to 180° frontal coverage (-90° to +90°).

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan)

**Published Topics:**
- `/scan_filtered` (sensor_msgs/LaserScan)

**Parameters:**
- `lower_angle`: -1.5708 rad (-90°)
- `upper_angle`: 1.5708 rad (+90°)

### `nav2_controller`
Interfaces with Nav2 navigation stack for autonomous path planning and execution.

**Action Servers:**
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose)

### `direct_motion_controller`
Provides direct motion control for specialized behaviors (centering, approaching, searching).

**Subscribed Topics:**
- `/cmd_vel` commands from state machine

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist)

### `main_controller`
Finite state machine coordinating all robot behaviors.

**States:**
- IDLE
- NAVIGATE
- CENTER_AND_APPROACH
- SEARCH

## Topics

### Key System Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Raw 360° LIDAR data |
| `/scan_filtered` | sensor_msgs/LaserScan | Filtered 180° frontal scan |
| `/odom` | nav_msgs/Odometry | Robot odometry from GPS |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands to robot |
| `/latitude` | std_msgs/Float64 | GPS latitude input |
| `/longitude` | std_msgs/Float64 | GPS longitude input |
| `/imu/data` | sensor_msgs/Imu | IMU orientation data |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

## Troubleshooting

### GPS Not Publishing

**Problem:** No data on `/latitude` or `/longitude`

**Solutions:**
1. Check GPS module is connected:
```bash
   ls /dev/ttyACM* /dev/ttyUSB*
```

2. Verify GPS node is running:
```bash
   ros2 node list | grep gps
```

3. Check GPS has satellite fix (may take 30-60 seconds outdoors)

### LIDAR Not Detected

**Problem:** Error "cannot bind to serial port /dev/ttyUSB0"

**Solutions:**
1. Check port permissions:
```bash
   sudo chmod 666 /dev/ttyUSB0
```

2. Verify LIDAR is connected:
```bash
   ls -l /dev/ttyUSB*
```

3. Try different port:
```bash
   ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

4. Verify you modified the `frame_id` in the launch file to `'laser_frame'`

### LIDAR Frame ID Mismatch

**Problem:** Transform errors like "frame [laser] does not exist"

**Solution:**
Make sure you modified the LIDAR launch file as described in the installation section:
```bash
nano ~/ros2_ws/src/sllidar_ros2/launch/sllidar_a1_launch.py
# Change: frame_id = LaunchConfiguration('frame_id', default='laser')
# To:     frame_id = LaunchConfiguration('frame_id', default='laser_frame')
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

### Transform Errors

**Problem:** "Could not find a connection between 'odom' and 'base_footprint'"

**Solutions:**
1. Verify `odom_by_gps` is running and receiving GPS data:
```bash
   ros2 topic hz /latitude
   ros2 topic echo /odom --once
```

2. Check transform tree:
```bash
   ros2 run tf2_ros tf2_echo odom base_footprint
```

3. Ensure GPS has published at least once to set origin

### Nav2 Not Planning

**Problem:** Navigation goals fail or no path is generated

**Solutions:**
1. Verify costmaps are receiving scan data:
```bash
   ros2 topic hz /scan_filtered
   ros2 topic hz /local_costmap/costmap_updates
```

2. Check robot position is known:
```bash
   ros2 topic echo /odom --once
```

3. Ensure goal is within costmap bounds and not in obstacle

### IMU Not Publishing

**Problem:** No orientation data available

**Solutions:**
1. Verify IMU node is running:
```bash
   ros2 node list | grep imu
   ros2 topic hz /imu/data
```

2. Check IMU calibration (some IMUs require calibration on startup)

3. Verify USB connection and permissions:
```bash
   ls -l /dev/ttyUSB*
   sudo chmod 666 /dev/ttyUSB1  # Adjust port as needed
```

### ZED Camera Issues

**Problem:** ZED camera not publishing images

**Solutions:**
1. Verify ZED SDK installation:
```bash
   /usr/local/zed/tools/ZED_Diagnostic
```

2. Check USB 3.0 connection (ZED requires USB 3.0)

3. Launch ZED node separately to debug:
```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

4. Check CUDA is properly installed (for GPU processing):
```bash
   nvidia-smi
```

## Performance Tips

1. **GPS Update Rate**: Use at least 5Hz GPS for smooth odometry
2. **LIDAR Frequency**: Ensure LIDAR publishes at 10Hz or higher
3. **IMU Rate**: 20Hz+ IMU data improves orientation accuracy
4. **Costmap Resolution**: Balance between 0.05m (detailed) and 0.1m (faster)
5. **CPU Usage**: Monitor with `htop` and reduce Nav2 frequencies if needed

## License

Apache-2.0

## Contributors

Eduardo Chavez Martin - eduardochavezmartin10@gmail.com  
Quantum Robotics URC

## Acknowledgments

- Nav2 Navigation Stack
- SLAMTEC for RPLIDAR drivers
- Stereolabs for ZED SDK
- ROS 2 Community

---

For issues and feature requests, please open an issue on the repository.