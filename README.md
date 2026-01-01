# Autonomous Navigation Package

Autonomous navigation system integrating GPS-based odometry, IMU sensor fusion, Nav2, LIDAR obstacle avoidance, and direct motion control for Quantum Robotics Autonomous Mission.

## Table of Contents
- [Features](#features)
- [System Requirements](#system-requirements)
- [Hardware Requirements](#hardware-requirements)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [Sensor Fusion Architecture](#sensor-fusion-architecture)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Nodes Description](#nodes-description)
- [Topics](#topics)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features

- **Dual EKF Sensor Fusion**: Advanced robot_localization with separate local and global filtering
- **GPS-IMU-Wheel Integration**: EKF fuses GPS, IMU orientation, and wheel velocities for precise localization
- **GPS-based Odometry**: Converts GPS latitude/longitude to local odometry frame
- **IMU Integration**: Real-time orientation correction and angular velocity measurement via EKF fusion
- **Nav2 Integration**: Full autonomous navigation with dynamic obstacle avoidance
- **LIDAR Filtering**: 180° frontal scan filtering for optimized navigation
- **Direct Motion Control**: Specialized behaviors for centering and approaching targets
- **Finite State Machine**: Coordinated state management for complex behaviors
- **ZED Camera Support**: Integration with Stereolabs ZED camera for vision tasks
- **Drift Correction**: Long-range GPS corrections prevent odometry drift accumulation
- **Skid-Steering Optimization**: Angular correction factor compensates for wheel slippage

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
   - **Note**: Skid-steering robots require angular correction calibration due to wheel slippage

2. **LIDAR**
   - SLAMTEC RPLIDAR A1/A2/A3 or compatible
   - USB connection
   - 360° scanning capability (filtered to 180° frontal view)

3. **GPS Module**
   - Must publish to ROS 2 topics (see [Hardware Setup](#hardware-setup))
   - Minimum 5Hz update rate recommended (10Hz preferred)
   - GPS accuracy: <5m (RTK recommended for precision applications)
   - Used for absolute position and drift correction

4. **IMU Sensor** (Required for sensor fusion)
   - 9-DOF IMU (accelerometer, gyroscope, magnetometer)
   - Must publish orientation and angular velocity to ROS 2 topics
   - Examples: BNO055 (recommended), MPU9250, or similar
   - Minimum 20Hz update rate recommended (50-100Hz preferred)
   - **Critical**: EKF fuses IMU orientation with wheel odometry for accurate heading

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
- `robot_localization` - **EKF sensor fusion** (critical dependency)
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

### 2. Install robot_localization (Required for EKF)
```bash
sudo apt update
sudo apt install ros-humble-robot-localization -y
```

### 3. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 4. Clone Required Repositories

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

### 5. Install Dependencies
```bash
cd ~/ros2_ws/src/autonomous_nav
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### 6. Build Packages
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
    Description: Current latitude in decimal degrees (multiply raw value by 1e-7)
    Rate: ≥5 Hz recommended (10 Hz preferred)
    Note: Topic must be at root level (not /gps/latitude)
    
  /longitude:
    Type: std_msgs/Float64
    Description: Current longitude in decimal degrees (multiply raw value by 1e-7)
    Rate: ≥5 Hz recommended (10 Hz preferred)
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

### IMU Node Setup (Required for EKF)

Your IMU must publish orientation and angular velocity data:
```yaml
Required Topics:
  /bno055/imu:  # Adjust topic name based on your IMU driver
    Type: sensor_msgs/Imu
    Description: IMU orientation (quaternion), angular velocity, linear acceleration
    Rate: ≥20 Hz recommended (50-100 Hz preferred for best results)
    Frame: imu_link or base_link
    
Required Fields:
  - orientation (quaternion): Used for robot heading (EKF fuses with wheel odometry)
  - angular_velocity: Used for rotation rate (EKF fuses with wheel angular velocity)
  - orientation_covariance: Should be published by IMU (typically ~0.01 for good IMUs)
  - angular_velocity_covariance: Should be published by IMU
```

**Example IMU node launch** (BNO055):
```bash
# Example for BNO055 IMU
ros2 run bno055 bno055 --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p frame_id:=imu_link \
  -r imu:=/bno055/imu
```

**Calibration Notes:**
- BNO055 and similar IMUs may require calibration on first use
- Keep robot stationary during IMU initialization
- Magnetometer calibration improves heading accuracy (wave robot in figure-8 pattern)
- **Critical for skid-steering**: IMU compensates for wheel slippage during turns

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

### EKF Configuration (ekf.yaml)

The `config/ekf.yaml` file configures dual Extended Kalman Filters for sensor fusion:

**EKF Local** (frame: `odom`)
- Fuses wheel odometry + IMU (separate inputs)
- Provides smooth local odometry for Nav2 control
- Frequency: 20 Hz
- Publishes: `/odometry/local` and TF `odom → base_footprint`

**EKF Global** (frame: `map`)
- Fuses GPS position + wheel odometry + IMU
- Corrects long-term drift using GPS
- Frequency: 10 Hz
- Publishes: `/odometry/global` and TF `map → odom`

**Key parameters to adjust:**
```yaml
ekf_local:
  ros__parameters:
    frequency: 20.0              # Reduce to 15-20 Hz if CPU constrained
    
    # Wheel odometry input (position, velocities, angular velocity)
    odom0: /wheel/odom
    odom0_config: [false, false, false,
                   false, false, false,  # Don't use position (EKF integrates)
                   true,  true,  false,  # vx, vy (from wheels)
                   false, false, true,   # vyaw (from wheels, with correction factor)
                   false, false, false]
    
    # IMU input (orientation, angular velocity) - SEPARATE FROM WHEELS
    imu0: /bno055/imu
    imu0_config: [false, false, false,
                  false, false, true,    # yaw (from IMU - fused with wheels)
                  false, false, false,
                  false, false, true,    # vyaw (from IMU - fused with wheels)
                  false, false, false]
    imu0_remove_gravitational_acceleration: true
    
    # Process noise (model uncertainty)
    # Increase if robot motion is jerky or unpredictable
    # Decrease if robot motion is smooth and predictable
    process_noise_covariance: [0.05, ... ]  # See ekf.yaml for full matrix

ekf_global:
  ros__parameters:
    frequency: 10.0              # GPS update rate
    
    # Same wheel + IMU config as local
    odom0: /wheel/odom
    odom0_config: [...]          # Same as local
    
    imu0: /bno055/imu
    imu0_config: [...]           # Same as local
    
    # GPS input
    odom1: /gps/odom
    odom1_config: [true,  true,  false,  # x, y from GPS
                   false, false, true,   # yaw from IMU (in /gps/odom)
                   false, false, false,  # Don't use GPS velocities
                   false, false, false,
                   false, false, false]
```

### Angular Correction Factor Calibration

For skid-steering robots, calibrate the angular correction factor in `odometry.py`:
```python
# In odometry.py, line ~30
self.angular_correction = 0.8  # DEFAULT - adjust based on your robot

# Calibration procedure:
# 1. Command 360° rotation:
#    ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --rate 10
#
# 2. Wait until robot completes full rotation
#
# 3. Check actual rotation in odometry:
#    ros2 topic echo /wheel/odom --field pose.pose.orientation
#
# 4. Adjust factor:
#    - If robot rotates too much (>360°): DECREASE factor (e.g., 0.75)
#    - If robot rotates too little (<360°): INCREASE factor (e.g., 0.85)
#
# 5. Repeat until robot rotates exactly 360° (quaternion w ≈ 1.0)
#
# Typical values:
# - Asphalt/concrete: 0.85-0.95
# - Grass: 0.75-0.85
# - Sand/gravel: 0.60-0.75
```

### Odometry Covariance Tuning

Adjust covariances in `odometry.py` to control EKF sensor weighting:
```python
# In odometry.py (line ~85)

# POSE COVARIANCES
self.odom_msg.pose.covariance[0] = 0.05    # x position
self.odom_msg.pose.covariance[7] = 0.05    # y position
self.odom_msg.pose.covariance[35] = 0.5    # yaw orientation

# TWIST COVARIANCES
self.odom_msg.twist.covariance[0] = 0.05   # vx linear velocity
self.odom_msg.twist.covariance[35] = 0.5   # vyaw angular velocity

# Covariance interpretation:
# - LOWER values = EKF trusts this sensor MORE
# - HIGHER values = EKF trusts this sensor LESS
#
# For skid-steering with wheel slippage:
# - Angular covariance (0.5-1.0): Moderate confidence (wheels slip during turns)
# - IMU will have HIGHER weight (typically 0.01) for orientation
#
# For differential drive with encoders:
# - Angular covariance (0.1): Higher confidence (less slippage)
#
# Tuning guide:
# - Increase angular covariances (0.5 → 1.0 → 2.0) if:
#   * Robot operates on slippery surfaces
#   * Skid-steering with lots of wheel slip
#   * Want IMU to dominate orientation
#
# - Decrease angular covariances (0.5 → 0.3 → 0.1) if:
#   * Robot is differential drive with encoders
#   * Surfaces provide good traction
#   * Want balanced fusion of wheels + IMU
```

### Nav2 Configuration (config_nav2.yaml)

Edit `config/config_nav2.yaml` to customize Nav2 parameters:
```yaml
# Key parameters to adjust:

bt_navigator:
  ros__parameters:
    global_frame: map            # Works in GPS-corrected frame
    odom_topic: /odometry/local  # Uses EKF-fused local odometry

controller_server:
  ros__parameters:
    controller_frequency: 20.0          # Control loop frequency (Hz)
    odom_topic: /odometry/local         # IMPORTANT: Must match bt_navigator
    min_vel_x: 0.0                      # No reverse for 180° LIDAR
    max_vel_x: 0.3                      # Maximum linear velocity (m/s)
    max_vel_theta: 1.0                  # Maximum angular velocity (rad/s)

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom                # Local frame for smooth control
      width: 3                          # Costmap width (meters)
      height: 3                         # Costmap height (meters)
      resolution: 0.05                  # Cell resolution (meters)
      
      # Asymmetric footprint for 180° LIDAR (larger rear safety margin)
      footprint: "[[0.525, 0.425], [0.525, -0.425], [-0.525, -0.425], [-0.525, 0.425]]"
      
      inflation_radius: 0.70            # Safety margin around obstacles

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map                 # GPS-corrected frame
      rolling_window: true              # No pre-loaded map
      width: 15
      height: 15
      track_unknown_space: true         # Important for mapless navigation

velocity_smoother:
  ros__parameters:
    odom_topic: "odometry/local"        # IMPORTANT: Must use EKF output
```

### Transform Configuration

Adjust robot dimensions in `launch/navigation_launch.py`:
```python
# base_footprint to base_link (robot height off ground)
arguments=['0', '0', '0.40', '0', '0', '0', 'base_footprint', 'base_link']

# base_link to laser_frame (LIDAR position on robot)
arguments=['0.525', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
```

## Sensor Fusion Architecture

### Transform Tree (TF)
```
map (GPS origin, fixed world frame)
 │
 └─ [TF: map→odom] ← Published by EKF Global (GPS-corrected, updates ~10Hz)
     │
     odom (local odometry frame, drifts slowly)
      │
      └─ [TF: odom→base_footprint] ← Published by EKF Local (smooth, updates ~20Hz)
          │
          base_footprint (robot ground plane)
           │
           └─ [TF: base_footprint→base_link] ← Static (robot height)
               │
               base_link (robot center)
                │
                └─ [TF: base_link→laser_frame] ← Static (LIDAR position)
                    │
                    laser_frame (LIDAR sensor)
```

### Data Flow Diagram
```
┌──────────────────────────────────────────────────────────────┐
│                    PHYSICAL SENSORS                          │
└──────────────────────────────────────────────────────────────┘
    │              │              │
    │              │              │
 [Wheels]       [IMU]          [GPS]
 /cmd_vel     /bno055/imu   lat/lon (Float64)
    │              │              │
    ▼              │              ▼
┌─────────┐        │        ┌─────────────┐
│odometry │        │        │odom_by_gps  │
│   .py   │        │        │    .py      │
│         │        │        │             │
│vx, vy   │        │        │x, y         │
│vyaw*0.8 │◄───────┘        │yaw◄─────IMU │
└─────────┘                 └─────────────┘
    │                            │
    ▼                            ▼
/wheel/odom                  /gps/odom
[vx,vy,vyaw*0.8]            [x,y,yaw]
    │                            │
    │         /bno055/imu        │
    │         [yaw, vyaw]        │
    │              │             │
    └──────┬───────┴─────────────┘
           │
           ▼
  ┌─────────────────┐
  │  EKF LOCAL      │  Fuses wheel + IMU separately
  │  (frame: odom)  │  Integrates velocities
  └─────────────────┘  Combines wheel & IMU orientation
           │
           ├──> /odometry/local (Nav2 uses this)
           └──> TF: odom → base_footprint
                     │
      ┌──────────────┴──────────────┐
      │                             │
      ▼                             ▼
NAV2 Control                  EKF GLOBAL
(uses /odometry/local)        (frame: map)
                              Fuses GPS + wheel + IMU
                                   │
                                   ├──> /odometry/global
                                   └──> TF: map → odom
                                        (GPS drift correction)
```

### EKF Fusion Strategy

**Key difference from direct IMU integration:**

The EKF receives TWO separate orientation sources and fuses them intelligently:

1. **Wheel odometry** (`/wheel/odom`):
   - Orientation from integrated angular velocity (with correction factor)
   - Covariance: 0.5 (moderate confidence due to wheel slippage)

2. **IMU** (`/bno055/imu`):
   - Direct orientation measurement (no slippage)
   - Covariance: ~0.01 (high confidence)

3. **EKF combines both** using Kalman fusion:
   - Weights each source by inverse of covariance
   - IMU gets ~50x more weight (0.01 vs 0.5)
   - Result: ~98% IMU, ~2% wheels for orientation
   - Provides redundancy and cross-validation

**Advantages of this approach:**
- ✅ EKF handles sensor fusion (no manual coding)
- ✅ Automatic weighting based on covariances
- ✅ Redundancy: if IMU fails, wheels provide backup
- ✅ Cross-validation detects sensor failures
- ✅ Standard robot_localization approach

**Trade-offs vs direct IMU integration:**
- ⚠️ Slightly more complex configuration
- ⚠️ Requires both topics publishing correctly
- ✅ More robust to sensor failures
- ✅ Better for differential drive (less slippage)

### How Nav2 Uses GPS

Nav2 **indirectly** uses GPS through the TF tree:

1. **EKF Global** publishes `map → odom` transform (GPS-corrected)
2. **Nav2** reads `/odometry/local` (in `odom` frame)
3. **Nav2** works in `map` frame (global planning)
4. **TF system** automatically converts between frames using `map → odom` transform
5. **Result**: Nav2 plans in GPS-corrected coordinates without drift

**Example:**
- Robot position in `odom` frame: x=10.0, y=0.0 (with 0.2m drift)
- GPS correction in `map→odom` TF: x=-0.2, y=0.0
- Robot position in `map` frame: x=9.8, y=0.0 (corrected)
- Nav2 uses x=9.8 for global planning

### EKF Configuration Matrix Explanation

The `odom0_config` matrix has 15 elements representing the state vector:

| Index | Variable | Your Config | Meaning |
|-------|----------|-------------|---------|
| 0-2 | x, y, z | `false` | Position - EKF calculates by integrating velocities |
| 3-5 | roll, pitch, yaw | `false, false, false` | Orientation - EKF fuses from wheels + IMU |
| 6-8 | vx, vy, vz | `true, true, false` | Linear velocity - Uses from wheels |
| 9-11 | vroll, vpitch, vyaw | `false, false, true` | Angular velocity - Uses from wheels (corrected) |
| 12-14 | ax, ay, az | `false` | Acceleration - Not used |

The `imu0_config` matrix:

| Index | Variable | Your Config | Meaning |
|-------|----------|-------------|---------|
| 0-2 | x, y, z | `false` | Position - IMU doesn't provide position |
| 3-5 | roll, pitch, yaw | `false, false, true` | Orientation - Uses yaw from IMU |
| 6-8 | vx, vy, vz | `false` | Linear velocity - IMU doesn't provide this |
| 9-11 | vroll, vpitch, vyaw | `false, false, true` | Angular velocity - Uses vyaw from IMU |
| 12-14 | ax, ay, az | `false` | Acceleration - Not used (can enable for dynamics) |

**Why this configuration works:**
- ❌ Position not used from either sensor (EKF integrates velocities)
- ✅ Orientation fused from both wheels (low weight) + IMU (high weight)
- ✅ Linear velocities from wheels only
- ✅ Angular velocity fused from both wheels (moderate weight) + IMU (high weight)

## Usage

### Launch Complete System
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch navigation system (includes dual EKF, GPS, IMU, LIDAR, Nav2)
ros2 launch autonomous_nav navigation_launch.py
```

### Launch Without RViz
```bash
ros2 launch autonomous_nav navigation_launch.py use_rviz:=false
```

### Verify Sensor Fusion is Working
```bash
# Check EKF nodes are running
ros2 node list | grep ekf
# Should show: /ekf_local and /ekf_global

# Verify EKF subscriptions (CRITICAL - should show BOTH sources)
ros2 node info /ekf_local
# Should subscribe to:
#   /wheel/odom (wheel odometry)
#   /bno055/imu (IMU - separate input)

# Check all odometry sources
ros2 topic hz /wheel/odom      # Should be ~20-30 Hz (wheel odometry)
ros2 topic hz /bno055/imu      # Should be ~50-100 Hz (IMU)
ros2 topic hz /odometry/local  # Should be ~20 Hz (EKF fused)
ros2 topic hz /odometry/global # Should be ~10 Hz (GPS-corrected)

# Verify TF tree
ros2 run tf2_tools view_frames
evince frames_*.pdf
# Should show: map → odom → base_footprint → base_link → laser_frame

# Monitor GPS correction
ros2 run tf2_ros tf2_echo map odom
# Shows GPS drift correction in real-time

# Check robot position in map frame (GPS-corrected)
ros2 run tf2_ros tf2_echo map base_footprint

# Test angular correction factor
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --rate 10
# (Let robot complete 360° rotation, then check orientation)
ros2 topic echo /wheel/odom --field pose.pose.orientation --once
# w should be close to 1.0 for correct calibration
```

### Calibrate Angular Correction Factor

For optimal performance on your specific robot/surface:
```bash
# 1. Launch system
ros2 launch autonomous_nav navigation_launch.py

# 2. Command full rotation in terminal
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --rate 10

# 3. In another terminal, monitor orientation
ros2 topic echo /wheel/odom --field pose.pose.orientation

# 4. Wait for complete rotation, then Ctrl+C both terminals

# 5. Check final orientation quaternion:
#    - w ≈ 1.0, z ≈ 0.0 = CORRECT (exactly 360°)
#    - w < 1.0, z > 0 = OVER-rotated (decrease correction factor)
#    - w > 1.0, z < 0 = UNDER-rotated (increase correction factor)

# 6. Edit odometry.py and adjust:
nano ~/ros2_ws/src/autonomous_nav/autonomous_nav/odometry.py
# Line ~30: self.angular_correction = 0.8  # Adjust this value

# 7. Rebuild and test again
cd ~/ros2_ws
colcon build --packages-select autonomous_nav
source install/setup.bash

# 8. Repeat until w ≈ 1.0
```

### Send Navigation Goal

**Option 1: Using RViz**
1. Click "2D Goal Pose" button in RViz toolbar
2. Click and drag on map to set goal position and orientation
3. Nav2 will plan and execute path in GPS-corrected `map` frame

**Option 2: Using Command Line**
```bash
# Goal in map frame (GPS coordinates)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 5.0, y: 2.0, z: 0.0},
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
evince frames_*.pdf

# Monitor GPS origin
ros2 topic echo /gps_origin

# Check LIDAR filtering
ros2 topic hz /scan
ros2 topic hz /scan_filtered

# Monitor EKF diagnostics
ros2 topic echo /diagnostics | grep ekf

# Compare wheel vs IMU orientation (should converge)
watch -n 0.5 'echo "Wheel:" && ros2 topic echo /wheel/odom --field pose.pose.orientation --once && echo "IMU:" && ros2 topic echo /bno055/imu --field orientation --once && echo "Fused:" && ros2 topic echo /odometry/local --field pose.pose.orientation --once'
```

## Package Structure
```
autonomous_nav/
├── autonomous_nav/
│   ├── __init__.py
│   ├── odometry.py              # Wheel odometry with angular correction
│   ├── odom_by_gps.py           # GPS to odometry converter
│   ├── nav2_controller.py       # Nav2 interface controller
│   ├── move.py                  # Direct motion controller
│   ├── main_controller.py       # Finite state machine
│   └── laser_filter_180.py      # LIDAR 180° filter
├── config/
│   ├── config_nav2.yaml         # Nav2 configuration
│   └── ekf.yaml                 # EKF sensor fusion configuration
├── launch/
│   └── navigation_launch.py     # Main system launch file
├── install_dependencies.sh      # Dependency installation script
├── package.xml                  # Package manifest
├── setup.py                     # Python package setup
└── README.md                    # This file
```

## Nodes Description

### `odometry` (wheel_odometry)
Computes wheel-based odometry from cmd_vel with angular correction factor.

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Published Topics:**
- `/wheel/odom` (nav_msgs/Odometry) - Wheel odometry (position, velocities)

**Key Features:**
- Integrates linear velocities to calculate position
- Integrates angular velocity (with correction factor) to calculate orientation
- Publishes covariances for EKF weighting (moderate confidence for angular)
- Angular correction compensates for wheel slippage (calibrate for your robot)
- **Note:** Does NOT publish TF (EKF Local handles odom→base_footprint)

**Important Parameters:**
```python
self.angular_correction = 0.8  # Calibrate this for your robot
# Typical values:
# - Asphalt: 0.85-0.95
# - Grass: 0.75-0.85  
# - Sand: 0.60-0.75
```

### `odom_by_gps` (gps_odometry)
Converts GPS coordinates (latitude/longitude) to local Cartesian odometry.

**Subscribed Topics:**
- `/latitude` (std_msgs/Float64) - GPS latitude
- `/longitude` (std_msgs/Float64) - GPS longitude
- `/bno055/imu` (sensor_msgs/Imu) - IMU for orientation

**Published Topics:**
- `/gps/odom` (nav_msgs/Odometry) - GPS-based odometry in map frame
- `/gps_origin` (std_msgs/Float64MultiArray) - GPS origin coordinates

**Key Features:**
- Establishes GPS origin on first fix
- Converts lat/lon to local XY using alvinxy library
- Includes IMU orientation in GPS odometry
- Used by EKF Global for drift correction
- **Note:** Does NOT publish TF (EKF Global handles map→odom)

### `ekf_local` (robot_localization)
Local Extended Kalman Filter for smooth control odometry.

**Subscribed Topics:**
- `/wheel/odom` (nav_msgs/Odometry) - Wheel velocities + corrected angular velocity
- `/bno055/imu` (sensor_msgs/Imu) - IMU orientation + angular velocity

**Published Topics:**
- `/odometry/local` (nav_msgs/Odometry) - Fused local odometry
- `/tf` - Transform `odom → base_footprint`

**Parameters:**
- Frequency: 20 Hz
- Frame: odom
- Fuses: wheel velocities (vx, vy, vyaw*0.8), IMU orientation (yaw, vyaw)
- **Key:** Receives wheels and IMU as SEPARATE inputs, fuses intelligently

### `ekf_global` (robot_localization)
Global Extended Kalman Filter with GPS correction.

**Subscribed Topics:**
- `/wheel/odom` (nav_msgs/Odometry) - Wheel velocities
- `/bno055/imu` (sensor_msgs/Imu) - IMU orientation
- `/gps/odom` (nav_msgs/Odometry) - GPS position

**Published Topics:**
- `/odometry/global` (nav_msgs/Odometry) - GPS-corrected global odometry
- `/tf` - Transform `map → odom` (GPS drift correction)

**Parameters:**
- Frequency: 10 Hz
- Frame: map
- Fuses: GPS position (x, y), wheel velocities, IMU orientation (yaw, vyaw)

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

**Key Features:**
- Converts GPS waypoints to map frame goals
- Monitors navigation status
- Handles Nav2 action interface

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

| Topic | Type | Description | Rate |
|-------|------|-------------|------|
| `/scan` | sensor_msgs/LaserScan | Raw 360° LIDAR data | 10 Hz |
| `/scan_filtered` | sensor_msgs/LaserScan | Filtered 180° frontal scan | 10 Hz |
| `/wheel/odom` | nav_msgs/Odometry | Wheel odometry (EKF input) | 20-30 Hz |
| `/bno055/imu` | sensor_msgs/Imu | **IMU data (separate EKF input)** | 50-100 Hz |
| `/gps/odom` | nav_msgs/Odometry | GPS-based odometry (EKF input) | 5-10 Hz |
| `/odometry/local` | nav_msgs/Odometry | **EKF-fused local odometry (Nav2 uses this)** | 20 Hz |
| `/odometry/global` | nav_msgs/Odometry | EKF GPS-corrected odometry | 10 Hz |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands to robot | 20 Hz |
| `/latitude` | std_msgs/Float64 | GPS latitude input | 5-10 Hz |
| `/longitude` | std_msgs/Float64 | GPS longitude input | 5-10 Hz |
| `/gps_origin` | std_msgs/Float64MultiArray | GPS origin lat/lon | Once |
| `/tf` | tf2_msgs/TFMessage | Transform tree | 100+ Hz |

### Transform (TF) Topics

| Transform | Published By | Update Rate | Description |
|-----------|-------------|-------------|-------------|
| `map → odom` | ekf_global | 10 Hz | GPS drift correction |
| `odom → base_footprint` | ekf_local | 20 Hz | Robot position in local frame (fuses wheels + IMU) |
| `base_footprint → base_link` | static | Static | Robot height (0.40m) |
| `base_link → laser_frame` | static | Static | LIDAR position (0.525m forward) |

## Troubleshooting

### EKF Not Publishing Transforms

**Problem:** Error "Invalid frame ID odom passed to canTransform"

**Solutions:**
1. Verify EKF nodes are running:
```bash
   ros2 node list | grep ekf
   # Should show: /ekf_local and /ekf_global
```

2. **CRITICAL:** Check EKF is receiving BOTH wheel AND IMU data:
```bash
   ros2 node info /ekf_local
   # Must subscribe to BOTH:
   #   /wheel/odom
   #   /bno055/imu  ← Make sure this is present
   
   ros2 topic hz /wheel/odom
   ros2 topic hz /bno055/imu  # Should be 50-100 Hz
```

3. Verify EKF is publishing:
```bash
   ros2 topic hz /odometry/local
   ros2 run tf2_ros tf2_echo odom base_footprint
```

4. Check EKF configuration file exists and has IMU config:
```bash
   cat ~/ros2_ws/src/autonomous_nav/config/ekf.yaml | grep -A 10 "imu0:"
   # Should show imu0 configuration
```

5. **Common issue:** EKF node names must match config file:
```yaml
   # In ekf.yaml, must be:
   ekf_local:  # NOT ekf_local_node
   ekf_global: # NOT ekf_global_node
```

### EKF Not Using IMU Data

**Problem:** EKF Local subscribes to `/wheel/odom` but not `/bno055/imu`

**Solutions:**
1. Verify ekf.yaml has imu0 section:
```bash
   grep -A 15 "imu0:" ~/ros2_ws/src/autonomous_nav/config/ekf.yaml
```

2. Check IMU topic name matches:
```yaml
   # In ekf.yaml, should be:
   imu0: /bno055/imu  # Must match your IMU node's topic
```

3. Verify IMU is publishing:
```bash
   ros2 topic hz /bno055/imu
   ros2 topic echo /bno055/imu --once
```

4. Check IMU message has required fields:
```bash
   ros2 topic echo /bno055/imu --field orientation
   ros2 topic echo /bno055/imu --field angular_velocity
   # Both should have non-zero values
```

### Angular Correction Factor Wrong

**Problem:** Robot rotates too much or too little for commanded angular velocity

**Solutions:**
1. Perform calibration test:
```bash
   # Terminal 1: Monitor orientation
   watch -n 0.5 'ros2 topic echo /wheel/odom --field pose.pose.orientation.w'
   
   # Terminal 2: Command 360° rotation
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --rate 10
```

2. Adjust correction factor in `odometry.py`:
```python
   # Line ~30
   self.angular_correction = 0.8  # Adjust based on test results
   
   # If robot over-rotates (w < 1.0 after 360°):
   self.angular_correction = 0.75  # Decrease
   
   # If robot under-rotates (w > 1.0 or negative rotation):
   self.angular_correction = 0.85  # Increase
```

3. Rebuild and test again:
```bash
   cd ~/ros2_ws
   colcon build --packages-select autonomous_nav
   source install/setup.bash
```

4. **Alternative:** If calibration is difficult, increase wheel angular covariance:
```python
   # In odometry.py, line ~85
   self.odom_msg.pose.covariance[35] = 2.0  # Higher = trust IMU more
   self.odom_msg.twist.covariance[35] = 2.0
```

### EKF Performance Warnings

**Problem:** "Failed to meet update rate! Took 0.22 seconds"

**Solutions:**
1. Reduce EKF frequency in `ekf.yaml`:
```yaml
   ekf_local:
     ros__parameters:
       frequency: 20.0  # Reduce from 30.0
```

2. Check CPU usage with `htop`

3. Reduce sensor publishing rates if too high

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

4. Verify GPS publishes Float64 (not NavSatFix):
```bash
   ros2 topic echo /latitude --once
   # Should show: data: 19.432616 (example)
```

### IMU Not Publishing

**Problem:** No orientation data available or EKF not using IMU

**Solutions:**
1. Verify IMU node is running and publishing:
```bash
   ros2 node list | grep imu
   ros2 topic hz /bno055/imu
   ros2 topic echo /bno055/imu --once
```

2. Check IMU calibration (some IMUs require calibration on startup):
```bash
   # For BNO055, check calibration status
   ros2 topic echo /bno055/calib_status
```

3. Verify USB connection and permissions:
```bash
   ls -l /dev/ttyUSB*
   sudo chmod 666 /dev/ttyUSB1  # Adjust port as needed
```

4. Check IMU orientation quaternion is valid:
```bash
   ros2 topic echo /bno055/imu --field orientation
   # All values should be non-zero, w typically close to 1.0 when stationary
```

5. **Critical for EKF:** Verify IMU publishes covariances:
```bash
   ros2 topic echo /bno055/imu --field orientation_covariance
   ros2 topic echo /bno055/imu --field angular_velocity_covariance
   # Should not be all zeros
```

### Odometry Topics Missing

**Problem:** `/odometry/local` or `/wheel/odom` not publishing

**Solutions:**
1. Check if `odometry.py` node is running:
```bash
   ros2 node list | grep odometry
```

2. Verify wheel odometry receives cmd_vel:
```bash
   ros2 topic echo /cmd_vel --once
```

3. Check for Python errors in launch output

4. Rebuild package:
```bash
   cd ~/ros2_ws
   colcon build --packages-select autonomous_nav
   source install/setup.bash
```

### Nav2 Not Using Correct Odometry

**Problem:** Controller server subscribes to `/odom` instead of `/odometry/local`

**Solution:**
Add `odom_topic` parameter to `controller_server` in `config_nav2.yaml`:
```yaml
controller_server:
  ros__parameters:
    odom_topic: /odometry/local  # Add this line
```

### Orientation Drift

**Problem:** Robot orientation drifts over time even with IMU

**Solutions:**
1. **For skid-steering:** This is expected - EKF fusion should handle it

2. Check IMU is being used by EKF:
```bash
   ros2 node info /ekf_local | grep Subscribers
   # Should show /bno055/imu
```

3. Compare orientations:
```bash
   # Wheel orientation (will drift)
   ros2 topic echo /wheel/odom --field pose.pose.orientation.w
   
   # IMU orientation (accurate)
   ros2 topic echo /bno055/imu --field orientation.w
   
   # Fused orientation (should follow IMU closely)
   ros2 topic echo /odometry/local --field pose.pose.orientation.w
```

4. If fused orientation doesn't follow IMU, check covariances:
```bash
   # Increase wheel angular covariance (trust IMU more)
   # In odometry.py:
   self.odom_msg.pose.covariance[35] = 1.0  # or higher
```

5. Check IMU magnetometer calibration:
```bash
   # For BNO055
   ros2 topic echo /bno055/calib_status
   # Magnetometer should be calibrated (value 3)
```

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

**Problem:** "Could not find a connection between 'map' and 'base_footprint'"

**Solutions:**
1. Verify complete TF chain:
```bash
   ros2 run tf2_ros tf2_echo map odom        # EKF Global
   ros2 run tf2_ros tf2_echo odom base_footprint  # EKF Local
```

2. Check EKF nodes are active:
```bash
   ros2 node list | grep ekf
```

3. Verify GPS has published at least once (sets map origin):
```bash
   ros2 topic echo /gps_origin --once
```

4. View full TF tree:
```bash
   ros2 run tf2_tools view_frames
   evince frames_*.pdf
```

### Nav2 Not Planning

**Problem:** Navigation goals fail or no path is generated

**Solutions:**
1. Verify costmaps are receiving scan data:
```bash
   ros2 topic hz /scan_filtered
   ros2 topic hz /local_costmap/costmap_updates
```

2. Check robot position is known in map frame:
```bash
   ros2 run tf2_ros tf2_echo map base_footprint
```

3. Verify odometry is publishing:
```bash
   ros2 topic hz /odometry/local
```

4. Ensure goal is within costmap bounds and not in obstacle

5. Check Nav2 is using correct odometry topic:
```bash
   ros2 param get /controller_server odom_topic
   # Should return: /odometry/local
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

1. **GPS Update Rate**: Use at least 5Hz GPS for smooth odometry (10Hz preferred)
2. **LIDAR Frequency**: Ensure LIDAR publishes at 10Hz or higher
3. **IMU Rate**: 50-100Hz IMU data improves orientation fusion significantly
4. **EKF Frequency**: 
   - EKF Local: 20Hz is optimal (reduce to 15Hz if CPU constrained)
   - EKF Global: 10Hz matches typical GPS rate
5. **Costmap Resolution**: Balance between 0.05m (detailed) and 0.1m (faster)
6. **CPU Usage**: Monitor with `htop` and reduce Nav2/EKF frequencies if needed
7. **Sensor Priorities**:
   - IMU is critical (EKF fuses with wheels for accurate orientation)
   - GPS quality directly affects global accuracy
   - Wheel angular correction should be calibrated for your surface
8. **Angular Correction**:
   - Test on actual operating surface
   - May need different values for different terrains
   - Consider using lookup table for multi-terrain operation

## Advanced Configuration

### Tuning EKF for Your Robot

**For robots with high wheel slippage (sand, mud, skid-steering):**

1. **Increase wheel angular covariance** (trust IMU more):
```python
# In odometry.py
self.odom_msg.pose.covariance[35] = 1.5  # Increase from 0.5
self.odom_msg.twist.covariance[35] = 1.5
```

2. **Increase EKF process noise for angular states**:
```yaml
# In ekf.yaml
process_noise_covariance: [
  # ... (keep position/velocity values same)
  0.0,  0.0,  0.0,  0.0,  0.0,  0.15,  # Increase yaw noise (was 0.06)
  # ...
  0.0,  0.0,  0.0,  0.0,  0.0,  0.03,  # Increase vyaw noise (was 0.02)
  # ...
]
```

**For differential drive robots with encoders and low slippage:**

1. **Decrease wheel angular covariance** (trust wheels more):
```python
# In odometry.py
self.odom_msg.pose.covariance[35] = 0.1  # Higher confidence
self.odom_msg.twist.covariance[35] = 0.1
```

2. **Fine-tune angular correction**:
```python
# In odometry.py
self.angular_correction = 0.95  # Closer to 1.0 for less slippage
```

### Multi-Terrain Angular Correction

For robots operating on varying surfaces:
```python
# In odometry.py - Dynamic correction based on terrain
def __init__(self):
    # ...
    self.terrain_corrections = {
        'asphalt': 0.92,
        'grass': 0.80,
        'sand': 0.65,
        'gravel': 0.75
    }
    self.current_terrain = 'grass'  # Default
    self.angular_correction = self.terrain_corrections[self.current_terrain]

# Add subscriber to terrain classifier (if you have one)
# Or use manual mode switching
```

### GPS RTK for Centimeter Accuracy

For precision applications, use RTK GPS:
1. Reduces GPS error from meters to centimeters
2. Update rate can be 10-20Hz
3. May require base station setup
4. Dramatically improves global positioning accuracy

## Comparison with Direct IMU Integration

### This Approach (EKF Fusion)
**Pros:**
- ✅ Standard robot_localization methodology
- ✅ Automatic sensor weighting via covariances
- ✅ Redundancy: IMU failure doesn't break system
- ✅ Cross-validation detects sensor failures
- ✅ Better for differential drive (balanced fusion)
- ✅ No manual coding of sensor combination

**Cons:**
- ⚠️ Slightly more complex configuration
- ⚠️ Requires both `/wheel/odom` and `/bno055/imu` working
- ⚠️ Need to calibrate angular correction factor
- ⚠️ More topics to monitor

### Alternative (Direct IMU in Code)
**Pros:**
- ✅ Simpler configuration (one less EKF input)
- ✅ IMU orientation used directly (no fusion delay)
- ✅ Fewer topics to monitor
- ✅ Slightly lower CPU usage

**Cons:**
- ⚠️ No redundancy if IMU fails
- ⚠️ Manual integration in code
- ⚠️ IMU single point of failure for orientation
- ⚠️ Not standard robot_localization approach

**Recommendation:** Use EKF fusion (this approach) for production systems where robustness matters. Use direct IMU integration for prototyping or when simplicity is critical.

## License

Apache-2.0

## Contributors

Eduardo Chavez Martin - eduardochavezmartin10@gmail.com  
Quantum Robotics URC

## Acknowledgments

- Nav2 Navigation Stack
- robot_localization for EKF implementation
- SLAMTEC for RPLIDAR drivers
- Stereolabs for ZED SDK
- ROS 2 Community

---

For issues and feature requests, please open an issue on the repository.