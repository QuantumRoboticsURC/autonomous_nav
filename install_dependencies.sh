#!/bin/bash
set -e
echo "=========================================="
echo "Installing dependencies for autonomous_nav"
echo "=========================================="
# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color
# Verify that ROS 2 is installed and sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS 2 is not sourced.${NC}"
    echo -e "${YELLOW}Run: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS 2 $ROS_DISTRO detected${NC}"
echo ""
# Update apt
echo "Updating package list..."
sudo apt update
echo ""
echo "Installing ROS 2 dependencies..."
# Install ROS 2 dependencies
sudo apt install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-laser-filters \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher
echo -e "${GREEN}✓ ROS 2 dependencies installed${NC}"
echo ""
# Install build tools
echo "Installing build tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip
echo -e "${GREEN}✓ Build tools installed${NC}"
echo ""
# Install Python dependencies
echo "Installing Python dependencies..."
sudo apt install -y python3-numpy
sudo apt install -y python3-opencv ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-opencv 
echo -e "${GREEN}✓ Python dependencies installed${NC}"
echo ""
echo ""
echo -e "${GREEN}=========================================="
echo "✓ Installation complete"
echo "==========================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "  1. Go to workspace:"
echo "     cd ~/ros2_ws"
echo ""
echo "  2. Build the package:"
echo "     colcon build --packages-select autonomous_nav"
echo ""
echo "  3. Source the workspace:"
echo "     source install/setup.bash"
echo ""
echo "  4. Launch the system:"
echo "     ros2 launch autonomous_nav nav2_launch.py"
echo ""