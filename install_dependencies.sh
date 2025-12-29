#!/bin/bash
set -e

echo "=========================================="
echo "Instalando dependencias para autonomous_nav"
echo "=========================================="

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Verificar que ROS 2 esté instalado y sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS 2 no está sourced.${NC}"
    echo -e "${YELLOW}Ejecuta: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS 2 $ROS_DISTRO detectado${NC}"
echo ""

# Actualizar apt
echo "Actualizando lista de paquetes..."
sudo apt update

echo ""
echo "Instalando dependencias de ROS 2..."

# Instalar dependencias de ROS 2
sudo apt install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-laser-filters \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher

echo -e "${GREEN}✓ Dependencias de ROS 2 instaladas${NC}"
echo ""

# Instalar herramientas de build
echo "Instalando herramientas de compilación..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

echo -e "${GREEN}✓ Herramientas de compilación instaladas${NC}"
echo ""

# Instalar dependencias Python
echo "Instalando dependencias de Python..."
sudo apt install -y python3-numpy

echo -e "${GREEN}✓ Dependencias de Python instaladas${NC}"
echo ""

echo ""
echo -e "${GREEN}=========================================="
echo "✓ Instalación completa"
echo "==========================================${NC}"
echo ""
echo "Siguiente paso:"
echo ""
echo "  1. Ir al workspace:"
echo "     cd ~/ros2_ws"
echo ""
echo "  2. Instalar dependencias específicas del paquete:"
echo "     rosdep install --from-paths src --ignore-src -r -y"
echo ""
echo "  3. Compilar el paquete:"
echo "     colcon build --packages-select autonomous_nav"
echo ""
echo "  4. Source el workspace:"
echo "     source install/setup.bash"
echo ""
echo "  5. Lanzar el sistema:"
echo "     ros2 launch autonomous_nav main.launch.py"
echo ""