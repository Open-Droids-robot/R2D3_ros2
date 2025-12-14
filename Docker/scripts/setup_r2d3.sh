#!/usr/bin/env bash
set -euo pipefail

# R2D3 Robot Setup Script for ROS2 (Multi-Distro Support)
# This script sets up the complete R2D3 dual-arm composite lifting robot environment
# Supports: Foxy, Humble, Jazzy
# Author: Life of Droids Project
# Date: $(date +%Y-%m-%d)

echo "=== R2D3 Robot Setup Script Starting ==="
echo "Setting up OpenDroids R2D3 dual-arm composite lifting robot..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [ "$(id -u)" -eq 0 ]; then
    log_error "This script should not be run as root. Run as regular user."
    exit 1
fi

# Get current user and home directory
USERNAME=$(whoami)
USER_HOME="/home/$USERNAME"
WORKSPACE_DIR="$USER_HOME/ros2_ws"

# Detect ROS distro
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        export ROS_DISTRO="jazzy"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        export ROS_DISTRO="humble"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        export ROS_DISTRO="foxy"
    else
        log_error "No ROS2 installation detected. Please install ROS2 first."
        exit 1
    fi
fi

log_info "Current user: $USERNAME"
log_info "ROS Distro: $ROS_DISTRO"
log_info "Workspace directory: $WORKSPACE_DIR"

# 1. Install additional ROS2 packages for R2D3
log_info "Installing additional ROS2 packages for R2D3..."

# Install MoveIt2 for current ROS2 distro
sudo apt-get update

# Core ROS2 packages
sudo apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-wstool

# Install distro-specific packages
if [ "$ROS_DISTRO" = "foxy" ]; then
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-moveit-* \
        ros-${ROS_DISTRO}-gazebo-* \
        ros-${ROS_DISTRO}-rqt-*
elif [ "$ROS_DISTRO" = "humble" ]; then
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-moveit-* \
        ros-${ROS_DISTRO}-gazebo-* \
        ros-${ROS_DISTRO}-rqt-*
elif [ "$ROS_DISTRO" = "jazzy" ]; then
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-moveit-* \
        ros-${ROS_DISTRO}-gazebo-* \
        ros-${ROS_DISTRO}-rqt-*
fi

# Install additional dependencies for camera and vision
sudo apt-get install -y \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-yaml

# Install RealSense packages if available for the distro
if apt-cache search ros-${ROS_DISTRO}-realsense2 | grep -q realsense2; then
    sudo apt-get install -y ros-${ROS_DISTRO}-realsense2-*
else
    log_warning "RealSense packages not available for $ROS_DISTRO, will build from source"
fi

# Install development tools
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    dos2unix

log_success "Additional ROS2 packages installed"

# 2. Create ROS2 workspace
log_info "Setting up ROS2 workspace..."

# Ensure workspace directory exists and has correct permissions
if [ ! -d "$WORKSPACE_DIR" ]; then
    log_info "Creating workspace directory..."
    mkdir -p "$WORKSPACE_DIR/src" || {
        log_warning "Failed to create workspace directory, trying with sudo..."
        sudo mkdir -p "$WORKSPACE_DIR/src"
        sudo chown -R "$USERNAME:$USERNAME" "$WORKSPACE_DIR"
        sudo chmod -R 755 "$WORKSPACE_DIR"
    }
else
    log_info "Workspace directory exists, ensuring src directory exists..."
    # Ensure src directory exists even if workspace directory exists
    if [ ! -d "$WORKSPACE_DIR/src" ]; then
        log_info "Creating src directory..."
        mkdir -p "$WORKSPACE_DIR/src" || {
            log_warning "Failed to create src directory, trying with sudo..."
            sudo mkdir -p "$WORKSPACE_DIR/src"
        }
    fi
    sudo chown -R "$USERNAME:$USERNAME" "$WORKSPACE_DIR" 2>/dev/null || true
    sudo chmod -R 755 "$WORKSPACE_DIR" 2>/dev/null || true
fi

# Ensure ros user owns everything in the workspace
sudo chown -R "$USERNAME:$USERNAME" "$WORKSPACE_DIR" 2>/dev/null || true
sudo chmod -R 755 "$WORKSPACE_DIR" 2>/dev/null || true

cd "$WORKSPACE_DIR"

# Clone R2D3 packages to workspace
log_info "Cloning R2D3 packages to workspace..."
cd "$WORKSPACE_DIR/src"

if [ ! -d "R2D3_ros2" ]; then
    log_info "Cloning R2D3_ros2 repository..."
    git clone https://github.com/OpenDroids-robot/R2D3_ros2.git
    if [ $? -eq 0 ]; then
        log_success "R2D3_ros2 repository cloned successfully"
    else
        log_error "Failed to clone R2D3_ros2 repository"
        exit 1
    fi
else
    log_info "R2D3_ros2 already exists, updating..."
    cd R2D3_ros2
    git pull origin main || git pull origin master || log_warning "Failed to update repository"
    cd "$WORKSPACE_DIR/src"
fi

# Move packages from R2D3_ros2 to src directory
if [ -d "R2D3_ros2" ]; then
    log_info "Moving R2D3 packages to workspace src..."
    mv R2D3_ros2/* . 2>/dev/null || cp -r R2D3_ros2/* . 
    rm -rf R2D3_ros2
    log_success "R2D3 packages organized in workspace"
fi

# Ensure all packages have proper permissions
log_info "Setting proper permissions for all packages..."
sudo chown -R "$USERNAME:$USERNAME" "$WORKSPACE_DIR/src" 2>/dev/null || true
sudo chmod -R 755 "$WORKSPACE_DIR/src" 2>/dev/null || true

# 3. Install RM robot arm libraries
log_info "Installing RM robot arm libraries..."

# Check if lib files exist and install them
if [ -d "$WORKSPACE_DIR/src/ros2_rm_robot/dual_rm_driver/rm_driver/lib" ]; then
    cd "$WORKSPACE_DIR/src/ros2_rm_robot/dual_rm_driver/rm_driver/lib"
    
    # Fix line endings if needed
    if [ -f "lib_install.sh" ]; then
        dos2unix lib_install.sh 2>/dev/null || true
        chmod +x lib_install.sh
        
        # Install the libraries
        log_info "Installing RM Service libraries..."
        sudo ./lib_install.sh || log_warning "RM Service library installation may have failed"
    else
        log_warning "lib_install.sh not found, skipping RM library installation"
    fi
else
    log_warning "RM driver lib directory not found"
fi

# 4. Install chassis interface (woosh robot)
log_info "Installing chassis interface..."

if [ -d "$WORKSPACE_DIR/src/ros2_agv_robot/lib" ]; then
    cd "$WORKSPACE_DIR/src/ros2_agv_robot/lib"
    
    # Determine architecture and install appropriate package
    ARCH=$(uname -m)
    if [ "$ARCH" = "x86_64" ]; then
        if [ -f "ros-foxy-woosh-robot-agent_0.0.1-0focal_amd64.run" ]; then
            chmod +x ros-foxy-woosh-robot-agent_0.0.1-0focal_amd64.run
            sudo ./ros-foxy-woosh-robot-agent_0.0.1-0focal_amd64.run || log_warning "Woosh robot agent installation may have failed"
        fi
    elif [ "$ARCH" = "aarch64" ]; then
        if [ -f "ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run" ]; then
            chmod +x ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run
            sudo ./ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run || log_warning "Woosh robot agent installation may have failed"
        fi
    else
        log_warning "Unsupported architecture: $ARCH"
    fi
else
    log_warning "AGV robot lib directory not found"
fi

# 5. Initialize rosdep
log_info "Initializing rosdep..."
cd "$WORKSPACE_DIR"

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init || log_warning "rosdep init may have failed"
fi

# Update rosdep
rosdep update || log_warning "rosdep update may have failed"

# Add missing rosdep definitions
log_info "Adding missing rosdep definitions..."
sudo mkdir -p /etc/ros/rosdep/sources.list.d
echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
echo "yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list

# Add custom rosdep rules for missing packages
sudo mkdir -p /etc/ros/rosdep/rules.d
cat << 'EOF' | sudo tee /etc/ros/rosdep/rules.d/50-custom.rules
# Custom rosdep rules for R2D3
warehouse_ros_mongo:
  ubuntu:
    foxy: [ros-foxy-warehouse-ros-mongo]
    humble: [ros-humble-warehouse-ros-mongo]
    jazzy: [ros-jazzy-warehouse-ros-mongo]
EOF

# Update rosdep again with new rules
rosdep update || log_warning "rosdep update may have failed"

# Install dependencies
log_info "Installing ROS dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y || log_warning "Some rosdep dependencies may have failed"

# Install additional packages manually
log_info "Installing additional required packages..."
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-warehouse-ros-mongo || log_warning "Failed to install warehouse-ros-mongo"
sudo apt install -y v4l-utils || log_warning "Failed to install v4l-utils"

# Remove servo driver directory
log_info "Removing servo driver directory..."
rm -rf /home/ros/ros2_ws/src/ros2_servo_driver/servo_driver
rm -rf /home/ros/ros2_ws/src/ros2_servo_driver/servo_example
log_success "Servo driver directory removed"

# 6. Build the workspace
log_info "Building ROS2 workspace..."
cd "$WORKSPACE_DIR"

# Verify we're in the correct directory
log_info "Current directory: $(pwd)"
log_info "Workspace directory: $WORKSPACE_DIR"
if [ "$(pwd)" != "$WORKSPACE_DIR" ]; then
    log_error "Failed to change to workspace directory!"
    exit 1
fi

# Source ROS2 environment
# Fix ALL common ROS2 unbound variable errors before sourcing
# AMENT variables
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
export AMENT_PREFIX_PATH=""
export AMENT_ENVIRONMENT_HOOKS=""

# COLCON variables
export COLCON_TRACE=0
export COLCON_PREFIX_PATH=""
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3

# ROS2 variables
export ROS_PYTHON_VERSION=3

# Additional variables that might cause issues
export AMENT_CURRENT_PREFIX=""
export AMENT_SHELL_HOOKS=""
export CMAKE_PREFIX_PATH=""
export LD_LIBRARY_PATH=""
export PYTHONPATH=""

source /opt/ros/${ROS_DISTRO}/setup.bash

# Build specific packages first to resolve dependencies
log_info "Building interface packages first..."
colcon build --packages-select rm_ros_interfaces || log_warning "rm_ros_interfaces build may have failed"
colcon build --packages-select realsense2_camera_msgs || log_warning "realsense2_camera_msgs build may have failed"
colcon build --packages-select servo_interfaces || log_warning "servo_interfaces build may have failed"

# Source the built packages
source "$WORKSPACE_DIR/install/setup.bash"

# Build all packages
log_info "Building all packages..."
colcon build --symlink-install || log_warning "Some packages may have failed to build"

log_success "Workspace build completed"

# 7. Set up environment
log_info "Setting up environment..."

# Add workspace sourcing to bashrc if not already present
BASHRC_LINE="source $WORKSPACE_DIR/install/setup.bash"
if ! grep -Fxq "$BASHRC_LINE" "$USER_HOME/.bashrc"; then
    echo "" >> "$USER_HOME/.bashrc"
    echo "# R2D3 Robot Workspace" >> "$USER_HOME/.bashrc"
    echo "$BASHRC_LINE" >> "$USER_HOME/.bashrc"
    log_success "Added workspace sourcing to .bashrc"
else
    log_info "Workspace already sourced in .bashrc"
fi

# 8. Create useful aliases
log_info "Creating useful aliases..."

ALIASES="
# R2D3 Robot Aliases
alias r2d3_ws='cd $WORKSPACE_DIR'
alias r2d3_build='cd $WORKSPACE_DIR && colcon build --symlink-install'
alias r2d3_source='source $WORKSPACE_DIR/install/setup.bash'
alias r2d3_clean='cd $WORKSPACE_DIR && rm -rf build install log'
alias r2d3_camera='ros2 launch realsense2_camera rs_launch.py'
alias r2d3_demo='ros2 launch ros2_total_demo total_demo.launch.py'
"

if ! grep -q "# R2D3 Robot Aliases" "$USER_HOME/.bashrc"; then
    echo "$ALIASES" >> "$USER_HOME/.bashrc"
    log_success "Added R2D3 aliases to .bashrc"
fi

# 10. Final setup and verification
log_info "Performing final setup..."

# Source the workspace for current session
source "$WORKSPACE_DIR/install/setup.bash"

# 11. Display completion message
clear
echo ""
echo "=================================================================="
echo -e "${GREEN}R2D3 Robot Setup Completed Successfully!${NC}"
echo "=================================================================="
echo ""
echo -e "${BLUE}Workspace Location:${NC} $WORKSPACE_DIR"
echo ""
echo -e "${YELLOW}Available Commands:${NC}"
echo "  r2d3_ws          - Navigate to workspace"
echo "  r2d3_build       - Build the workspace"
echo "  r2d3_source      - Source the workspace"
echo "  r2d3_camera      - Launch camera"
echo "  r2d3_demo        - Launch demo"
echo ""
echo -e "${YELLOW}Quick Start:${NC}"
echo "  1. Open a new terminal (to load new environment)"
echo "  2. Launch camera: r2d3_camera"
echo "  3. Launch demo: r2d3_demo"
echo ""
echo -e "${YELLOW}Documentation:${NC}"
echo "  - R2D3 README: ~/ros2_ws/src/README.md"
echo "  - Service List: ~/ros2_ws/src/List_of_services_*.md"
echo ""
echo "=================================================================="
echo -e "${GREEN}Setup Complete! Please open a new terminal to start using R2D3.${NC}"
echo "=================================================================="
echo ""

log_success "R2D3 Robot setup script completed successfully!"
