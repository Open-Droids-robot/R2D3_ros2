# Life of Droids - R2D3 Robot Development Environment

A comprehensive ROS2 Jazzy development environment with GUI and audio support, specifically configured for the **OpenDroids R2D3 dual-arm composite lifting robot**.

## Overview

This project provides a containerized ROS2 Jazzy environment with:
- **R2D3 Robot Support**: Complete setup for OpenDroids R2D3 dual-arm composite lifting robot
- **Full desktop GUI support**: X11 forwarding for RViz, Gazebo, and other GUI applications
- **Audio support**: PulseAudio integration for robot audio feedback
- **MoveIt2 Integration**: Motion planning and manipulation capabilities
- **Camera Support**: Intel RealSense D435 camera integration
- **Python & C++ development tools**: Complete robotics development stack
- **Cross-platform compatibility**: Works on Linux, macOS, and Windows (with WSL2)

## R2D3 Robot Specifications

Based on the [OpenDroids R2D3 repository](https://github.com/OpenDroids-robot/R2D3_ros2):

| Component | Hardware | Software |
|-----------|----------|----------|
| **Robotic Arms** | RM75-B (7-axis) | Controller V1.6.5+, API V4.2.8+, ROS2 V1.0.1 |
| **Camera** | Intel RealSense D435C | ros2_realsense2 |
| **Main Control** | Jetson Xavier NX | Ubuntu 20.04, ROS2 Foxy (adapted to Jazzy) |
| **Chassis** | Woosh AGV | API version 0.10.8, socket communication |
| **Head/Lift Motors** | WHJ30-80 joints | Connected as expansion axes |
| **End Effectors** | EG2-4C2 grippers/Dexterous hands | Integrated with robotic arm API |
| **Voice Module** | Fun M240 microphone array | Voice module V5.1 |

## ROS2 Distribution Support

| Distribution | Status | Recommended Use | Original R2D3 Support |
|--------------|--------|-----------------|----------------------|
| **Foxy** | ✅ Supported | Original development, legacy systems | ✅ Native (Original target) |
| **Humble** | ✅ Supported | **Recommended** for new projects | ✅ Adapted |
| **Jazzy** | ✅ Supported | Latest features, cutting-edge | ✅ Adapted |

**Recommendation**: Use **Foxy or Humble** for the best balance of stability and features. The original R2D3 was designed for Foxy, and all packages have been adapted to work with newer distributions.

## Quick Start

### Option 1: Docker Compose (Recommended)

The **easiest and most robust** way to run R2D3 with automatic setup and persistent storage.

1. **Prerequisites:**
   ```bash
   # Install Docker and Docker Compose
   # Ubuntu/Debian:
   sudo apt update && sudo apt install docker.io docker-compose-plugin
   
   # Alternative: Install Docker Desktop (includes Compose v2)
   # Or use the official Docker installation script
   
   # Add user to docker group
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. **One-Command Setup:**
   ```bash
   # Setup host environment and start default (Humble)
   make setup && make start
   
   # Or use the management script
   ./scripts/r2d3.sh start
   ```

3. **Choose Your Distribution:**
   ```bash
   # Start specific distribution
   make start DISTRO=foxy     # Original R2D3
   make start DISTRO=humble   # Recommended
   make start DISTRO=jazzy    # Latest (experimental)
   
   # Or with the script
   ./scripts/r2d3.sh start foxy
   ./scripts/r2d3.sh start humble
   ./scripts/r2d3.sh start jazzy
   ```

4. **Access the Environment:**
   ```bash
   # Open shell in running container
   make shell
   
   # Or specify distribution
   make shell DISTRO=foxy
   
   # Using the script
   ./scripts/r2d3.sh shell humble
   ```

### Option 2: DevContainer (IDE Integration)

Perfect for development with VS Code or Cursor IDE integration.

1. **Prerequisites:**
   - Docker installed and running
   - VS Code or Cursor with Dev Containers extension

2. **Setup:**
   ```bash
   # Enable X11 forwarding and audio
   make setup
   ```

3. **Launch:**
   - Open this folder in VS Code/Cursor
   - Choose distribution config:
     - **Default**: `.devcontainer/devcontainer.json` (Humble)
     - **Foxy**: `.devcontainer/devcontainer.foxy.json`
     - **Jazzy**: `.devcontainer/devcontainer.jazzy.json`
   - Click "Reopen in Container"

### Option 3: Direct Docker (Advanced)

For users who prefer direct Docker commands.

```bash
# Setup host environment
make setup

# Run specific distribution
./docker/run.foxy.sh    # Foxy
./docker/run.humble.sh  # Humble  
./docker/run.jazzy.sh   # Jazzy
```

## Features

### Core ROS2 Environment
- **Multi-Distro Support**: Choose from Foxy, Humble, or Jazzy distributions
- **MoveIt2**: Advanced motion planning framework for robotic manipulation
- **GUI Applications**: Full X11 forwarding support for RViz, Gazebo, rqt tools
- **Audio Support**: PulseAudio integration for robot audio feedback
- **Development Tools**: Python, C++, debugging tools, and robotics libraries

### R2D3 Robot Capabilities
- **Dual-Arm Control**: Simultaneous control of two RM75-B robotic arms
- **Vision System**: Intel RealSense D435C camera with depth perception
- **Mobile Base**: Woosh AGV chassis for autonomous navigation
- **Object Manipulation**: Visual object detection and grasping capabilities
- **Voice Interface**: Microphone array for voice commands and feedback
- **Simulation Support**: Gazebo simulation environment for development

### Development Features
- **Automated Setup**: Complete R2D3 environment setup with one command
- **Pre-built Packages**: All R2D3 ROS2 packages pre-compiled and ready
- **Launch Scripts**: Quick-start scripts for common operations
- **Cross-Platform**: Works on Linux, macOS, and Windows (with WSL2)

## Troubleshooting

### Audio Issues
If you encounter audio problems:
```bash
# Restart PulseAudio on host
pulseaudio --kill
pulseaudio --start

# Check audio devices in container
aplay -l
```

### GUI Issues
If GUI applications don't display:
```bash
# Reset X11 permissions
xhost -
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY
```

### Container Build Issues
If the container fails to build:
```bash
# Clean Docker cache
docker system prune -a

# Rebuild container
docker build -t ros2-jazzy-audio .devcontainer/
```

## R2D3 Robot Usage

### Docker Compose Commands

The Docker Compose setup provides the most convenient way to manage R2D3:

```bash
# Management with Makefile (Recommended)
make help                    # Show all available commands
make setup                   # Setup host environment
make start                   # Start default (Humble) environment
make start DISTRO=foxy      # Start specific distribution
make shell                   # Open shell in running container
make logs                    # Show container logs
make stop                    # Stop environment
make restart                 # Restart environment
make build                   # Build images
make clean                   # Clean up everything
make status                  # Show container status

# Management with script
./scripts/r2d3.sh help      # Show help
./scripts/r2d3.sh start     # Start default environment
./scripts/r2d3.sh start foxy # Start Foxy environment
./scripts/r2d3.sh shell     # Open shell
./scripts/r2d3.sh stop      # Stop environment
./scripts/r2d3.sh clean     # Clean up

# Direct Docker Compose (Advanced)
docker-compose --profile humble up -d    # Start Humble
docker-compose --profile foxy up -d      # Start Foxy
docker-compose --profile jazzy up -d     # Start Jazzy
docker-compose down                       # Stop current
```

### Container Environment

After the container starts, you can set up the R2D3 environment by running the setup script:

### Setup and Quick Commands

```bash
# First, run the setup script to clone R2D3 packages
~/scripts/setup_r2d3.sh

# After setup, use these commands:
r2d3_ws          # Navigate to workspace
r2d3_build       # Build the workspace
r2d3_source      # Source the workspace
r2d3_camera      # Launch camera
r2d3_demo        # Launch demo
```

### Camera Operations
```bash
# Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py

# View camera images
ros2 run rm_camera_demo sub_image_node

# Get center coordinates
ros2 run rm_camera_demo Center_Coordinate_node

# Point cloud visualization
ros2 launch realsense2_camera demo_pointcloud_launch.py
```

### Robot Control
```bash
# Launch complete robot system
ros2 launch ros2_total_demo total_demo.launch.py

# Run demo node
ros2 run ros2_total_demo total_demo_node

# Visual object grasping (gripper)
ros2 run ros2_total_demo catch2object_gripper.py

# Visual object grasping (dexterous hand)
ros2 run ros2_total_demo catch2object_aoyi_hand.py
```

### Launch Scripts
Pre-built scripts are available in `~/r2d3_scripts/`:

## Development

### Workspace Structure
```
~/ros2_ws/                    # Main ROS2 workspace
├── src/                      # Source packages
│   ├── ros2_rm_robot/       # Robotic arm control
│   ├── ros2_realsense2/     # Camera integration
│   ├── ros2_agv_robot/      # Mobile base control
│   ├── ros2_servo_driver/   # Servo motor control
│   └── ros2_total_demo/     # Complete system demos
├── build/                    # Build artifacts
├── install/                  # Installed packages
└── log/                      # Build logs
```

### Development Tools
The container includes:
- **ROS2 Jazzy**: Complete desktop installation with all tools
- **MoveIt2**: Motion planning and manipulation framework
- **Gazebo**: 3D robot simulation environment
- **RViz2**: 3D visualization tool for robotics
- **Python 3**: Scientific libraries (NumPy, OpenCV, SciPy, Matplotlib)
- **C++ Tools**: Build-essential, CMake, debugging tools
- **Intel RealSense SDK**: Camera drivers and utilities

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test in both devcontainer and run.sh modes
5. Submit a pull request

## License

[Add your license information here]
