# R2D3 Robot Quick Start Guide

Get up and running with the R2D3 robot in under 5 minutes!

## 🚀 Super Quick Start

```bash
# 1. Clone and enter directory
git clone https://github.com/Open-Droids-robot/R2D3_ros2.git
cd R2D3_ros2/Docker

# 2. One command setup and start
make setup && make start DISTRO=foxy

# 3. Open shell in container
make shell DISTRO=foxy

# Note: Container hostname will automatically use your system hostname
# instead of the default r2d3-foxy. This ensures proper ROS2 node identification.


# 4. Setup R2D3 packages
~/scripts/setup_r2d3.sh
```

That's it! You now have a fully functional R2D3 robot development environment.

## 📋 Prerequisites

- **Docker**: `sudo apt install docker.io docker-compose`
- **User permissions**: `sudo usermod -aG docker $USER` (then logout/login)
- **X11 server**: Usually pre-installed on Linux desktops

## 🎯 Choose Your ROS2 Distribution

| Command | Distribution | Best For |
|---------|-------------|----------|
| `make start` | Humble (default) | **Recommended** - Production use |
| `make start DISTRO=foxy` | Foxy | Original R2D3 compatibility |
| `make start DISTRO=jazzy` | Jazzy | Latest features (experimental) |

## 🛠️ Essential Commands

```bash
# Environment Management
make start              # Start environment
make shell              # Open shell
make stop               # Stop environment
make restart            # Restart environment
make status             # Check status

# Development
make build              # Build images
make logs               # View logs
make clean              # Clean up everything

# R2D3 Operations (inside container)
r2d3_camera            # Launch camera
r2d3_demo              # Launch robot demo
r2d3_build             # Build workspace
r2d3_ws                # Go to workspace
```

## 🎮 Try the Robot

Once inside the container:

```bash
# Launch camera
r2d3_camera

# In another terminal (make shell)
# Launch robot demo
r2d3_demo

# Launch visual grasping
ros2 run ros2_total_demo catch2object_gripper.py
```

## 🐛 Troubleshooting

### Container won't start
```bash
# Check Docker is running
sudo systemctl start docker

# Check permissions
groups $USER  # Should include 'docker'
```

### GUI applications don't show
```bash
# Setup X11 forwarding
make setup
# Or manually:
xhost +local:docker
```

### Audio doesn't work
```bash
# Start PulseAudio
pulseaudio --start

# Check audio devices in container
aplay -l
```

### Build failures
```bash
# Clean and rebuild
make clean
make build
```

### Hostname issues
```bash
# Container hostname automatically uses your system hostname
# To override, set HOSTNAME environment variable:
HOSTNAME=my-custom-hostname make start DISTRO=humble

# Check current hostname:
hostname
```

### Permission denied errors
```bash
# If you get "Permission denied" when creating workspace:
make fix-permissions DISTRO=humble

# This ensures the ros user has full access to all files and directories
# The ros user can now create, edit, and delete any file in the container

# Or restart the container to apply permission fixes:
make restart DISTRO=humble
```

## 📚 Next Steps

1. **Read the full README.md** for detailed information
2. **Check DISTRO_GUIDE.md** for distribution-specific details
3. **Explore the R2D3_ros2/** directory for robot packages
4. **Modify docker-compose.yml** for custom configurations

## 🆘 Getting Help

- **Commands**: `make help` or `./scripts/r2d3.sh help`
- **Status**: `make status`
- **Logs**: `make logs`
- **Documentation**: Check README.md and DISTRO_GUIDE.md

---

**Happy Robot Development! 🤖**
