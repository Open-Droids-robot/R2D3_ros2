# Life of Droids - R2D3 Robot Workspace Overview

## 📁 Workspace Structure

```
life_of_droids/
├── 📋 Documentation
│   ├── README.md                    # Main project documentation
│   ├── QUICKSTART.md               # 5-minute setup guide
│   ├── DISTRO_GUIDE.md            # ROS2 distribution guide
│   └── WORKSPACE_OVERVIEW.md       # This file
│
├── 🐳 Docker Infrastructure
│   ├── docker-compose.yml          # Multi-distro Docker Compose
│   ├── Makefile                    # Convenient make commands
│   ├── env.example                 # Environment configuration template
│   └── .gitignore                  # Git ignore rules
│
├── 🐳 Docker Files
│   └── docker/
│       ├── Dockerfile.foxy         # ROS2 Foxy container
│       ├── Dockerfile.humble       # ROS2 Humble container
│       ├── Dockerfile.jazzy        # ROS2 Jazzy container
│       ├── run.foxy.sh            # Direct Docker run script (Foxy)
│       ├── run.humble.sh          # Direct Docker run script (Humble)
│       └── run.jazzy.sh           # Direct Docker run script (Jazzy)
│
├── 📦 DevContainer Setup (IDE Integration)
│   └── .devcontainer/
│       ├── devcontainer.json       # Default (Humble) IDE config
│       ├── devcontainer.foxy.json  # Foxy IDE config
│       ├── devcontainer.humble.json # Humble IDE config
│       └── devcontainer.jazzy.json # Jazzy IDE config
│
├── 🤖 Robot Source Code
│   └── R2D3_ros2/                 # OpenDroids R2D3 robot packages
│       ├── ros2_rm_robot/         # Robotic arm control
│       ├── ros2_realsense2/       # Camera integration
│       ├── ros2_agv_robot/        # Mobile base control
│       ├── ros2_servo_driver/     # Servo motor control
│       └── ros2_total_demo/       # Complete system demos
│
└── 🛠️ Management Tools & Scripts
    └── scripts/
        ├── r2d3.sh                # Comprehensive management script
        ├── setup_devcontainer.sh  # Base environment setup
        └── setup_r2d3.sh          # R2D3 robot setup
```

## 🚀 Usage Patterns

### 1. Quick Development (Recommended)
```bash
make setup && make start    # One-command setup
make shell                  # Enter development environment
```

### 2. IDE Integration
```bash
# Open in VS Code/Cursor with devcontainer support
code .  # Will prompt to reopen in container
```

### 3. Production Deployment
```bash
make build DISTRO=humble    # Build production image
make start DISTRO=humble    # Start with specific distro
```

### 4. Multi-Distribution Development
```bash
make build-all              # Build all distributions
make start DISTRO=foxy      # Develop with Foxy
make start DISTRO=humble    # Develop with Humble
make start DISTRO=jazzy     # Develop with Jazzy
```

## 🎯 Key Features

### Multi-Distribution Support
- **Foxy**: Original R2D3 target, maximum compatibility
- **Humble**: Recommended for production, LTS support
- **Jazzy**: Latest features, cutting-edge development

### Persistent Storage
- **Workspaces**: Separate volumes per distribution
- **Home directories**: Persistent user settings
- **Build artifacts**: Cached between sessions

### Automatic Setup
- **Environment detection**: Auto-detects ROS distribution
- **Package installation**: Installs correct packages per distro
- **Workspace building**: Automatically builds R2D3 packages
- **Alias creation**: Sets up convenient commands

### Development Tools
- **Make targets**: Simple commands for common tasks
- **Management script**: Comprehensive CLI tool
- **IDE integration**: DevContainer support
- **Direct Docker**: Traditional Docker workflows

## 🔧 Configuration

### Environment Variables (.env)
```bash
USER_UID=1000              # Host user ID
USER_GID=1000              # Host group ID
ROS_DOMAIN_ID=1            # ROS2 domain ID
DISPLAY=${DISPLAY}         # X11 display
```

### Docker Compose Profiles
- **default**: Humble distribution (recommended)
- **foxy**: Foxy distribution (original)
- **humble**: Humble distribution (explicit)
- **jazzy**: Jazzy distribution (latest)

### Persistent Volumes
- `r2d3_*_workspace`: ROS2 workspace per distribution
- `r2d3_*_home`: User home directory per distribution

## 🛠️ Maintenance

### Regular Tasks
```bash
make status                 # Check system status
make logs                   # Monitor container logs
make build                  # Rebuild after changes
```

### Cleanup
```bash
make clean                  # Remove containers and volumes
docker system prune -a      # Clean Docker system
```

### Updates
```bash
git pull                    # Update source code
make build                  # Rebuild images
make restart                # Restart with new images
```

## 🐛 Troubleshooting

### Common Issues
1. **Permission errors**: Check Docker group membership
2. **GUI not working**: Run `make setup` for X11 forwarding
3. **Audio issues**: Ensure PulseAudio is running
4. **Build failures**: Try `make clean && make build`

### Debug Commands
```bash
make status                 # Check container status
make logs                   # View container logs
docker ps -a               # List all containers
docker images              # List all images
```

## 📚 Learning Path

1. **Start here**: `QUICKSTART.md` - Get running in 5 minutes
2. **Understand distributions**: `DISTRO_GUIDE.md`
3. **Deep dive**: `README.md` - Complete documentation
4. **Explore code**: `R2D3_ros2/` - Robot packages
5. **Customize**: Modify `docker-compose.yml` and scripts

## 🤝 Contributing

1. **Fork** the repository
2. **Create** a feature branch
3. **Develop** with multiple distributions
4. **Document** your changes
5. **Submit** a pull request

---

**Happy Robot Development! 🤖**

*This workspace provides a complete, production-ready development environment for the OpenDroids R2D3 dual-arm composite lifting robot with support for multiple ROS2 distributions.*
