# ROS2 Distribution Guide for R2D3 Robot

This guide helps you choose and use the right ROS2 distribution for your R2D3 robot development.

## Quick Reference

| Distribution | Ubuntu | Status | Use Case |
|--------------|--------|--------|----------|
| **Foxy** | 20.04 | ✅ Supported | Original R2D3 development, legacy compatibility |
| **Humble** | 22.04 | ✅ **Recommended** | Production use, best stability |
| **Jazzy** | 24.04 | ✅ Supported | Latest features, experimental - **some scripts may not work** |

## File Structure

```
.devcontainer/
├── Dockerfile.foxy          # ROS2 Foxy container
├── Dockerfile.humble        # ROS2 Humble container  
├── Dockerfile.jazzy         # ROS2 Jazzy container
├── devcontainer.json        # Default (Humble)
├── devcontainer.foxy.json   # Foxy configuration
├── devcontainer.humble.json # Humble configuration
├── devcontainer.jazzy.json  # Jazzy configuration
├── run.foxy.sh             # Foxy Docker run script
├── run.humble.sh           # Humble Docker run script
├── run.jazzy.sh            # Jazzy Docker run script
└── scripts/
    ├── setup_devcontainer.sh # Base environment setup
    └── setup_r2d3.sh         # R2D3 robot setup (multi-distro)
```

## Usage Instructions

### DevContainer Method

1. **Default (Humble)**:
   ```bash
   # Just open in VS Code/Cursor - uses devcontainer.json
   ```

2. **Specific Distribution**:
   ```bash
   # Copy desired config to main devcontainer.json
   cp .devcontainer/devcontainer.foxy.json .devcontainer/devcontainer.json
   # Then reopen in container
   ```

### Docker Script Method

```bash
# ROS2 Foxy (Original R2D3)
./.devcontainer/run.foxy.sh

# ROS2 Humble (Recommended)
./.devcontainer/run.humble.sh

# ROS2 Jazzy (Latest)
./.devcontainer/run.jazzy.sh
```

## Distribution-Specific Notes

### Foxy (Original)
- **Pros**: Native R2D3 support, original development target
- **Cons**: Older packages, limited long-term support
- **Best for**: Reproducing original R2D3 behavior, legacy systems

### Humble (Recommended)
- **Pros**: LTS support until 2027, stable, well-supported
- **Cons**: Some newer features not available
- **Best for**: Production deployments, stable development

### Jazzy (Latest)
- **Pros**: Latest features, newest packages, modern tooling
- **Cons**: Newer, potentially less stable
- **Best for**: Cutting-edge development, latest ROS2 features

## Package Compatibility

| Package | Foxy | Humble | Jazzy | Notes |
|---------|------|--------|-------|-------|
| MoveIt2 | ✅ | ✅ | ✅ | All versions supported |
| RealSense | ✅ | ✅ | ✅ | May need source build on some distros |
| Gazebo | Classic | Garden | Harmonic | Different versions per distro |
| Navigation2 | ✅ | ✅ | ✅ | All versions supported |

## Switching Distributions

### From DevContainer
1. Stop current container
2. Copy desired config: `cp .devcontainer/devcontainer.{distro}.json .devcontainer/devcontainer.json`
3. Rebuild container: "Dev Containers: Rebuild Container"

### From Docker Scripts
1. Stop current container: `docker stop ros2-*-r2d3-container`
2. Run new distribution script: `./.devcontainer/run.{distro}.sh`

## Troubleshooting

### Package Not Found
```bash
# Check if package exists for your distro
apt search ros-${ROS_DISTRO}-package-name

# If not available, the setup script will attempt source build
```

### Build Failures
```bash
# Clean workspace
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log

# Rebuild
cd ~/ros2_ws && colcon build --symlink-install
```

### Environment Issues
```bash
# Check ROS distro
echo $ROS_DISTRO

# Re-source environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Migration Guide

### From Foxy to Humble
- Most packages compatible
- Some API changes in MoveIt2
- Update launch files if needed

### From Humble to Jazzy  
- Check for deprecated APIs
- Update package.xml format if needed
- Verify thoroughly due to newer packages

## Support

For distribution-specific issues:
1. Check ROS2 documentation for your distro
2. Verify package availability: `apt search ros-${ROS_DISTRO}-`
3. Check R2D3 repository issues for known compatibility problems
