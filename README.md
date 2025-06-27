# Isaac Sim + ROS 2 Humble Container

An Apptainer container that combines NVIDIA Isaac Sim with ROS 2 Humble Desktop, enabling robotics simulation with seamless ROS 2 integration.

## What's Included

- **NVIDIA Isaac Sim** (version 4.5.0 by default)
- **ROS 2 Humble Desktop** with RViz, demos, and CLI tools
- **ROS 2 Bridge Extension** for Isaac Sim
- **Development Tools** including colcon, pip, and build essentials
- **Python Environment** with common robotics packages

## Quick Start

### Building the Container

```bash
# Build with default Isaac Sim 4.5.0
apptainer build /containers/isaac_ros2_humble.sif IsaacROS2.def

# Build with a specific Isaac Sim version
apptainer build --build-arg ISAAC_VERSION=4.6.0 /containers/isaac_ros2_humble.sif IsaacROS2.def
```

### Running Isaac Sim with GUI

```bash
apptainer exec --nv \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif
```

### Running Headless (for remote/server use)

```bash
apptainer exec --nv \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif \
  --headless
```

### Using with Project Overlays

```bash
apptainer exec --nv \
  --overlay ~/projects/my-robot-project/overlay.img \
  --bind ~/projects/my-robot-project:/home/dev/project \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif
```

## Key Features

### ROS 2 Integration
- Pre-configured ROS 2 Humble environment
- Isaac Sim ROS 2 bridge extension automatically loaded
- All ROS 2 desktop tools available (RViz, rqt, etc.)

### Performance Optimizations
- Cache binding for faster startup times
- Persistent asset storage support
- NVIDIA GPU acceleration with `--nv` flag

### Development Ready
- Python development tools and pip
- Colcon build system for ROS 2 packages
- Overlay support for custom packages

## Environment Variables

The container sets several important environment variables:

- `ROS_DISTRO=humble` - ROS 2 distribution
- `ISAAC_NUCLEUS_ROOT=/persistent/isaac/asset_root` - Asset storage location
- `ACCEPT_EULA=Y` - Accepts Isaac Sim EULA
- `PRIVACY_CONSENT=Y` - Accepts privacy terms

## Common Use Cases

### Robotics Simulation
Perfect for developing and testing ROS 2 robotics applications in Isaac Sim's physics-accurate environment.

### Multi-Robot Systems
Use with ROS 2's distributed architecture to simulate complex multi-robot scenarios.

### CI/CD Pipelines
Run headless simulations for automated testing of robotics software.

### Development Workflows
Combine with overlays to maintain separate environments for different projects.

## Prerequisites

### System Requirements
- **NVIDIA GPU** with compatible drivers
- **Apptainer/Singularity** installed
- Sufficient disk space for Isaac Sim assets (~10-15 GB)

### Required Host Directories
Create these directories on your host system **before** first running the container:

```bash
# Create container and cache directories
sudo mkdir -p /containers
sudo mkdir -p /var/cache/isaac/kit /var/cache/isaac/ov
sudo mkdir -p /persistent/isaac/asset_root

# Set proper permissions for cache directories (optional isaac group)
sudo chown root:isaac /var/cache/isaac/{kit,ov} 2>/dev/null || sudo chown -R $USER /var/cache/isaac
sudo chmod 2775 /var/cache/isaac/{kit,ov} 2>/dev/null || sudo chmod -R 755 /var/cache/isaac
```

**Note**: These directories must exist on the host system and are bind-mounted into the container at runtime. They are not created inside the container since they live outside the SIF file.

## Tips

1. **Host Directory Setup**: Always create the required host directories before first use
2. **Cache Directories**: Bind mount cache directories to avoid re-downloading assets each time
3. **Persistent Assets**: Use `/persistent/isaac/asset_root` for local Nucleus store and offline asset access
4. **Overlays**: Use overlays for project-specific ROS 2 packages
5. **Networking**: Isaac Sim can stream via WebRTC for remote access
6. **Performance**: Ensure your NVIDIA drivers support the Isaac Sim version you're using

## Version History

- **v0.2**: Isaac Sim 4.5.0 + ROS 2 Humble Desktop integration
