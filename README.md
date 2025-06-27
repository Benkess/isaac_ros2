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
apptainer build isaac_ros2_humble.sif IsaacROS2.def

# Build with a specific Isaac Sim version
apptainer build --build-arg ISAAC_VERSION=4.6.0 isaac_ros2_humble.sif IsaacROS2.def
```

### Running Isaac Sim with GUI

```bash
apptainer exec --nv \
  --bind ~/cache/kit:/isaac-sim/kit/cache:rw \
  --bind ~/cache/ov:/root/.cache/ov:rw \
  isaac_ros2_humble.sif
```

### Running Headless (for remote/server use)

```bash
apptainer exec --nv \
  --bind ~/cache/kit:/isaac-sim/kit/cache:rw \
  --bind ~/cache/ov:/root/.cache/ov:rw \
  isaac_ros2_humble.sif \
  --headless
```

### Using with Project Overlays

```bash
apptainer exec --nv \
  --overlay ~/projects/my-robot-project/overlay.img \
  --bind ~/projects/my-robot-project:/home/dev/project \
  --bind ~/cache/kit:/isaac-sim/kit/cache:rw \
  --bind ~/cache/ov:/root/.cache/ov:rw \
  isaac_ros2_humble.sif
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

## Requirements

- **NVIDIA GPU** with compatible drivers
- **Apptainer/Singularity** installed
- Sufficient disk space for Isaac Sim assets (~10-15 GB)

## Tips

1. **Cache Directories**: Always bind mount cache directories to avoid re-downloading assets
2. **Overlays**: Use overlays for project-specific ROS 2 packages
3. **Networking**: Isaac Sim can stream via WebRTC for remote access
4. **Performance**: Ensure your NVIDIA drivers support the Isaac Sim version you're using

## Version History

- **v0.2**: Isaac Sim 4.5.0 + ROS 2 Humble Desktop integration
