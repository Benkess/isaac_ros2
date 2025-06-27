# Isaac Sim + ROS 2 Humble Container

An Apptainer container combining NVIDIA Isaac Sim with ROS 2 Humble for robotics simulation.

## Quick Start
See [Usage Examples](docs/usage.md) for launch modes.

## What's Included
- NVIDIA Isaac Sim (4.5.0 by default)
- ROS 2 Humble Base (CLI tools, bags, tf2)
- Isaac Sim ROS 2 Bridge Extension
- Development tools (colcon, pip)

## Documentation
- **[Complete Setup Guide](docs/host_setup.md)** - ROS 2 workspace integration, development workflow
- **[Usage Examples](docs/usage.md)** - GUI mode, WebRTC, overlays, troubleshooting
- **[Humble WS](docs/Isaac_Sim_humble_ws.md)** - Isaac Sim ROS2 Tutorials

## Requirements
- NVIDIA GPU with compatible drivers
- Apptainer installed
> **Note:** on the UVA CS server Apptainer is installed as a module. Use: 
> ```bash
> source /etc/profile.d/modules.sh
> module load aptainer
> ```

## Tips
1. **Host Directory Setup**: Always create the required host directories before first use
2. **Cache Directories**: Bind mount cache directories to avoid re-downloading assets each time
3. **Persistent Assets**: Use `/persistent/isaac/asset_root` for local Nucleus store and offline asset access
4. **Overlays**: Use overlays for project-specific ROS 2 packages
5. **Networking**: Isaac Sim can stream via WebRTC for remote access
6. **Performance**: Ensure your NVIDIA drivers support the Isaac Sim version you're using