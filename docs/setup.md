# Isaac Sim ROS 2 Workspace Setup Guide

This guide provides detailed instructions for setting up and using the Isaac Sim ROS 2 workspace with your Apptainer container.

## Prerequisites

Ensure you have completed the basic setup from the main README:
- Built the container: `/containers/isaac-ros2-humble.sif`
- Created required host directories: `/var/cache/isaac/`, `/persistent/isaac/asset_root/`

## 1. Clone the Isaac Sim ROS 2 Workspace

On your host machine, clone the official ROS 2 workspace:

```bash
# Create projects directory if it doesn't exist
sudo mkdir -p /projects
sudo chown $USER:$USER /projects

# Clone the Isaac Sim ROS 2 workspace
git clone https://github.com/NVIDIA-Omniverse/IsaacSim-ROS-Workspaces.git /projects/IsaacSim-ros_workspaces
```

Your directory structure will be:
```
/projects/
└── IsaacSim-ros_workspaces/
    └── humble_ws/
        ├── src/
        ├── build/
        ├── install/
        └── log/
```

## 2. Build the ROS 2 Workspace

Enter the container with the workspace mounted:

```bash
apptainer exec --nv \
  --bind /projects/IsaacSim-ros_workspaces/humble_ws:/root/humble_ws:rw \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac-ros2-humble.sif \
  bash
```

Inside the container:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Set Fast RTPS profiles for Isaac Sim
export FASTRTPS_DEFAULT_PROFILES_FILE=/exts/isaacsim.ros2.bridge/humble/config/default_profiles.xml

# Update rosdep and install dependencies
rosdep update
rosdep install --from-paths /root/humble_ws/src \
               --ignore-src --rosdistro humble -y

# Build the workspace
cd /root/humble_ws
colcon build --symlink-install

# Source the workspace
source install/local_setup.bash
```

## 3. Launch Isaac Sim with ROS 2 Bridge

### Option A: Headless Mode (for servers/remote)

```bash
apptainer exec --nv \
  --bind /projects/IsaacSim-ros_workspaces/humble_ws:/root/humble_ws:rw \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  --env FASTRTPS_DEFAULT_PROFILES_FILE=/exts/isaacsim.ros2.bridge/humble/config/default_profiles.xml \
  /containers/isaac-ros2-humble.sif \
  ./isaac-sim.sh \
    --headless \
    --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

### Option B: GUI Mode (for local development)

```bash
apptainer exec --nv \
  --bind /projects/IsaacSim-ros_workspaces/humble_ws:/root/humble_ws:rw \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  --bind /tmp/.X11-unix:/tmp/.X11-unix \
  --env DISPLAY=$DISPLAY \
  --env FASTRTPS_DEFAULT_PROFILES_FILE=/exts/isaacsim.ros2.bridge/humble/config/default_profiles.xml \
  /containers/isaac-ros2-humble.sif \
  ./isaac-sim.sh \
    --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

### Option C: WebRTC Mode (for remote access)

```bash
apptainer exec --nv \
  --bind /projects/IsaacSim-ros_workspaces/humble_ws:/root/humble_ws:rw \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  --env WEBRTC_ENABLE=1 \
  --env FASTRTPS_DEFAULT_PROFILES_FILE=/exts/isaacsim.ros2.bridge/humble/config/default_profiles.xml \
  -p 9090:9090 \
  /containers/isaac-ros2-humble.sif \
  ./isaac-sim.sh \
    --headless \
    --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

## 4. Verify ROS 2 Integration

In a separate terminal, check that the ROS 2 bridge is working:

```bash
# Option 1: Use the same container
apptainer exec --nv \
  --bind /projects/IsaacSim-ros_workspaces/humble_ws:/root/humble_ws:rw \
  /containers/isaac-ros2-humble.sif \
  bash -c "source /opt/ros/humble/setup.bash && source /root/humble_ws/install/local_setup.bash && ros2 topic list"

# Option 2: Use host ROS 2 (if installed)
source /opt/ros/humble/setup.bash
ros2 topic list

# Check for clock messages (confirms bridge is active)
ros2 topic echo /clock
```

You should see topics like:
```
/clock
/tf
/tf_static
/isaac_sim_info
```

## 5. Development Workflow

### Interactive Development

For interactive development, keep a shell session open in the container:

```bash
# Enter development shell
apptainer exec --nv \
  --bind /projects/IsaacSim-ros_workspaces/humble_ws:/root/humble_ws:rw \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac-ros2-humble.sif \
  bash

# Inside container - set up environment
source /opt/ros/humble/setup.bash
source /root/humble_ws/install/local_setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/exts/isaacsim.ros2.bridge/humble/config/default_profiles.xml

# Now you can run ROS 2 commands, build packages, etc.
cd /root/humble_ws
colcon build --packages-select your_package
```

### Adding Custom Packages

To add your own ROS 2 packages:

1. **Create packages in the mounted workspace:**
   ```bash
   cd /projects/IsaacSim-ros_workspaces/humble_ws/src
   ros2 pkg create --build-type ament_python my_isaac_package
   ```

2. **Build and source:**
   ```bash
   # Inside container
   cd /root/humble_ws
   colcon build --packages-select my_isaac_package
   source install/local_setup.bash
   ```

3. **Packages persist** on the host at `/projects/IsaacSim-ros_workspaces/humble_ws/src/`

## Troubleshooting

### Common Issues

1. **No ROS 2 topics visible:**
   - Ensure `FASTRTPS_DEFAULT_PROFILES_FILE` is set
   - Check that Isaac Sim launched with the bridge extension

2. **Permission errors:**
   - Verify host directory permissions
   - Check that bind mounts are writable (`:rw`)

3. **Build failures:**
   - Update rosdep: `rosdep update`
   - Check for missing dependencies: `rosdep install --from-paths src --ignore-src -y`

4. **Graphics issues:**
   - Ensure `--nv` flag is used for GPU access
   - For GUI mode, verify X11 forwarding is working

### Performance Tips

1. **Use symlink installs:** `colcon build --symlink-install`
2. **Parallel builds:** `colcon build --parallel-workers 4`
3. **Selective building:** `colcon build --packages-select package_name`
4. **Cache directories:** Always bind mount cache directories to avoid re-downloading

### Installing Additional ROS 2 Packages

The container includes ROS 2 Humble Base. To add GUI tools like RViz:

```bash
# Inside the container
apt-get update && apt-get install -y \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-demo-nodes-cpp \
  ros-humble-demo-nodes-py

# Or install via overlays for persistence
```

## Next Steps

- Explore the Isaac Sim ROS 2 examples in the workspace
- Create your own robot simulation packages
- Integrate with other ROS 2 tools like RViz, MoveIt, or Nav2
- Set up continuous integration with your custom packages
