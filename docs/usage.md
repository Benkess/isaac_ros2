# Usage Guide

This guide covers various usage scenarios for users.

## Launch Modes
> Notes:
> - All host-side dirs (/var/cache/isaac/..., /projects, etc.) should exist before you run these commands.
> - Your container already has EULA acceptance built-in via environment variables.
> - The same container can run headless, GUI, or WebRTC sessions with ROS 2 integration.

### Interactive Bash in the Container

```bash
apptainer shell --nv --contain \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /var/cache/isaac/pip:/root/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:/root/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:/root/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:/root/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:/root/.local/share/ov/data:rw \
  --bind /projects:/root/Documents:rw \
  /containers/isaac_ros2_humble.sif
```
- --nv enables NVIDIA GPU support.
- --cleanenv avoids inheriting unwanted host variables.
- Each --bind maps a host cache or data dir into the container.
- Once inside, you’re at / and can run ./isaac-sim.sh or any CLI.

```bash
cd /isaac-sim
./isaac-sim.sh
```

### Headless Mode (Servers/Remote)

For headless operation on servers or remote machines:

```bash
apptainer exec --nv --contain \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /var/cache/isaac/pip:/root/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:/root/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:/root/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:/root/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:/root/.local/share/ov/data:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc " \
    source /opt/ros/humble/setup.bash && \
    cd /isaac-sim && \
    ./isaac-sim.sh --headless \
  "
```

### Local GUI Mode (X11)

For local development with Isaac Sim's graphical interface:

```bash
apptainer exec --nv --contain \
  --bind /tmp/.X11-unix:/tmp/.X11-unix \
  --env DISPLAY=$DISPLAY \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /var/cache/isaac/pip:/root/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:/root/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:/root/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:/root/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:/root/.local/share/ov/data:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc " \
    source /opt/ros/humble/setup.bash && \
    cd /isaac-sim && \
    ./isaac-sim.sh \
  "
```

### Remote GUI Mode (WebRTC)

For remote access via web browser:

```bash
apptainer exec --nv --contain \
  --env WEBRTC_ENABLE=1 \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /var/cache/isaac/pip:/root/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:/root/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:/root/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:/root/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:/root/.local/share/ov/data:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc " \
    source /opt/ros/humble/setup.bash && \
    cd /isaac-sim && \
    ./isaac-sim.sh --webrtc_server=0.0.0.0 --webrtc_port=9090 \
  "
```

Then access via browser at `http://localhost:9090`

### ROS2 Integration
Your container includes ROS 2 Humble. To verify ROS 2 is working:
```bash
apptainer exec --nv --contain \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc " \
    source /opt/ros/humble/setup.bash && \
    ros2 topic list \
  "
```

### Connecting to an External Nucleus Server (optional)
If you have your own Nucleus enterprise instance, add:
```bash
--bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
--env ISAAC_NUCLEUS_ROOT=/persistent/isaac/asset_root
```
## Working with Overlays

### Creating Overlays

Overlays allow you to install additional packages persistently:

```bash
# Create a 1GB overlay image in the projects directory
dd if=/dev/zero of=/projects/my-project.img bs=1M count=1024
mkfs.ext3 /projects/my-project.img
```

### Using Overlays

```bash
apptainer exec --nv --contain \
  --overlay /projects/my-project.img \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif
```

### Installing Packages in Overlays

```bash
# Inside container with overlay
apt-get update && apt-get install -y \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-navigation2

# Packages will persist in the overlay
```

## Project Integration

### Binding Project Directories

Bind your project directories for development:

```bash
apptainer exec --nv --contain \
  --bind /projects/my_robot_project:/workspace:rw \
  --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif \
  bash
```

### Development Workflow

1. **Enter development shell:**
   ```bash
   # Inside container
   source /opt/ros/humble/setup.bash
   cd /workspace
   ```

2. **Build your packages:**
   ```bash
   colcon build --symlink-install
   source install/local_setup.bash
   ```

3. **Run your nodes:**
   ```bash
   ros2 run my_package my_node
   ```

