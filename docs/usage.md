# Usage Guide

This guide covers how to use the Isaac Sim container.

> **Note:** on the UVA CS server Apptainer is installed as a module. Load it with:
>
> ```bash
> source /etc/profile.d/modules.sh
> module load apptainer
> ```

---

## Launch Modes
This section lists different ways of launching Isaac Sim. These commands are designed to be exicuted from the terminal in any directory on the host system.

For an explanation of the launch modes see [Isaac Sim Modes](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#isaac-sim-launch-scripts).

> Note: 
> See **[Run Launch Script](/docs/script_docs/isaac_container.md)** for an automated script of these commands.

> Note: 
> These commands assume host setup has been completed by the admin as per the **[Host Setup Guide](/docs/host_setup.md)**.

### Notes

* Host-side directories (`/var/cache/isaac/...`, `/projects`, etc.) must exist.
* You must belong to the `isaac` group.
* The container includes environment variables for EULA and data consent.
* Isaac Sim can run in headless, GUI, or WebRTC modes with ROS 2 integration.

---

### Interactive Bash Session
This workflow allows you to interact with Isaac Sim similarly to the docs on thier website.

```bash
apptainer shell --nv --contain \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME/.local/share/ov/data:rw \
  --bind /projects:$HOME/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif
```

Once inside:

```bash
cd /isaac-sim
./isaac-sim.sh
```

> Note: For other launch scripts see [Isaac Sim Modes](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#isaac-sim-launch-scripts)
---

### Local GUI Mode (X11 Forwarding)

For local development with Isaac Sim's graphical interface:

```bash
apptainer exec --nv --contain \
  --bind /tmp/.X11-unix:/tmp/.X11-unix \
  --env DISPLAY=$DISPLAY \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME/.local/share/ov/data:rw \
  --bind /projects:$HOME/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "cd /isaac-sim && ./isaac-sim.sh"
```

---

### Headless Mode (Servers/Remote)

For headless operation on servers or remote machines:

```bash
apptainer exec --nv --contain \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME/.local/share/ov/data:rw \
  --bind /projects:$HOME/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "cd /isaac-sim && ./isaac-sim.sh --headless"
```

---

### Remote GUI Mode (WebRTC)

> Note:
> - For local use, launch the GUI instead.
> - Remote use requires access from UVA subnets to access cral.cs.virginia.edu.
>   -  i.e. eduroam | UVA Anywhere VPN | wired CS network
> 

**Launch Isaac Sim in headless mode:**

> The following commands can be used in the terminal

Isaac Sim headless with Isaac Sim WebRTC Streaming Client service:
```bash
apptainer exec --nv --contain \
  --env WEBRTC_ENABLE=1 \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME/.local/share/ov/data:rw \
  --bind /projects:$HOME/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "cd /isaac-sim && ./runheadless.sh"
```

Isaac Sim headless full app with Isaac Sim WebRTC Streaming Client service:
```bash
apptainer exec --nv --contain \
  --env WEBRTC_ENABLE=1 \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME/.local/share/ov/data:rw \
  --bind /projects:$HOME/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "cd /isaac-sim && ./isaac-sim.streaming.sh"
```

**For remote access via WebRTC:**
> Note: 
> More information is on the [Livestream Isaac Sim Docs](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/manual_livestream_clients.html)


1) Download **Isaac Sim WebRTC Streaming Client**

  - Download Isaac Sim WebRTC Streaming Client from the [Latest Release](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html#isaac-sim-latest-release) section for your platform.

2) Launch WebRTC app

3) Find the host IP address
  > **Note:**
  > 
  > In a terminal on the UVA CS Sever run:
  > ```bash
  >   host cral
  > ```

4) Connect

  In the WebRTC app:
  - Enter the IP address in 'Server'
  - Press Connect

---

## ROS 2 Integration

Test ROS 2 CLI:

```bash
apptainer exec --nv --contain \
  --bind /var/cache/isaac/kit:$HOME/.cache/kit:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "source /opt/ros/humble/setup.bash && ros2 topic list"
```

---

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
  --bind /var/cache/isaac/kit:$HOME/.cache/kit:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
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

---

## Project Integration

### Binding Project Directories

Bind your project directories for development:

```bash
apptainer exec --nv --contain \
  --bind /projects/my_robot_project:/workspace:rw \
  --bind /var/cache/isaac/kit:$HOME/.cache/kit:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
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

