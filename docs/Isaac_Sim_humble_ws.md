# Tutorials Project Setup for Isaac Sim + ROS 2

This one-time guide walks you through creating a dedicated `tutorials` project under `/projects`, complete with:

* An ext3 overlay image for project–specific package installs
* A cloned Isaac Sim ROS Workspace repository

Once configured, you’ll be able to build and launch the NVIDIA tutorials inside the Apptainer container.

---

## Prerequisites

* **Apptainer image**: `/containers/isaac_ros2_humble.sif`
* **Host directory**: `/projects` (bind-mounted to `$HOME/Documents` in-container)
* **Permissions**: You are in the `isaac` group and can write to `/projects`

---

## 1. Create the Project Directory

On the **host machine**, run:

```bash
mkdir -p /projects/tutorials
cd /projects/tutorials
```

Inside the container, this path appears as `~/Documents/tutorials`.

---

## 2. Create an ext3 Overlay Image

Still on the **host**, make a 1 GiB overlay file (adjust size as needed):

```bash
dd if=/dev/zero of=isaac_ros_overlay.img bs=1M count=1024
mkfs.ext3 isaac_ros_overlay.img
```

> **isaac\_ros\_overlay.img** is your writable layer for installing OS or ROS packages without modifying the base container.

---

## 3. Clone the Isaac Sim ROS Workspace

Clone directly into your project directory and checkout the tag matching your Isaac Sim version (e.g., `IsaacSim-4.5.0`):

```bash
cd /projects/tutorials
git clone https://github.com/NVIDIA-Omniverse/IsaacSim-ROS-Workspaces.git IsaacSim-ros_workspaces
cd IsaacSim-ros_workspaces
# Optional: switch to the matching release tag
git checkout IsaacSim-4.5.0
```

This creates `/projects/tutorials/IsaacSim-ros_workspaces`, containing `humble_ws` with all tutorial packages.

---

## 4. Build the Workspace Inside the Container

### Interactive Shell Workflow

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
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif
```

Inside the container, run:

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build --event-handlers console_direct+
source install/local_setup.bash
echo "Workspace built and sourced"
```

### One-Line Exec Workflow

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
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    cd ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --event-handlers console_direct+ && \
    source install/local_setup.bash && \
    echo 'Workspace built and sourced'\
  "
```

---

## 5. Launch Isaac Sim with the Tutorials

### Interactive Shell Workflow

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
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif
```

Inside the container, run:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
cd /isaac-sim
./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

### One-Line Exec Workflow

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
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    source ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash && \
    cd /isaac-sim && \
    ./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge\
  "
```

---
