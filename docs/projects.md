# Creating a New Project with an Overlay and Workspace

This guide shows you how to set up a self‑contained project under `/projects`, with its own ext3 overlay image and ROS 2 workspace, for use in the Isaac Sim + ROS 2 Apptainer container.

## 1. Create the Project Directory

```bash
# On the host machine:
mkdir -p /projects/<your-project>/ros_ws
cd    /projects/<your-project>
```

* Everything for `your-project` will live under `/projects/your-project`
* Inside the container this maps to `~/Documents/<your-project>`

## 2. Create an ext3 Overlay Image

```bash
# Still in /projects/<your-project>:
# 1 GiB overlay (adjust size as needed)
dd if=/dev/zero of=overlay.img bs=1M count=1024
mkfs.ext3 overlay.img
```

* `overlay.img` lets you install project‑specific OS/ROS packages without touching the base container.

## 3. Clone the ROS 2 Tutorial Workspace

```bash
cd ros_ws
git clone https://github.com/NVIDIA-Omniverse/IsaacSim-ROS-Workspaces.git humble_ws
```

* This creates the `humble_ws` folder containing the tutorial packages.

## 4. Build the Workspace Inside the Container

```bash
apptainer exec --nv \
  --overlay /projects/<your-project>/overlay.img \
  --bind    /projects/<your-project>/ros_ws:/home/dev/ros_ws:rw \
  /containers/isaac_ros2_humble.sif \
  bash -lc "
    source /opt/ros/humble/setup.bash
    cd /home/dev/ros_ws/humble_ws
    rosdep update
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build --event-handlers console_direct+
    source install/local_setup.bash
    echo '✅ Workspace built and sourced'
  "
```

* `--overlay` mounts your `overlay.img` as an ext3 layer.
* `--bind` maps your host `ros_ws` into the container.

## 5. Launch Isaac Sim with Your Project

```bash
apptainer exec --nv \
  --overlay /projects/<your-project>/overlay.img \
  --bind    /projects/<your-project>/ros_ws:/home/dev/ros_ws:rw \
  /containers/isaac_ros2_humble.sif \
  bash -lc "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros_ws/humble_ws/install/local_setup.bash
    cd /isaac-sim
    ./isaac-sim.sh --headless --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
  "
```

* Omit `--headless` if you need the GUI (use X11/VirtualGL forwarding).

---

Now you have a clean, reproducible project environment. Repeat these steps for each new research project, adjusting `<your-project>` as needed. Happy simulating!
