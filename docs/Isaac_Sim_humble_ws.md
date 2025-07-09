# One-Time Setup: Tutorials Project for Isaac Sim + ROS 2

This guide covers creating a dedicated `tutorials` project under `/projects`, complete with an ext3 overlay image and a cloned Isaac Sim ROS Workspace repository for running the NVIDIA tutorials.

---

## Prerequisites

* Apptainer container image: `/containers/isaac_ros2_humble.sif`
* Host directory `/projects` is bind-mounted to `$HOME/Documents` inside the container
* User is in the `isaac` group and has write permissions on `/projects`

---

## 1. Create the Project Directory

```bash
# On the host machine:
mkdir -p /projects/tutorials
cd    /projects/tutorials
```

* This will map inside the container to `~/Documents/tutorials`.

---

## 2. Create an ext3 Overlay Image

```bash
# Still on the host:
# Create a 1 GiB overlay file (adjust size if needed)
dd if=/dev/zero of=isaac_ros_overlay.img bs=1M count=1024
mkfs.ext3 isaac_ros_overlay.img
```

* `isaac_ros_overlay.img` lets you install project-specific OS/ROS packages without modifying the base container.

---

## 3. Clone the Isaac Sim ROS Workspace Repository

```bash
# Clone directly into your project directory:
cd /projects/tutorials
git clone https://github.com/NVIDIA-Omniverse/IsaacSim-ROS-Workspaces.git IsaacSim-ros_workspaces
# Move into the cloned repo
cd IsaacSim-ros_workspaces
# (Optional) Checkout the branch or tag matching your Isaac Sim version
# For Isaac Sim 4.5.0:
git checkout IsaacSim-4.5.0
```

* This creates `/projects/tutorials/IsaacSim-ros_workspaces`, containing `humble_ws` with all tutorial packages tuned for your Isaac Sim version.

---

## 4. Build the Workspace Inside the Container

Run this single command from your host shell:

```bash
apptainer exec --nv \
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  --bind    /projects/tutorials/IsaacSim-ros_workspaces:/home/dev/ros_ws:rw \
  /containers/isaac_ros2_humble.sif \
  bash -lc "
    set -e
    # 1. Source ROS 2 Humble
    source /opt/ros/humble/setup.bash

    # 2. Navigate to the tutorial workspace
    cd /home/dev/ros_ws/humble_ws

    # 3. Install dependencies with rosdep
    rosdep update
    rosdep install -i --from-path src --rosdistro humble -y

    # 4. Build with colcon
    colcon build --event-handlers console_direct+

    # 5. Source the newly built workspace
    source install/local_setup.bash

    echo '✅ Tutorial workspace built and sourced'
  "
```

* `--overlay` mounts your `isaac_ros_overlay.img`; `--bind` maps the cloned repo.

---

## 5. Launch Isaac Sim with the Tutorials

Once built, start Isaac Sim so it loads the ROS 2 bridge and your tutorial packages:

```bash
apptainer exec --nv \
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  --bind    /projects/tutorials/IsaacSim-ros_workspaces:/home/dev/ros_ws:rw \
  /containers/isaac_ros2_humble.sif \
  bash -lc "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros_ws/humble_ws/install/local_setup.bash
    cd /isaac-sim
    ./isaac-sim.sh --headless \
      --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
  "
```

* Omit `--headless` for GUI mode (use X11/VirtualGL forwarding).

---

## Notes

* **Overlay size**: Increase `count=1024` if you add many packages.
* **Permissions**: If you hit `EACCES`, ensure `/projects/tutorials` is group-writable by `isaac`.

---

You’re all set! Repeat these steps for any new research project by replacing `tutorials` with your project name. Happy simulating!
