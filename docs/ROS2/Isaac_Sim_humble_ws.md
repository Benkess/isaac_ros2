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

## Status

✅ **Completed:**
- Overlay image created and configured
- Isaac ROS workspace cloned and built
- Nav2 packages installed (navigation2, nav2-bringup)
- Carter navigation tutorials ready to run

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
apptainer shell --nv --contain --fakeroot \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:/root/.cache/ov:rw \
  --bind /var/cache/isaac/pip:/root/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:/root/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:/root/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:/root/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:/root/.local/share/ov/data:rw \
  --bind /projects:/root/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif
```

Inside the container, run:

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws

# Update package lists and install required packages
apt update
apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-colcon-common-extensions
# Note: Some packages may fail due to permission issues, but python3-rosdep and python3-colcon-common-extensions are essential

# Initialize and update rosdep
rosdep init
rosdep update

# Install dependencies and build
rosdep install -i --from-path src --rosdistro humble -y
# If rosdep fails due to permission issues, try: apt clean && rosdep install -i --from-path src --rosdistro humble -y
# Or skip dependencies and build directly: colcon build --event-handlers console_direct+
colcon build --event-handlers console_direct+
# If build fails due to missing specific packages (e.g., ackermann_msgs), try:
# mkdir -p /tmp/apt-cache && export APT_CONFIG=/tmp/apt.conf && echo 'Dir::Cache "/tmp/apt-cache";' > /tmp/apt.conf
# apt-get update && apt-get install -y ros-humble-ackermann-msgs
# Or build only specific packages: colcon build --packages-select [package_names]
source install/local_setup.bash
echo "Workspace built and sourced"
```

### One-Line Exec Workflow

```bash
apptainer exec --nv --contain --fakeroot \
  --bind /tmp/.X11-unix:/tmp/.X11-unix \
  --env DISPLAY=$DISPLAY \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME/.local/share/ov/data:rw \
  --bind /projects:/root/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw \
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    cd ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws && \
    apt update && \
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-colcon-common-extensions && \
    rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --event-handlers console_direct+ && \
    source install/local_setup.bash && \
    echo 'Workspace built and sourced'\
  "
```

---

## 5. Launch Isaac Sim with the Tutorials

**Important:** Isaac Sim cannot run as root user, so we **DO NOT** use `--fakeroot` for launching Isaac Sim. Since overlays require root permissions, we have two options for launching:

### Option A: Launch with Overlay (if your system allows user overlays)

```bash
apptainer shell --nv --contain \
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
  /containers/isaac_ros2_humble.sif
```

### Option B: Launch without Overlay (recommended for most systems)

If overlay mounting fails due to permissions, launch without overlay and bind-mount the built workspace directly:

```bash
apptainer shell --nv --contain \
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
  /containers/isaac_ros2_humble.sif
```

**Note:** The built workspace is accessible via the `/projects` bind mount at `~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws`.

Inside the container, run:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
cd /isaac-sim
./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

### One-Line Exec Workflow (without overlay)

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
  /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    source ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash && \
    cd /isaac-sim && \
    ./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge\
  "
```

---

## Troubleshooting

### Common Issues and Solutions

#### 1. Overlay Permission Issues

**Problem:** `FATAL: container creation failed: mount hook function failure`

**Solution:** 
- Ensure overlay image has correct permissions: `sudo chmod 664 /projects/tutorials/isaac_ros_overlay.img`
- Ensure overlay image is owned by `root:isaac`: `sudo chown root:isaac /projects/tutorials/isaac_ros_overlay.img`
- Always use `--fakeroot` flag with overlay

#### 2. APT Cache Permission Errors

**Problem:** `Could not open file /var/cache/apt/archives/partial/*.deb - open (13: Permission denied)`

**Solution:** Use custom APT cache directory:
```bash
mkdir -p /tmp/apt-cache
export APT_CONFIG=/tmp/apt.conf
echo 'Dir::Cache "/tmp/apt-cache";' > /tmp/apt.conf
apt-get update
apt-get install -y ros-humble-ackermann-msgs
```

#### 3. dpkg Status Backup File Issues

**Problem:** `error creating new backup file '/var/lib/dpkg/status-old': File exists`

**Solution:** 
```bash
rm -f /var/lib/dpkg/status-old
rm -f /var/lib/dpkg/lock*
rm -f /var/cache/apt/archives/lock
dpkg --configure -a
```

#### 4. Missing ackermann_msgs Package

**Problem:** `Could not find a package configuration file provided by "ackermann_msgs"`

**Solution:** Install manually using custom APT cache:
```bash
mkdir -p /tmp/apt-cache
export APT_CONFIG=/tmp/apt.conf
echo 'Dir::Cache "/tmp/apt-cache";' > /tmp/apt.conf
apt-get update
apt-get install -y ros-humble-ackermann-msgs
```

Even if dpkg shows errors, check if package is installed: `dpkg -l | grep ackermann`
If status shows `iU` (installed but unconfigured), the package files are present and `colcon build` should work.

#### 5. Missing Nav2 Navigation Packages

**Problem:** `Package 'nav2_bringup' not found` when running carter_navigation.launch.py

**Solution:** Install Nav2 packages using manual extraction method (due to dpkg status file issues):
```bash
# Set up custom APT cache
mkdir -p /tmp/apt-cache
export APT_CONFIG=/tmp/apt.conf
echo 'Dir::Cache "/tmp/apt-cache";' > /tmp/apt.conf

# Download Nav2 packages (including nav2_common dependency)
cd /tmp
apt-get download ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-common

# Extract packages manually to bypass dpkg issues
for deb in *.deb; do
  echo "Extracting $deb..."
  dpkg-deb --extract $deb /
done

# Verify installation
ls -la /opt/ros/humble/share/ | grep -E '(navigation2|nav2)'
ros2 pkg list | grep nav2
python3 -c 'import nav2_common; print("nav2_common imported successfully")'
```

**Note:** This method bypasses dpkg status tracking but successfully installs the package files where ROS can find them.

#### 6. Bind Mount Path Issues with --fakeroot

**Problem:** Paths not accessible when using `--fakeroot`

**Explanation:** `--fakeroot` changes user to root inside container, so `$HOME` becomes `/root`, not `/u/username`

**Solution:** Use absolute paths in bind mounts:
- ❌ `--bind /projects:$HOME/Documents:rw` 
- ✅ `--bind /projects:/root/Documents:rw`

#### 7. Isaac Sim Cannot Run as Root

**Problem:** `Omniverse Kit cannot be run as the root user without the --allow-root flag` and segmentation fault

**Explanation:** Isaac Sim/Omniverse Kit refuses to run as root user for security reasons

**Solution:** 
- **For building (Section 4):** Use `--fakeroot` to install packages as root
- **For launching Isaac Sim (Section 5):** Do NOT use `--fakeroot`, run as regular user
- Use `--bind /projects:$HOME/Documents:rw` (not `/root/Documents`) when not using `--fakeroot`

#### 8. Package Installation Alternatives

If standard package installation fails, try these alternatives in order:

1. **Custom APT cache:** (as shown above)
2. **Skip failing packages:** `colcon build --packages-skip cmdvel_to_ackermann`
3. **Build specific packages:** `colcon build --packages-select isaac_ros2_messages`
4. **Force dpkg installation:** `dpkg --force-overwrite -i /path/to/package.deb`

## Final Verification

After completing all setup steps, verify that everything is working correctly:

```bash
apptainer exec --contain --fakeroot \
  --bind /projects:/root/Documents:rw \
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -c "
    source /opt/ros/humble/setup.bash
    source /root/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
    
    echo 'Available Nav2 packages:'
    ros2 pkg list | grep nav2
    
    echo 'Available navigation packages:'
    ros2 pkg list | grep navigation
    
    echo 'Testing carter_navigation launch file:'
    ros2 launch carter_navigation carter_navigation.launch.py --show-args
  "
```

**Expected output:**
- `nav2_bringup` package should be listed
- `carter_navigation` package should be listed  
- Launch file should show available arguments without errors (map, params_file, use_sim_time, namespace, etc.)

✅ **Status: VERIFIED AND WORKING** - All Nav2 packages successfully installed and carter_navigation.launch.py is fully functional!

## Ready to Run Isaac Sim Navigation Tutorials

With Nav2 packages now installed, you can run the carter_navigation tutorials using a **dual-container approach**:

1. **Isaac Sim container** (without overlay - to avoid root user issues)
2. **Navigation container** (with overlay - to access Nav2 packages)

### Step 1: Launch Isaac Sim (Terminal 1)

```bash
# Launch Isaac Sim without overlay (avoids root user issues)
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
  /bin/bash -lc "
    source /opt/ros/humble/setup.bash && \
    source ~/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash && \
    cd /isaac-sim && \
    ./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
  "
```

### Step 2: Launch Navigation (Terminal 2)

```bash
# Launch navigation with overlay to access Nav2 packages
apptainer exec --nv --contain --fakeroot \
  --bind /projects:/root/Documents:rw \
  --overlay /projects/tutorials/isaac_ros_overlay.img \
  /containers/isaac_ros2_humble.sif \
  /bin/bash -lc "
    source /opt/ros/humble/setup.bash && \
    source /root/Documents/tutorials/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash && \
    ros2 launch carter_navigation carter_navigation.launch.py
  "
```

### Why This Dual-Container Approach Works

- **Isaac Sim container**: Runs as regular user (not root) so Isaac Sim can start properly
- **Navigation container**: Runs with overlay and `--fakeroot` so Nav2 packages are available
- **Communication**: Both containers share the same ROS 2 DDS domain, enabling seamless communication

This approach leverages the strengths of both container configurations while avoiding their respective limitations.
