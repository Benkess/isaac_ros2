# Isaac Sim ROS2 Setup

This document explains how to use Isaac Sim with ROS2. For more details, see the [Isaac Sim Docs](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_ros.html#getting-started).

## Running ROS Without a System-Level Install

> **Note:**
> - This script sets up the environment for the Isaac Sim ROS2 bridge.
> - It enables the ROS2 bridge for Isaac Sim to run using the internal ROS2 packages.
> - To use a system-level ROS2 installation, source the system ROS2 setup script instead of this one.

Inside the Isaac Sim container, set these environment variables:

```bash
export isaac_sim_package_path=/isaac-sim
export isaac_sim_ros2_bridge_path=$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again, potentially leading to conflicts.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
```

## Enabling the ROS Bridge Extension

### Create FastDDS Profile
Create a file named `fastdds.xml` under `~/.ros/` and paste the following snippet into the file:

```xml
<?xml version="1.0" encoding="UTF-8" ?>

<license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.</license>


<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

### Environment Setup
In each terminal running ROS2 or Isaac Sim:

1. Run:
   ```bash
   export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
   ```
   in the terminals that will use ROS 2 functions.
2. (Optional) Run:
   ```bash
   export ROS_DOMAIN_ID=<id_number>
   ```
   before launching Isaac Sim. You can later decide whether to use this `ROS_DOMAIN_ID` inside your environment, or explicitly use a different ID for any given topic.
3. Source your ROS 2 installation and workspace before launching Isaac Sim.

## Run Isaac Sim with ROS2 Bridge

```bash
$isaac_sim_package_path/isaac-sim.sh \
    --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge \
    --/rtx/ecoMode/enabled=True
```

# ROS2 Projects / Workspaces

refer to [projects](/docs/projects.md)

## Recomended Project Layout:

For Isaac Sim and ROS2 projects it is recommended to have the following files and directories located in /projects. Note that these are optional and may not all be needed for your project.

- /projects
    - /ros2_ws
    - /isaac-sim/documents
    - ros2_overlay.img
    - isaac_overlay.img

## Make a ros2 overlay: 
```bash
apptainer overlay create --sparse --fakeroot --size 1024 ros2_overlay.img
```

## launch isaac and ros2

**Lanch ROS2 Container**
```bash
apptainer exec --nv --no-mount /l \
  --overlay /projects/<your-project>/ros2_overlay.img \
  --bind    /projects/<your-project>/ros2_ws:/ros2_ws:rw \
  /containers/ros2_humble.sif \
```
> Note: use '--fakeroot' to modify the overlay (apt or rosdep)

Once inside:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export ROS_DOMAIN_ID=<id_number>
source /opt/ros/humble/setup.bash
source <path_ros2_ws>/install/setup.bash
```

Optionally:
```bash
cd /root/humble_ws
apt-get update
rosdep install --from-paths src --ignore-src --rosdistro=humble -y
source /opt/ros/humble/setup.sh
colcon build
source install/local_setup.bash
```

**Launch Isaac Sim Container**
