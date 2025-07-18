#!/bin/bash
# -------------------------------------------- 
# Notes:

# This script sets up the environment for the Isaac Sim ROS2 bridge.
# This script enables the ROS2 bridge for Isaac Sim to run using the internal ROS2 packages.
# To use a system level ROS2 installation instead source the system ROS2 setup script instead of this one.

# -------------------------------------------- 
# Settings:

export isaac_sim_package_path=/isaac-sim
export isaac_sim_ros2_bridge_path=$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble

# -------------------------------------------- 
# Setup:


export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib

# -------------------------------------------- 
# Run Isaac Sim
$isaac_sim_package_path/isaac-sim.sh \
    --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge \
    --/rtx/ecoMode/enabled=True