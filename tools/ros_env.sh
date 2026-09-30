#!/bin/bash
# Source after ROS/install setup when issuing manual ROS commands in WSL.
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi
# Gazebo odometry stayed aligned with world coordinates in the six-task run.
# Use its time-independent map alignment by default; 0 restores AMCL TF.
export AIC_ODOM_MAP="${AIC_ODOM_MAP:-1}"
