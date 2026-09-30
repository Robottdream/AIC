#!/bin/bash
# Source after ROS/install setup when issuing manual ROS commands in WSL.
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
# Configure loopback in Cyclone XML rather than ROS_LOCALHOST_ONLY: Humble's
# RMW adds its own interface settings in that mode, conflicting with custom
# settings. The XML also gives this multi-process project enough peer ports.
# Explicit AIC_ROS_LOCALHOST_ONLY=0 disables the loopback profile for LAN use.
export ROS_LOCALHOST_ONLY="${AIC_ROS_LOCALHOST_ONLY:-0}"
if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ] &&
   [ "${AIC_ROS_LOCALHOST_ONLY:-1}" = "1" ]; then
  export ROS_LOCALHOST_ONLY=0
  if [ -z "${CYCLONEDDS_URI:-}" ]; then
    export CYCLONEDDS_URI="file://$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/cyclonedds_local.xml"
  fi
fi
# Fast DDS remains an explicit comparison option, with the old XML opt-in.
if [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ] &&
   [ "${AIC_FASTDDS_LOCAL_PROFILE:-0}" = "1" ] &&
   [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_local.xml"
fi
# Gazebo odometry stayed aligned with world coordinates in the six-task run.
# Use its time-independent map alignment by default; 0 restores AMCL TF.
export AIC_ODOM_MAP="${AIC_ODOM_MAP:-1}"
