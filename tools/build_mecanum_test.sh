#!/usr/bin/env bash
set -euo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target="$HOME/ws_aic_mecanum_20261002"
evidence="$HOME/aic_round_omni_evidence_20261003"
mkdir -p "$evidence"
# src was copied from the runtime before edits. Only send maintenance files
# touched by this request; preserve the original runtime's uncommitted work.
for relative in \
  src/main_controller/main_controller/main_controller_node.py \
  src/arm_action_integration/arm_action_integration/arm_grab_place_node.py \
  src/arm_action_integration/package.xml \
  src/yzbot/mybot_description/setup.py \
  src/yzbot/mybot_description/urdf/lidar_gazebo.xacro \
  src/yzbot/mybot_description/urdf/mecanum_base.xacro \
  src/yzbot/mybot_description/urdf/mecanum_visuals.xacro \
  src/yzbot/mybot_description/urdf/mecanum_with_arm_gazebo.xacro \
  src/yzbot/mybot_description/urdf/originbot_mecanum_gazebo.xacro \
  src/yzbot/mybot/launch/gazebo_world.launch.py \
  src/yzbot/mybot/package.xml \
  src/yzbot/mybot/launch/my_moveit_rviz.launch.py \
  src/yzbot/mybot/config/ros2_controllers_mecanum.yaml \
  src/yzbot/mybot/config/moveit_controllers_mecanum.yaml \
  src/yzbot/mybot/config/six_arm_mecanum.srdf \
  src/yzbot/bot_navigation/CMakeLists.txt \
  src/yzbot/bot_navigation/package.xml \
  src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py \
  src/yzbot/bot_navigation/scripts/scan_velocity_guard.py \
  src/yzbot/bot_navigation/scripts/dynamic_obstacle_tracker.py \
  src/yzbot/bot_navigation/scripts/mecanum_io.py \
  src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml \
  src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml \
  src/yzbot/bot_navigation/param/collision_monitor_mecanum.yaml \
  ros_round_omni_start.sh \
  ros_mecanum_start.sh; do
  cp "$repo/$relative" "$target/$relative"
done
mkdir -p "$target/src/yzbot/mybot_description/meshes/mecanum"
rsync -a "$repo/src/yzbot/mybot_description/meshes/mecanum/" "$target/src/yzbot/mybot_description/meshes/mecanum/"
cd "$target"
# Gazebo's CMake dependency search otherwise walks every Windows PATH entry
# via DrvFs; only Linux build tools are needed here.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src --parallel-workers 4 > "$evidence/build.log" 2>&1
source install/setup.bash
xacro src/yzbot/mybot_description/urdf/originbot_mecanum_gazebo.xacro > "$evidence/round-omni.urdf"
gz sdf -p "$evidence/round-omni.urdf" > "$evidence/round-omni.sdf"
echo BUILD_AND_URDF_OK
