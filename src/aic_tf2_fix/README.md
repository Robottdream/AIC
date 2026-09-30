# Humble asynchronous TF lock-order repair

This package provides a project-scoped replacement of `tf2_ros::Buffer::waitForTransform`
for the installed **tf2_ros 0.25.23** ABI. It does not modify `/opt/ros` or rewrite TF
timestamps. `nav_bringup_gazebo.launch.py` applies its shared library through
`LD_PRELOAD` only within the navigation process group; the launch also checks the
runtime dependency version. A ROS upgrade requires review and rebuilding this fix.

The original waiter holds `timer_to_request_map_mutex_` while calling
`addTransformableRequest`. Meanwhile BufferCore delivers a transform callback
under `transformable_requests_mutex_`, and that callback locks the timer map.
The two threads can deadlock. A GDB capture of the live controller confirmed both
stacks and the crossed mutex owners.

The repaired implementation registers the core request without holding the timer
map lock. A per-request registration gate records early completion instead of
waiting for registration; completion is delivered after the timer entry exists.
Immediate success, unavailable transforms and timeouts keep their existing roles.
The class layout and other TF methods are unchanged.

Build in WSL:

```bash
cd /home/polarbear/ws_aic
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src --packages-select aic_tf2_fix bot_navigation
source install/setup.bash
python3 /mnt/e/workspace/AIC/tools/check_tf_async_fix.py
```

The C++ probe runs 20,000 concurrent asynchronous requests while transforms arrive
on another thread, then checks timeout and immediate success. The Python runner
uses the same executable against the installed library and the repair. Baseline
timeout is expected; repaired runs must finish successfully. See NOTICE for the
upstream BSD attribution.
