#!/bin/bash
# 定义工作空间路径（根据实际情况确认，若你的dev_ws在用户根目录，此路径无需修改）
WORKSPACE_PATH="$HOME/dev_ws"

# 新增：确保Gazebo相关进程已关闭
echo "正在关闭可能残留的Gazebo进程..."
killall gzserver gzclient 2>/dev/null || true
sleep 1  # 等待进程关闭

# 切换到工作空间并加载ROS环境（关键步骤，确保能找到功能包和节点）
cd "$WORKSPACE_PATH"
source install/setup.bash

# 1. 启动仿真（机械臂+底盘+传感器）
gnome-terminal --title="仿真环境" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 launch mybot gazebo_world.launch.py; exec bash'"
# 等待2秒，确保仿真环境先初始化（避免后续模块依赖报错）
sleep 2
# 2. 启动moveit（机械臂规划、控制）
gnome-terminal --title="MoveIt机械臂控制" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 launch mybot my_moveit_rviz.launch.py; exec bash'"
# 3. 启动nav2（导航）
gnome-terminal --title="Nav2导航" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 launch bot_navigation nav_bringup_gazebo.launch.py; exec bash'"
# 4. 启动llama-server服务端
gnome-terminal --title="Llama服务端" --command="bash -c 'llama-server -m $HOME/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf -c 2048 --threads 8 --port 8081; exec bash'"
# 等待3秒，确保服务端启动完成（避免语言解析模块连接失败）
sleep 3
# 5. 启动语言解析模块
gnome-terminal --title="语言解析模块" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 run llama_command_parser command_parser; exec bash'"
# 6. 启动导航模块
gnome-terminal --title="导航执行模块" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 run nav_simple simple_navigator; exec bash'"
# 7. 启动机械臂模块
gnome-terminal --title="机械臂抓取放置" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 run arm_action_integration arm_grab_place_node; exec bash'"
# 8. 启动视觉模块
gnome-terminal --title="视觉颜色检测" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 run package_detector color_detector_node; exec bash'"
# 9. 启动主控模块
gnome-terminal --title="主控模块" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 run main_controller main_controller_node; exec bash'"
# 10. 运行rosbridge
gnome-terminal --title="ROSBridge" --command="bash -c 'source $WORKSPACE_PATH/install/setup.bash; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash'"
# 提示脚本执行完成
echo "所有模块已启动，共打开10个终端窗口，请查看各窗口是否正常运行！"

