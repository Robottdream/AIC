#!/usr/bin/env bash
# ros_competition_start.sh —— AIC 月球仓储机器人「一键启动 / 停止 / 状态 / 体检」（WSL 适配版）
#
# 用法（在 WSL 里，工作空间根目录执行）：
#   ./ros_competition_start.sh [start] [--no-rviz] [--headless] [--no-watchdog]
#                                     后台拉起全部模块 + 逐项等就绪 + 导航链路体检 + 看门狗
#   ./ros_competition_start.sh doctor  导航链路体检；不合格自动自愈（重启 Nav2 + 用真值重设 AMCL 初始位姿）
#   ./ros_competition_start.sh watchdog  前台跑看门狗（start 已自动在后台拉起一个）
#   ./ros_competition_start.sh status  查看各模块进程、就绪情况、最近一次体检结论与残留进程
#   ./ros_competition_start.sh stop    整组回收全部模块（SIGINT → SIGTERM → SIGKILL）
#   ./ros_competition_start.sh restart [--no-rviz]   先 stop 再 start
#   ./ros_competition_start.sh logs [模块名]         打印各模块日志尾部；给模块名则 tail -f
#
# 与原版的区别：
#   * 不再依赖 gnome-terminal / tmux：每个模块用 setsid 放进独立进程组，后台运行，日志落盘
#   * 工作空间按「脚本所在目录」自动推断，不再写死 $HOME/dev_ws
#   * 逐项就绪门控：/scan 有发布者 → move_group → planner_server/bt_navigator active
#     → llama-server /health → 业务节点 → /ATTACHLINK → rosbridge :9090
#   * Nav2 独立进程/容器异常会自动整组重启；重启后一定重设 AMCL 初始位姿（否则 AMCL 回到 0,0,0，机器人会被导到别处）
#   * 导航链路体检（tools/preflight_check.py）：抓“planner 起点与定位不一致 / 定位跑飞 / map->odom 断流”
#     —— 这三种状态都会让小车在目标附近空转很久不下爪，见 AGENTS.md 第 8 节
#   * stop 按进程组回收，避免 rviz2 / move_group / gzserver 变成孤儿
#   * 脚本自身若是 CRLF 行尾会自动修好再重新执行
#
# 原版已备份为同目录 ros_competition_start.sh.before-wsl

set -u

WS="$(cd "$(dirname "$0")" && pwd)"
# An explicit local selection keeps the familiar entry point on the active
# test workspace while preserving this workspace's original sources/configs.
if [ "${AIC_ROBOT_MODEL:-}" != legacy ] && [ -f "$WS/.aic_active_workspace" ]; then
  ACTIVE_WS="$(head -n 1 "$WS/.aic_active_workspace")"
  if [ "$(realpath "$ACTIVE_WS")" != "$(realpath "$WS")" ] && [ -f "$ACTIVE_WS/ros_round_omni_start.sh" ]; then
    echo "[车型] 圆形全向轮；转交运行工作空间：$ACTIVE_WS"
    exec bash "$ACTIVE_WS/ros_round_omni_start.sh" "$@"
  fi
fi
SELF="$WS/$(basename "$0")"
RUN_DIR="$WS/log/run"
PID_FILE="$RUN_DIR/pids"
PREFLIGHT_PY="$WS/tools/preflight_check.py"
PREFLIGHT_OUT="$RUN_DIR/preflight.txt"
NAMES="01_gazebo 02_moveit 03_nav2 04_llama 05_parser 06_nav 07_arm 08_detector 09_main 10_rosbridge 11_watchdog"
WATCH_PERIOD=30
NAV2_BIN_PATTERNS="/bot_navigation/scan_velocity_guard.py /bot_navigation/known_obstacle_forecaster.py /nav2_collision_monitor/collision_monitor /nav2_controller/controller_server /nav2_smoother/smoother_server /nav2_planner/planner_server /nav2_behaviors/behavior_server /nav2_bt_navigator/bt_navigator /nav2_waypoint_follower/waypoint_follower /nav2_velocity_smoother/velocity_smoother /nav2_amcl/amcl /nav2_map_server/map_server /nav2_lifecycle_manager/lifecycle_manager"

if grep -q $'\r' "$0" 2>/dev/null; then
  echo "[init] 脚本自身是 CRLF 行尾，转换为 LF 后重新执行..."
  sed -i 's/\r$//' "$0"
  exec bash "$0" "$@"
fi

CMD=start
NO_RVIZ=0
NO_GUI=0
NO_WATCHDOG=0
LOG_TARGET=""
for a in "$@"; do
  case "$a" in
    start|stop|status|restart|recover|logs|doctor|watchdog) CMD="$a" ;;
    --no-rviz) NO_RVIZ=1 ;;
    --headless) NO_GUI=1; NO_RVIZ=1 ;;
    --no-watchdog) NO_WATCHDOG=1 ;;
    -h|--help) CMD=help ;;
    *) LOG_TARGET="$a" ;;
  esac
done

mkdir -p "$RUN_DIR"
touch "$PID_FILE"

pid_of() { awk -F: -v n="$1" '$1==n {print $2}' "$PID_FILE" 2>/dev/null | tail -1; }
drop_pid() { grep -v "^$1:" "$PID_FILE" > "$PID_FILE.tmp" 2>/dev/null; mv -f "$PID_FILE.tmp" "$PID_FILE" 2>/dev/null; }
is_running() { p="$(pid_of "$1")"; [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }

has_publisher() { ros2 topic info "$1" 2>/dev/null | grep -qE 'Publisher count: [1-9]'; }
has_data() {
  if [ "$1" = "/amcl_pose" ]; then
    timeout -k 2 15 ros2 topic echo --once --no-daemon --qos-reliability reliable --qos-durability transient_local "$1" geometry_msgs/msg/PoseWithCovarianceStamped >/dev/null 2>&1
  else
    timeout -k 2 15 ros2 topic echo --once --no-daemon --qos-reliability best_effort "$1" sensor_msgs/msg/LaserScan >/dev/null 2>&1
  fi
}
has_node()      { ros2 node list 2>/dev/null | grep -qx "$1"; }
has_action()    { ros2 action list 2>/dev/null | grep -qx "$1"; }
has_service()   { ros2 service list 2>/dev/null | grep -qx "$1"; }
is_active()     { timeout 12 ros2 lifecycle get "$1" 2>/dev/null | grep -q active; }
llama_ok()      { curl -s -m 2 http://127.0.0.1:8081/health 2>/dev/null | grep -q ok; }
port_open()     { ss -ltn 2>/dev/null | grep -q ":$1 " || has_node /rosbridge_websocket; }
map_frame_ok()  { timeout 15 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -qm1 'Translation'; }

wait_gate() {
  local desc="$1" limit="$2" gate_start=$SECONDS
  shift 2
  while [ "$((SECONDS-gate_start))" -lt "$limit" ]; do
    if "$@" >/dev/null 2>&1; then echo "[就绪] $desc（$((SECONDS-gate_start)) s）"; return 0; fi
    sleep 3
  done
  echo "[警告] $desc 等待 $limit s 超时，继续后续步骤"
  return 1
}

start_mod() {
  name="$1"; shift
  if is_running "$name"; then echo "[跳过] $name 已在运行（pid=$(pid_of "$name")）"; return 0; fi
  drop_pid "$name"
  setsid nohup "$@" > "$RUN_DIR/$name.log" 2>&1 &
  p=$!
  echo "$name:$p" >> "$PID_FILE"
  echo "[启动] $name（pid=$p）: $*"
}

load_env() {
  if [ "$(id -u)" = "0" ]; then
    echo "错误：不要用 root/sudo 运行（ROS 节点不需要 root，且 HOME 会变成 /root）"; exit 1
  fi
  DISTRO="$(printenv ROS_DISTRO 2>/dev/null)"
  [ -z "$DISTRO" ] && DISTRO=humble
  if [ ! -f "/opt/ros/$DISTRO/setup.bash" ]; then echo "错误：找不到 /opt/ros/$DISTRO/setup.bash"; exit 1; fi
  if [ ! -f "$WS/install/setup.bash" ]; then
    echo "错误：$WS/install/setup.bash 不存在 —— 工作空间还没构建，先执行："
    echo "  cd $WS && source /opt/ros/$DISTRO/setup.bash && colcon build --symlink-install --base-paths src"
    exit 1
  fi
  # ROS / colcon 生成的 setup.bash 会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），必须先关掉 set -u
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/$DISTRO/setup.bash"
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  source "$WS/tools/ros_env.sh"

  LLAMA_BIN=""
  for c in "$WS/runtime/llama.cpp/build/bin/llama-server" "$HOME/ws_aic/runtime/llama.cpp/build/bin/llama-server" "$HOME/AIC/runtime/llama.cpp/build/bin/llama-server"; do
    if [ -x "$c" ]; then LLAMA_BIN="$c"; break; fi
  done
  [ -z "$LLAMA_BIN" ] && LLAMA_BIN="$(command -v llama-server 2>/dev/null || true)"
  MODEL=""
  for m in "$WS/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" "$HOME/ws_aic/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" "$HOME/AIC/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf"; do
    if [ -f "$m" ]; then MODEL="$m"; break; fi
  done
  if [ -z "$LLAMA_BIN" ] || [ -z "$MODEL" ]; then
    echo "警告：找不到 llama-server 或 GGUF 权重，第 4 步会失败"
    echo "      llama-server=$LLAMA_BIN"
    echo "      model=$MODEL"
    echo "      处理见 AGENTS.md 6.2（可复用 ~/ws_aic 或 ~/AIC 里的二进制与权重）"
  fi
}

# ---------- 定位与体检 ----------

# 从 /amcl_pose 取 "x y yaw"（无数据则输出空）
amcl_pose3() {
  timeout 12 ros2 topic echo --once --field pose.pose.position /amcl_pose > "$RUN_DIR/.amcl_pos" 2>/dev/null
  timeout 12 ros2 topic echo --once --field pose.pose.orientation /amcl_pose > "$RUN_DIR/.amcl_ori" 2>/dev/null
  python3 - "$RUN_DIR/.amcl_pos" "$RUN_DIR/.amcl_ori" <<'PYEOF'
import math, re, sys
def field(path, key):
    try:
        txt = open(path).read()
    except OSError:
        return None
    m = re.search(r'^\s*%s:\s*([-\d.eE+]+)' % key, txt, re.M)
    return float(m.group(1)) if m else None
px, py = field(sys.argv[1], 'x'), field(sys.argv[1], 'y')
qz, qw = field(sys.argv[2], 'z'), field(sys.argv[2], 'w')
if None in (px, py, qz, qw):
    print('')
else:
    print('%.3f %.3f %.4f' % (px, py, 2 * math.atan2(qz, qw)))
PYEOF
}

# Gazebo 真位姿 "x y yaw"（仿真专属；拿不到则输出空）
gz_pose3() {
  timeout 25 gz model -m six_arm -p 2>/dev/null | head -1 | awk 'NF >= 6 {print $1, $2, $6}'
}

# 用给定位姿重新初始化 AMCL，并验证已生效（Nav2 重启后必须做，否则 AMCL 回到 0,0,0）
# 用 tools/set_initial_pose.py 而不是 ros2 topic pub：后者的 YAML 解析对嵌套消息很脆，
# 实测直接抛 yaml.composer 异常、消息根本没发出去。
set_initial_pose() {
  x="$1"; y="$2"; yaw="$3"
  echo "[定位] 重设 AMCL 初始位姿：x=$x y=$y yaw=$yaw"
  timeout 90 python3 "$WS/tools/set_initial_pose.py" "$x" "$y" "$yaw" 2>&1 | grep -vE "RTPS_TRANSPORT_SHM|open_and_lock_file"
}

# 直接用 Gazebo 真位姿重设（仿真里最可靠；拿不到就返回非 0）
set_initial_pose_from_truth() {
  echo "[定位] 用 Gazebo 真位姿重设 AMCL 初始位姿"
  timeout 90 python3 "$WS/tools/set_initial_pose.py" --from-gazebo 2>&1 | grep -vE "RTPS_TRANSPORT_SHM|open_and_lock_file"
}

# 体检：输出写到 $PREFLIGHT_OUT，返回其退出码
# 0=健康 2=规划器起点与定位不一致 4=规划器不可用 5=TF 与 AMCL 不一致 6=定位跑飞 7=没有 map 帧 8=控制器 TF 过期
run_preflight() {
  if [ ! -f "$PREFLIGHT_PY" ]; then
    echo "[体检] 找不到 $PREFLIGHT_PY，跳过"; return 0
  fi
  timeout 200 python3 "$PREFLIGHT_PY" > "$RUN_DIR/.preflight_raw" 2>&1
  rc=$?
  grep -vE 'RTPS_TRANSPORT_SHM|open_and_lock_file' "$RUN_DIR/.preflight_raw" > "$PREFLIGHT_OUT"
  cat "$PREFLIGHT_OUT"
  return "$rc"
}

nav2_recover() {
  python3 "$WS/tools/nav_recovery_signal.py" true
  gzpose="$(gz_pose3)"
  pose="$gzpose"
  [ -z "$pose" ] && pose="$(amcl_pose3)"
  echo "[修复] 整组重启 Nav2（planner_server 未加载 / 位姿视图停更 / map->odom 断流）..."
  nav_pid="$(pid_of 03_nav2)"
  # Independent Nav2 servers are children in the launch process group.
  # A container-name cleanup alone would leave these servers running.
  if [ -n "$nav_pid" ]; then
    kill -INT -- "-$nav_pid" 2>/dev/null
    sleep 5
    kill -TERM -- "-$nav_pid" 2>/dev/null
    sleep 2
    kill -KILL -- "-$nav_pid" 2>/dev/null
  fi
  pkill -INT -f nav_bringup_gazebo.launch 2>/dev/null
  sleep 2
  pkill -f component_container_isolated 2>/dev/null
  sleep 5
  pkill -KILL -f 'component_container|nav_bringup_gazebo|bright_costmap' 2>/dev/null
  sleep 2
  for pat in $NAV2_BIN_PATTERNS; do pkill -KILL -f "$pat" 2>/dev/null; done
  drop_pid 03_nav2
  start_mod 03_nav2 ros2 launch bot_navigation nav_bringup_gazebo.launch.py rviz:=$([ "$NO_RVIZ" = "1" ] && echo false || echo true)
  wait_gate "/planner_server 处于 active（重启后）" 150 is_active /planner_server
  wait_gate "/bt_navigator 处于 active（重启后）" 90 is_active /bt_navigator
  wait_gate "/collision_monitor 处于 active（重启后）" 60 is_active /collision_monitor
  wait_gate "/scan_velocity_guard 节点（重启后）" 30 has_node /scan_velocity_guard
  if [ -n "$gzpose" ]; then
    set_initial_pose_from_truth
  elif [ -n "$pose" ]; then
    # shellcheck disable=SC2086
    set_initial_pose $pose
  else
    echo "[警告] 重启前没拿到位姿，AMCL 会回到 initial_pose(0,0,0)；机器人不在原点时请手工发 /initialpose"
  fi
  wait_gate "map->odom 恢复发布" 90 map_frame_ok
  python3 "$WS/tools/nav_recovery_signal.py" false
}

heal_nav() {
  code="$1"
  case "$code" in
    10)
      echo "[体检] 服务仍有响应，但当前位置到体检目标无路径；保留 Nav2，不反复重启"
      return 10
      ;;
    9)
      echo "[体检] Gazebo 物理状态已损坏；Nav2 重启无法修复，请 stop 后重新 start"
      return 9
      ;;
    6)
      if gz_pose3 | grep -q .; then
        set_initial_pose_from_truth
      else
        echo "[自愈] 定位跑飞但拿不到 Gazebo 真值，改为整组重启 Nav2"
        nav2_recover
      fi
      ;;
    *)
      nav2_recover
      ;;
  esac
}

doctor() {
  load_env
  echo "=== 导航链路体检 ==="
  run_preflight
  rc=$?
  if [ "$rc" = "0" ]; then
    echo "[体检] 通过：规划起点与定位一致，导航链路可用"
    return 0
  fi
  echo "[体检] 未通过（退出码 $rc），执行自愈..."
  if [ "$rc" = "9" ]; then heal_nav "$rc"; return 9; fi
  heal_nav "$rc"
  sleep 3
  echo "=== 自愈后复检 ==="
  run_preflight
  rc2=$?
  if [ "$rc2" = "0" ]; then
    echo "[体检] 自愈成功"
  else
    echo "[体检] 自愈后仍不通过（退出码 $rc2）：常见处理办法见 AGENTS.md 第 8 节"
  fi
  return "$rc2"
}

watchdog_loop() {
  load_env
  echo "[watchdog] 启动：每 $WATCH_PERIOD s 体检一次；导航链路坏死时自动重启 Nav2 并重设 AMCL 初始位姿"
  while true; do
    sleep "$WATCH_PERIOD"
    if [ ! -f "$PREFLIGHT_PY" ]; then continue; fi
    extra=""
    while [ "$#" -gt 0 ]; do extra="$extra $1"; shift; done
    run_preflight >/dev/null
    rc=$?
    if [ "$rc" != "0" ]; then
      echo "[watchdog $(date +%H:%M:%S)] 体检未通过（退出码 $rc）：$(tail -1 "$PREFLIGHT_OUT")"
      if [ "$rc" = "9" ]; then heal_nav "$rc"; return 9; fi
      heal_nav "$rc"
      sleep 5
      run_preflight >/dev/null
      rc2=$?
      if [ "$rc2" = "0" ]; then
        echo "[watchdog $(date +%H:%M:%S)] 自愈成功：$(tail -1 "$PREFLIGHT_OUT")"
      else
        echo "[watchdog $(date +%H:%M:%S)] 自愈未成功（退出码 $rc2）：$(tail -1 "$PREFLIGHT_OUT")"
      fi
    fi
  done
}

arm_ready() {
  timeout 8 ros2 topic info /nav_done_cargo -v 2>/dev/null | grep -q 'Node name: arm_grab_place_node'
}

start_all() {
  load_env
  if ! is_running 01_gazebo && ! is_running 03_nav2 && command -v fastdds >/dev/null 2>&1; then
    timeout -k 2 8 ros2 daemon stop >/dev/null 2>&1 || true
    # The upstream tool removes only abandoned shared-memory files.
    timeout 8 fastdds shm clean > "$RUN_DIR/dds-clean.log" 2>&1 || true
  fi
  echo "工作空间 : $WS"
  echo "通信实现 : $RMW_IMPLEMENTATION"
  echo "地图定位 : $([ "$AIC_ODOM_MAP" = "1" ] && echo '仿真里程计对齐' || echo 'AMCL 动态变换')"
  echo "llama    : $LLAMA_BIN"
  echo "权重     : $MODEL"
  echo "日志目录 : $RUN_DIR"
  echo

  if ! is_running 01_gazebo && pgrep -f gzserver >/dev/null 2>&1; then
    echo "[清理] 发现残留 Gazebo 进程，先关闭"
    pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; sleep 2
  fi

  echo "=== 1/11 仿真（底盘 + 六轴臂 + 传感器）==="
  start_mod 01_gazebo ros2 launch mybot gazebo_world.launch.py gui:=$([ "$NO_GUI" = "1" ] && echo false || echo true)
  if ! wait_gate "/scan 有发布者" 120 has_publisher /scan; then
    echo "[补救] 疑似 spawn_entity 超时，手工重新生成机器人..."
    grep -iE 'spawn service failed|Service /spawn_entity unavailable' "$RUN_DIR/01_gazebo.log" 2>/dev/null | tail -2
    setsid nohup ros2 run gazebo_ros spawn_entity.py -entity six_arm -topic robot_description -timeout 180 > "$RUN_DIR/01b_respawn.log" 2>&1 &
    wait_gate "/scan 有发布者（补救后）" 180 has_publisher /scan
  fi
  wait_gate "/scan 有数据" 60 has_data /scan
  echo "=== 验证机械臂控制器（不依赖 spawn 服务回执）==="
  if ! timeout 100 python3 "$WS/tools/ensure_controllers.py"; then
    echo "[失败] 机械臂控制器未就绪，保留诊断现场；请先处理后再下发任务"
    return 1
  fi

  echo "=== 2/11 MoveIt（机械臂规划与控制）==="
  start_mod 02_moveit ros2 launch mybot my_moveit_rviz.launch.py rviz:=$([ "$NO_RVIZ" = "1" ] && echo false || echo true)
  wait_gate "/move_group 节点" 150 has_node /move_group

  echo "=== 3/11 Nav2（导航栈）==="
  start_mod 03_nav2 ros2 launch bot_navigation nav_bringup_gazebo.launch.py rviz:=$([ "$NO_RVIZ" = "1" ] && echo false || echo true)
  wait_gate "/navigate_to_pose 动作" 240 has_action /navigate_to_pose
  attempt=0
  while [ "$attempt" -lt 2 ]; do
    if wait_gate "/planner_server 处于 active" 90 is_active /planner_server; then break; fi
    attempt=$((attempt+1))
    if [ "$attempt" -lt 2 ]; then nav2_recover; fi
  done
  wait_gate "/bt_navigator 处于 active" 60 is_active /bt_navigator
  wait_gate "/collision_monitor 处于 active" 60 is_active /collision_monitor
  wait_gate "/scan_velocity_guard 节点" 30 has_node /scan_velocity_guard
  wait_gate "/amcl_pose 有数据" 120 has_data /amcl_pose

  echo "=== 4/11 llama-server（本地大模型服务端）==="
  start_mod 04_llama "$LLAMA_BIN" -m "$MODEL" -c 2048 --threads 8 --port 8081
  wait_gate "llama-server /health" 120 llama_ok

  echo "=== 5-9/11 业务节点 ==="
  start_mod 05_parser ros2 run llama_command_parser command_parser
  start_mod 06_nav ros2 run nav_simple simple_navigator
  wait_gate "/simple_nav2_navigator 节点" 30 has_node /simple_nav2_navigator
  start_mod 07_arm ros2 run arm_action_integration arm_grab_place_node
  start_mod 08_detector ros2 run package_detector color_detector_node
  start_mod 09_main ros2 run main_controller main_controller_node
  wait_gate "/main_controller_node 节点" 30 has_node /main_controller_node
  wait_gate "/ATTACHLINK 吸附服务" 90 has_service /ATTACHLINK
  if ! wait_gate "机械臂已完成初始化并订阅抓取指令" 60 arm_ready; then
    echo "[失败] 机械臂业务链未就绪；不要下发任务"
    return 1
  fi

  echo "=== 10/11 rosbridge（Foxglove 看板）==="
  start_mod 10_rosbridge ros2 launch "$WS/tools/rosbridge_safe.launch.py"
  wait_gate "rosbridge :9090" 60 port_open 9090

  if [ "$NO_RVIZ" = "1" ]; then
    echo "[--no-rviz] 关闭 rviz 视图进程（纯观看用，主链路不依赖；WSLg 下 rviz2 易崩）"
    pkill -f rviz2 2>/dev/null
  fi

  echo
  echo "=== 11a/11 导航链路体检（发令前必须通过）==="
  doctor

  if [ "$NO_WATCHDOG" = "1" ]; then
    echo "[--no-watchdog] 不启动看门狗"
  else
    echo "=== 11b/11 看门狗（后台每 $WATCH_PERIOD s 复检一次）==="
    if [ "$NO_RVIZ" = "1" ]; then
      start_mod 11_watchdog "$SELF" watchdog --no-rviz
    else
      start_mod 11_watchdog "$SELF" watchdog
    fi
  fi

  echo
  echo "================= 启动结果 ================="
  show_status
  echo
  echo "下发任务： ros2 topic pub -r 2 /command std_msgs/String \"{data: '抓取1个蓝色去B'}\""
  echo "看板    ： 浏览器打开 Foxglove，连接 ws://localhost:9090，导入 demo1.json"
  echo "体检    ： $SELF doctor        （发令前/异常时先跑这个）"
  echo "停止    ： $SELF stop"
  echo "==========================================="
}

show_status() {
  echo "工作空间：$WS"
  printf '%-14s %-8s %-9s %s\n' module pid state "last-log-line"
  for n in $NAMES; do
    p="$(pid_of "$n")"
    [ -z "$p" ] && p="-"
    if [ "$p" != "-" ] && kill -0 "$p" 2>/dev/null; then st=RUNNING; else st=stopped; fi
    last="$(tail -1 "$RUN_DIR/$n.log" 2>/dev/null | tr -d '\r' | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-72)"
    printf '%-14s %-8s %-9s %s\n' "$n" "$p" "$st" "$last"
  done
  echo "--- 就绪检查 ---"
  printf 'llama-server /health  : '; llama_ok && echo ok || echo "no"
  ps_state="$(timeout 12 ros2 lifecycle get /planner_server 2>/dev/null | head -1)"
  [ -z "$ps_state" ] && ps_state="n/a（节点未加载）"
  printf '/planner_server       : %s\n' "$ps_state"
  bt_state="$(timeout 12 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)"
  [ -z "$bt_state" ] && bt_state="n/a（节点未加载）"
  printf '/bt_navigator         : %s\n' "$bt_state"
  scan_pub="$(ros2 topic info /scan 2>/dev/null | grep -E 'Publisher count')"
  [ -z "$scan_pub" ] && scan_pub="none"
  printf '/scan 发布者          : %s\n' "$scan_pub"
  printf '/ATTACHLINK 服务      : '; has_service /ATTACHLINK && echo ok || echo "no"
  printf 'rosbridge :9090       : '; port_open 9090 && echo ok || echo "no"
  printf '最近一次导航体检      : %s\n' "$(tail -1 "$PREFLIGHT_OUT" 2>/dev/null || echo '(未跑过)')"
  echo "--- 相关进程（含未被本脚本跟踪的）---"
  ps -eo pid,args | grep -E 'gzserver|gzclient|move_group|rviz2|llama-server|main_controller|simple_navigator|command_parser|arm_grab|color_detector|rosbridge|component_container|scan_velocity_guard|collision_monitor|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|/nav2_amcl/amcl|/nav2_map_server/map_server|/nav2_lifecycle_manager/lifecycle_manager|/robot_state_publisher/robot_state_publisher' | grep -v grep | cut -c1-108
}

stop_all() {
  echo "=== 停止全部模块（按进程组 SIGINT → SIGTERM → SIGKILL）==="
  for sig in INT TERM KILL; do
    for n in $NAMES; do
      p="$(pid_of "$n")"
      if [ -n "$p" ]; then kill -"$sig" -"$p" 2>/dev/null && echo "[$sig] $n（pgid=$p）"; fi
    done
    sleep 4
  done
  echo "=== 兜底清扫（按进程名）==="
  for pat in $NAV2_BIN_PATTERNS 'ros_competition_start.sh watchdog' 'ros2 launch' 'ros2 run' component_container nav_bringup_gazebo move_group rviz2 bright_costmap gzserver gzclient llama-server spawn_entity /robot_state_publisher/robot_state_publisher; do
    pkill -f "$pat" 2>/dev/null
  done
  sleep 3
  : > "$PID_FILE"
  left="$(ps -eo pid,stat,args | grep -E 'gzserver|gzclient|move_group|rviz2|llama-server|main_controller|simple_navigator|command_parser|arm_grab|color_detector|rosbridge|component_container|scan_velocity_guard|collision_monitor|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|/nav2_amcl/amcl|/nav2_map_server/map_server|/nav2_lifecycle_manager/lifecycle_manager|/robot_state_publisher/robot_state_publisher' | grep -v grep | grep -v defunct | cut -c1-108)"
  if [ -z "$left" ]; then
    echo "已全部停止，无残留（僵尸进程会自动被回收，不计入残留）"
  else
    echo "仍有残留，可再执行一次 stop 或手动 kill："
    echo "$left"
  fi
}

show_logs() {
  if [ -n "$LOG_TARGET" ]; then
    tail -n 40 -f "$RUN_DIR/$LOG_TARGET.log"
  else
    for n in $NAMES; do
      echo "=== $n（$RUN_DIR/$n.log）==="
      tail -n 5 "$RUN_DIR/$n.log" 2>/dev/null || echo "(无日志)"
    done
  fi
}

case "$CMD" in
  start)    start_all ;;
  stop)     stop_all ;;
  status)   load_env; show_status ;;
  restart)  stop_all; echo; start_all ;;
  recover)  load_env; nav2_recover ;;
  doctor)   doctor ;;
  watchdog) watchdog_loop ;;
  logs)     show_logs ;;
  help|*)   sed -n '2,24p' "$0" ;;
esac
