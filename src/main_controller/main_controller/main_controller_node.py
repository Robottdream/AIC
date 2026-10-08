import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.timer import Timer
import json
import math
import weakref
from itertools import permutations

class MainControllerNode(Node):
    def __init__(self):
        super().__init__("main_controller_node")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        
        # QoS配置
        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 预设位置参数
        self.RED_BLOCKS = [
            (x, y, False) for x, y in self._position_pairs_parameter(
                "red_block_positions", [
                    7.632928, 5.523903,
                    9.604514, -3.707741,
                    -5.735783, 5.507306,
                    -8.702837, 1.000008,
                    -10.748330, 4.014158,
                ])
        ]
        self.BLUE_BLOCKS = [
            (x, y, False) for x, y in self._position_pairs_parameter(
                "blue_block_positions", [
                    8.464876, -7.097603,
                    5.040937, -7.441163,
                    -1.478995, 6.646223,
                    -9.664453, -3.267239,
                    -3.703343, 0.829596,
                ])
        ]
        # Base parking poses; the arm still checks the unchanged physical zone bounds.
        self.AREA_COORDS = dict(zip(
            ("A", "B", "C"),
            self._position_pairs_parameter("area_parking_positions", [
                2.593086, -5.970000,
                -1.746544, -6.585499,
                -6.873777, -7.785160,
            ]),
        ))
        self.stable_layout_hints = self.declare_parameter("stable_layout_hints", True).value
        
        # 核心变量
        self.current_robot_pose = (0.0, 0.0)
        self.current_robot_yaw = 0.0
        self.turn_cost_weight = float(self.declare_parameter("turn_cost_weight", 0.20).value)
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.selection_id = 0
        self.failed_blocks_for_step = set()
        self.selection_candidates = []
        self.selection_best = None
        self.selection_options = []
        self.target_blocks = []
        self.GRASP_OFFSET = 0.55
        self.OFFSET_AXIS = "x"
        self.retry_timer = None
        self.grasp_timer = None
        self.place_timer = None
        self.current_task = None  # 当前执行任务
        self.task_queue = []      # 原始任务队列
        self.optimized_tasks = [] # 优化后的任务执行顺序
        self.completed_num = 0    # 当前任务已完成数量
        self.current_step = "WAIT_TASK"  # 状态：WAIT_TASK/NAV_TO_BLOCK/GRASP/NAV_TO_AREA/PLACE
        self.selected_block = None
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.max_grasp_retry = 3
        self.current_grasp_retry = 0
        self.closest_cache = None
        self.cache_expire = 2.0
        self.last_cache_time = 0.0
        
        # 超时设置
        self.GRASP_TIMEOUT = 8.0
        self.PLACE_TIMEOUT = 8.0
        self.max_place_retry = 3
        self.current_place_retry = 0
        
        # 订阅器
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, self.qos_best_effort
        )
        self.chat_sub = self.create_subscription(
            String, "/chat", self._chat_callback, self.qos_best_effort
        )
        self.nav_status_sub = self.create_subscription(
            String, "/nav_status", self._nav_status_callback, 10
        )
        self.arm_status_sub = self.create_subscription(
            String, "/arm_status", self._arm_status_callback, 10
        )
        
        # 发布器
        self.target_cube_pub = self.create_publisher(String, "/current_target_cube", 10)
        self.nav_target_pub = self.create_publisher(String, "/manual_nav_target", 10)
        self.arm_cargo_pub = self.create_publisher(String, "/nav_done_cargo", 10)
        self.arm_area_pub = self.create_publisher(String, "/nav_done_area", 10)
        self.foxglove_pubs = {
            "color": self.create_publisher(Int32, "/color", 10),
            "ask": self.create_publisher(Int32, "/ask", 10),
            "pick": self.create_publisher(Int32, "/pick", 10),
            "cur": self.create_publisher(Int32, "/cur", 10)
        }
        
        # 新增发布器：目标区域和抓取数量
        self.target_area_pub = self.create_publisher(Int32, "/target_area", 10)
        self.number_pick_pub = self.create_publisher(Int32, "/number_pick", 10)
        
        self.get_logger().info("主控节点（支持智能路径规划）启动成功")
        
        # 初始化状态
        self._update_foxglove()
        # 初始状态：空闲
        self.target_area_pub.publish(Int32(data=9))
        self.number_pick_pub.publish(Int32(data=0))
    
    def _position_pairs_parameter(self, name, default):
        values = self.declare_parameter(name, default).value
        if len(values) != len(default) or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain {len(default)} finite coordinates")
        return list(zip(values[::2], values[1::2]))

    # 计算两点之间的距离
    def calculate_distance(self, point1, point2):
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])
    
    # 生成抓取偏移位置
    def get_grasp_position(self, block_pos):
        return self.get_grasp_candidates(block_pos)[0][:2]

    def get_grasp_candidates(self, block_pos):
        """抓取距离保持不变，换方位时让车头始终朝向货物。"""
        x, y = block_pos
        d = self.GRASP_OFFSET
        return [
            (x - d, y, 0.0),
            (x, y - d, math.pi / 2),
            (x + d, y, math.pi),
            (x, y + d, -math.pi / 2),
        ]
    
    # 获取符合条件的物块列表
    def get_available_blocks(self, color):
        blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
        available = []
        for i, (x, y, is_grasped) in enumerate(blocks):
            if not is_grasped:
                available.append((x, y, i))
        return available
    
    def _pose_stamped(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @staticmethod
    def _path_length(path):
        points = path.poses
        return sum(
            math.hypot(
                b.pose.position.x - a.pose.position.x,
                b.pose.position.y - a.pose.position.y,
            ) for a, b in zip(points, points[1:])
        )

    @staticmethod
    def _endpoint_turn_cost(path, start_yaw, end_yaw):
        """Estimate endpoint rotations using 0.35 m chords, avoiding grid jitter."""
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if len(points) < 2:
            return 0.0
        def heading(origin, candidates, reverse=False):
            for point in candidates:
                dx, dy = point[0] - origin[0], point[1] - origin[1]
                if math.hypot(dx, dy) >= 0.35:
                    return math.atan2(-dy, -dx) if reverse else math.atan2(dy, dx)
            return None
        first = heading(points[0], points[1:])
        last = heading(points[-1], reversed(points[:-1]), reverse=True)
        def difference(a, b):
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))
        return ((difference(first, start_yaw) if first is not None else 0.0)
                + (difference(end_yaw, last) if last is not None else 0.0))

    def _select_reachable_block(self):
        """每件货物从当前定位出发，按 Nav2 实际往返路径重新选块。"""
        if not self.current_assignment:
            return
        self.selection_id += 1
        selection_id = self.selection_id
        self.current_step = "SELECT_BLOCK"
        self._update_foxglove()
        color = self.current_task["color"]
        self.selection_candidates = [
            (block, index, candidate)
            for block in self.get_available_blocks(color)
            if block[2] not in self.failed_blocks_for_step
            for index, candidate in enumerate(self.get_grasp_candidates(block[:2]))
        ]
        self.selection_candidates.sort(
            key=lambda item: self.calculate_distance(self.current_robot_pose, item[2])
        )
        self.selection_best = None
        self.selection_options = []
        if not self.selection_candidates:
            self._stop_failed_task(f"没有可选的{color}物块")
            return
        if not self.plan_client.wait_for_server(timeout_sec=2.0):
            self._stop_failed_task("Nav2路径规划服务不可用")
            return
        self.get_logger().info(
            f"按当前代价地图检查{len(self.selection_candidates)}个抓取位置及返回目标区域的路径"
        )
        self._plan_next_candidate(selection_id)

    def _approach_penalty(self, color, block_index, candidate_index):
        # These approaches completed the full mission without a progress
        # recovery. Keep alternatives available if the preferred path closes.
        if not self.stable_layout_hints:
            return 0.0
        preferred = {("red", 1): 0, ("red", 2): 2, ("blue", 4): 1}
        direction = preferred.get((color, block_index))
        return 20.0 if direction is not None and candidate_index != direction else 0.0

    def _remaining_same_area_tasks(self):
        """Only consecutive deliveries to this area can reuse its return paths."""
        remaining = 0
        for assignment in self.optimized_tasks:
            task = assignment["task"]
            if (task["color"] != self.current_task["color"]
                    or task["to"] != self.current_task["to"]):
                break
            remaining += 1
        return remaining

    def _choose_route_with_lookahead(self):
        """Reserve cheap area round trips for later deliveries.

        NavFn paths are bidirectional on this static map, so the planned
        candidate-to-area length also estimates the next area-to-candidate leg.
        Actual reachability is checked again before every later pickup.
        """
        remaining = self._remaining_same_area_tasks()
        if remaining == 0:
            return self.selection_best
        # In this five-cargo layout, the west red block is safest from the
        # initial pose. Reaching it later from area A crossed the narrow wall
        # during validation; use the previously completed order when reachable.
        if (self.current_task["color"] == "red" and self.current_task["to"] == "A"
                and self.current_task.get("original_num") == 3
                and self.current_task.get("current_index") == 1):
            preferred = [option for option in self.selection_options if option[2][2] == 3]
            if preferred:
                cost, _, block, direction = min(preferred, key=lambda option: option[0])
                return (cost, block, direction)

        future_cost_by_block = {}
        for current_cost, return_length, block, candidate_index in self.selection_options:
            future_cost = 2.0 * return_length + self._approach_penalty(
                self.current_task["color"], block[2], candidate_index
            )
            block_index = block[2]
            future_cost_by_block[block_index] = min(
                future_cost, future_cost_by_block.get(block_index, float("inf"))
            )

        best = None
        for current_cost, _, block, candidate_index in self.selection_options:
            future = sorted(
                cost for index, cost in future_cost_by_block.items()
                if index != block[2]
            )
            if len(future) < remaining:
                continue
            score = current_cost + sum(future[:remaining])
            key = (score, current_cost, block[2], candidate_index)
            if best is None or key < best[0]:
                best = (key, (current_cost, block, candidate_index))
        if best is None:
            return self.selection_best
        # A lookahead estimate must not send this delivery on a much longer
        # current round trip; live obstacle positions make future paths noisy.
        if best[1][0] > self.selection_best[0] + 3.0:
            return self.selection_best
        return best[1]

    def _plan_next_candidate(self, selection_id):
        if selection_id != self.selection_id or self.current_step != "SELECT_BLOCK":
            return
        if not self.selection_candidates:
            if self.selection_best is None:
                self._stop_failed_task("当前没有可达且可返回目标区域的同色物块")
                return
            greedy_best = self.selection_best
            option_summary = sorted(self.selection_options, key=lambda o: o[0])[:8]
            self.get_logger().info("候选往返代价: " + ", ".join(
                f"{self.current_task['color']}_cube_{option[2][2] + 1}"
                f"/方向{option[3] + 1}={option[0]:.1f}m"
                for option in option_summary))
            self.selection_best = self._choose_route_with_lookahead()
            _, block, candidate_index = self.selection_best
            if (block[2], candidate_index) != (greedy_best[1][2], greedy_best[2]):
                self.get_logger().info(
                    f"连续任务前瞻改选：{self.current_task['color']}_cube_{block[2] + 1}"
                    f"方向{candidate_index + 1}"
                )
            x, y, block_idx = block
            self.current_assignment.update(
                block_pos=(x, y), block_idx=block_idx,
                grasp_pos=self.get_grasp_candidates((x, y))[candidate_index][:2],
            )
            self.selected_block = (x, y, f"{self.current_task['color']}_cube_{block_idx + 1}", block_idx)
            self.current_grasp_candidate_index = candidate_index
            self.attempted_candidate_indices = set()
            self.get_logger().info(f"选定可达物块：{self.selected_block[2]}，接近方向{candidate_index + 1}")
            self.navigate_to_block()
            return
        block, candidate_index, candidate = self.selection_candidates.pop(0)
        self._active_candidate = (block, candidate_index, candidate)
        request = ComputePathToPose.Goal()
        request.goal = self._pose_stamped(*candidate)
        request.use_start = False
        future = self.plan_client.send_goal_async(request)
        future.add_done_callback(
            lambda result: self._plan_response(result, selection_id, False)
        )

    def _plan_response(self, future, selection_id, returning):
        if selection_id != self.selection_id or self.current_step != "SELECT_BLOCK":
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self._plan_next_candidate(selection_id)
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._plan_result(result, selection_id, returning)
            )
        except Exception as exc:
            self.get_logger().warn(f"检查候选路径失败：{exc}")
            self._plan_next_candidate(selection_id)

    def _plan_result(self, future, selection_id, returning):
        if selection_id != self.selection_id or self.current_step != "SELECT_BLOCK":
            return
        try:
            response = future.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.path.poses:
                self._plan_next_candidate(selection_id)
                return
            length = self._path_length(response.result.path)
            block, candidate_index, candidate = self._active_candidate
            if not returning:
                self._outbound_length = length
                self._outbound_turn_cost = self.turn_cost_weight * self._endpoint_turn_cost(
                    response.result.path, self.current_robot_yaw, candidate[2])
                request = ComputePathToPose.Goal()
                request.start = self._pose_stamped(*candidate)
                area = self.current_assignment["area_pos"]
                request.goal = self._pose_stamped(*area)
                request.use_start = True
                future = self.plan_client.send_goal_async(request)
                future.add_done_callback(
                    lambda result: self._plan_response(result, selection_id, True)
                )
                return
            # Express turning effort in equivalent metres; retain validated approach preferences.
            return_cost = length + self.turn_cost_weight * self._endpoint_turn_cost(
                response.result.path, candidate[2], 0.0)
            total_length = self._outbound_length + self._outbound_turn_cost + return_cost
            ranking_cost = total_length + self._approach_penalty(
                self.current_task["color"], block[2], candidate_index
            )
            self.selection_options.append(
                (ranking_cost, return_cost, block, candidate_index)
            )
            if self.selection_best is None or ranking_cost < self.selection_best[0]:
                self.selection_best = (ranking_cost, block, candidate_index)
            self._plan_next_candidate(selection_id)
        except Exception as exc:
            self.get_logger().warn(f"读取候选路径失败：{exc}")
            self._plan_next_candidate(selection_id)

    # 为单个任务分配最佳物块（考虑已使用的物块）
    def assign_best_block_to_task(self, task, used_blocks=None, current_pos=None):
        if used_blocks is None:
            used_blocks = set()
            
        if current_pos is None:
            current_pos = self.current_robot_pose
        color = task["color"]
        target_area = task["to"]
        area_pos = self.AREA_COORDS[target_area]
        
        best_block = None
        min_cost = float("inf")
        
        # 获取所有可用物块
        available_blocks = self.get_available_blocks(color)
        
        for block_pos in available_blocks:
            block_x, block_y, block_idx = block_pos
            if (color, block_idx) in used_blocks:
                continue
                
            # 计算抓取位置
            grasp_pos = self.get_grasp_position((block_x, block_y))
            
            # 计算成本（距离）
            # 从当前位置到抓取位置的距离
            distance = self.calculate_distance(current_pos, grasp_pos)
            # 从抓取位置到目标区域的距离
            distance += self.calculate_distance(grasp_pos, area_pos)
            
            if distance < min_cost:
                min_cost = distance
                best_block = block_pos
        
        return best_block, min_cost
    
    # 计算任务序列的总成本
    def calculate_task_sequence_cost(self, tasks):
        total_cost = 0.0
        current_pos = self.current_robot_pose
        used_blocks = set()
        
        # 首先为每个任务分配最佳物块
        task_assignments = []
        for task in tasks:
            best_block, cost = self.assign_best_block_to_task(task, used_blocks, current_pos)
            if not best_block:
                return float("inf"), None  # 无法完成所有任务
            
            block_x, block_y, block_idx = best_block
            target_area = task["to"]
            area_pos = self.AREA_COORDS[target_area]
            grasp_pos = self.get_grasp_position((block_x, block_y))
            
            task_assignments.append({
                "task": task,
                "block_pos": (block_x, block_y),
                "block_idx": block_idx,
                "grasp_pos": grasp_pos,
                "area_pos": area_pos
            })
            
            used_blocks.add((task["color"], block_idx))
            total_cost += cost
            current_pos = area_pos
        
        return total_cost, task_assignments
    
    # 优化多任务执行顺序
    def optimize_task_order(self, tasks):
        """保留出题顺序；具体物块在每一步开始时按实时路径确定。"""
        counts = {"red": 0, "blue": 0}
        for task in tasks:
            color = task["color"]
            if color not in counts or task["to"] not in self.AREA_COORDS:
                self.get_logger().error(f"不支持的任务：{task}")
                return []
            counts[color] += 1
        for color, count in counts.items():
            if count > len(self.get_available_blocks(color)):
                self.get_logger().error(f"{color}物块数量不足：需要{count}个")
                return []
        assignments = [
            {"task": task, "area_pos": self.AREA_COORDS[task["to"]]}
            for task in tasks
        ]
        self.get_logger().info(f"按出题顺序生成{len(assignments)}个步骤，每步开始时重新规划可达物块")
        return assignments

    # 为同一任务的多个物块生成优化顺序
    def optimize_multi_block_order(self, task, remaining_count):
        """为同一任务中的多个物块生成优化的抓取顺序"""
        if remaining_count <= 0:
            return []
            
        self.get_logger().info(f"正在为任务优化{remaining_count}个物块的抓取顺序...")
        
        # 获取所有可用物块
        color = task["color"]
        available_blocks = self.get_available_blocks(color)
        
        if len(available_blocks) < remaining_count:
            self.get_logger().warn(f"可用物块数量不足，需要{remaining_count}个，实际只有{len(available_blocks)}个")
            remaining_count = len(available_blocks)
        
        # 生成所有可能的物块组合和顺序
        min_total_cost = float("inf")
        best_sequence = None
        
        # 从可用物块中选择remaining_count个
        from itertools import combinations
        
        # 为了避免计算量过大，限制最大组合数
        max_combinations = 1000
        combo_count = 0
        
        for block_combination in combinations(available_blocks, remaining_count):
            combo_count += 1
            if combo_count > max_combinations:
                self.get_logger().warn("组合数量过多，使用贪心算法替代")
                # 使用贪心算法
                best_sequence = self._greedy_multi_block_selection(task, remaining_count)
                break
                
            # 尝试所有排列顺序
            for block_order in permutations(block_combination):
                # 计算这个顺序的总成本
                total_cost = 0.0
                current_pos = self.current_robot_pose
                valid = True
                
                for block_pos in block_order:
                    block_x, block_y, block_idx = block_pos
                    grasp_pos = self.get_grasp_position((block_x, block_y))
                    area_pos = self.AREA_COORDS[task["to"]]
                    
                    # 计算成本
                    cost = self.calculate_distance(current_pos, grasp_pos) + \
                           self.calculate_distance(grasp_pos, area_pos)
                    
                    if cost == float("inf"):
                        valid = False
                        break
                        
                    total_cost += cost
                    current_pos = area_pos  # 下一个任务从目标区域开始
                
                if valid and total_cost < min_total_cost:
                    min_total_cost = total_cost
                    best_sequence = block_order
        
        if best_sequence is None:
            # 如果没有找到最佳序列，使用贪心算法
            best_sequence = self._greedy_multi_block_selection(task, remaining_count)
        
        # 转换为任务分配格式
        assignments = []
        for block_pos in best_sequence:
            block_x, block_y, block_idx = block_pos
            area_pos = self.AREA_COORDS[task["to"]]
            grasp_pos = self.get_grasp_position((block_x, block_y))
            
            assignments.append({
                "task": task,
                "block_pos": (block_x, block_y),
                "block_idx": block_idx,
                "grasp_pos": grasp_pos,
                "area_pos": area_pos,
                "multi_block": True  # 标记为多物块任务的一部分
            })
        
        self.get_logger().info(f"多物块优化完成，最佳路径总距离：{min_total_cost:.2f}米")
        return assignments
    
    # 贪心算法选择多物块顺序
    def _greedy_multi_block_selection(self, task, remaining_count):
        """贪心算法：每次选择当前最优的物块"""
        selected_blocks = []
        used_blocks = set()
        current_pos = self.current_robot_pose
        
        for _ in range(remaining_count):
            best_block = None
            min_cost = float("inf")
            
            # 获取所有可用物块
            color = task["color"]
            available_blocks = self.get_available_blocks(color)
            
            for block_pos in available_blocks:
                block_x, block_y, block_idx = block_pos
                if (color, block_idx) in used_blocks:
                    continue
                    
                # 计算抓取位置
                grasp_pos = self.get_grasp_position((block_x, block_y))
                area_pos = self.AREA_COORDS[task["to"]]
                
                # 计算成本
                cost = self.calculate_distance(current_pos, grasp_pos) + \
                       self.calculate_distance(grasp_pos, area_pos)
                
                if cost < min_cost:
                    min_cost = cost
                    best_block = block_pos
            
            if best_block:
                selected_blocks.append(best_block)
                used_blocks.add(best_block[2])
                current_pos = self.AREA_COORDS[task["to"]]  # 下一个从目标区域开始
            else:
                break
        
        return selected_blocks
    
    # AMCL定位回调
    @staticmethod
    def _amcl_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        q = msg.pose.pose.orientation
        self.current_robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        new_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if abs(new_pose[0] - self.current_robot_pose[0]) > 0.01 or \
           abs(new_pose[1] - self.current_robot_pose[1]) > 0.01:
            if self.current_robot_pose == (0.0, 0.0):
                self.get_logger().info(f"初始定位：({new_pose[0]:.2f}, {new_pose[1]:.2f})")
            self.current_robot_pose = new_pose
            self.closest_cache = None
    
    def _amcl_callback(self, msg):
        self_ref = weakref.ref(self)
        self._amcl_callback_impl(self_ref, msg)
        del self_ref
    
    # 聊天指令回调（接收任务）
    @staticmethod
    def _chat_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        try:
            task_json = json.loads(msg.data)
            # 支持单个任务对象或任务列表
            if not isinstance(task_json, list):
                task_json = [task_json]
            
            # 验证任务格式并添加到队列
            valid_tasks = []
            for task in task_json:
                required = ["color", "num", "to"]
                if all(k in task for k in required):
                    # 标准化参数
                    task["color"] = task["color"].lower()
                    task["to"] = task["to"].upper()
                    valid_tasks.append(task)
                else:
                    self.get_logger().error(f"任务格式错误（缺少color/num/to）：{task}")
            if valid_tasks:
                self.task_queue.extend(valid_tasks)
                self.get_logger().info(f"接收{len(valid_tasks)}个任务，队列长度：{len(self.task_queue)}")
                
                # 发布抓取数量（有命令时显示1）
                self.number_pick_pub.publish(Int32(data=1))
                
                # 如果当前没有正在执行的任务，立即优化并开始处理
                if self.current_task is None and self.current_step == "WAIT_TASK":
                    self._optimize_and_process_tasks()
        except json.JSONDecodeError:
            self.get_logger().error("任务解析失败（非JSON格式）")
    
    def _chat_callback(self, msg):
        self_ref = weakref.ref(self)
        self._chat_callback_impl(self_ref, msg)
        del self_ref
    
    # 优化并处理任务
    def _optimize_and_process_tasks(self):
        if not self.task_queue:
            return
        
        # 复制当前任务队列并清空
        current_tasks = self.task_queue.copy()
        self.task_queue = []
        
        # 展开多数量任务为单个任务
        expanded_tasks = []
        for task in current_tasks:
            num = task["num"]
            if num > 1:
                # 为多数量任务创建子任务
                for i in range(num):
                    subtask = task.copy()
                    subtask["num"] = 1
                    subtask["original_num"] = num
                    subtask["current_index"] = i + 1
                    expanded_tasks.append(subtask)
            else:
                expanded_tasks.append(task)
        
        # 优化任务执行顺序
        self.optimized_tasks = self.optimize_task_order(expanded_tasks)
        
        if not self.optimized_tasks:
            self.get_logger().error("任务优化失败，无法生成执行计划")
            # 没有任务时发布抓取数量为0
            self.number_pick_pub.publish(Int32(data=0))
            # 发布目标区域为空闲（9）
            self.target_area_pub.publish(Int32(data=9))
            return
        
        self.get_logger().info(f"任务优化完成，共生成{len(self.optimized_tasks)}个执行步骤")
        
        # 处理第一个优化任务
        self._process_next_optimized_task()
    
    # 处理下一个优化后的任务
    def _process_next_optimized_task(self):
        if not self.optimized_tasks:
            self.get_logger().info("所有优化任务执行完成，等待新任务...")
            self.current_task = None
            self.current_step = "WAIT_TASK"
            # 发布抓取数量为0（没有命令）
            self.number_pick_pub.publish(Int32(data=0))
            # 发布目标区域为空闲（9）
            self.target_area_pub.publish(Int32(data=9))
            self._update_foxglove()
            return
        
        # 取出第一个优化任务
        self.current_assignment = self.optimized_tasks.pop(0)
        self.current_task = self.current_assignment["task"]
        self.completed_num = 0
        self.current_step = "NAV_TO_BLOCK"
        
        self.selected_block = None
        # 重置状态变量
        self.grasp_confirmed = False
        self.place_confirmed = False
        self.current_grasp_retry = 0
        self.current_grasp_candidate_index = 0
        self.attempted_candidate_indices = set()
        self.failed_blocks_for_step = set()
        self.current_area_nav_retry = 0
        self.area_nav_targets = []
        self.area_nav_target_index = 0
        self.closest_cache = None
        self._clean_timers()
        
        color = self.current_task["color"]
        original_num = self.current_task.get("original_num", 1)
        current_index = self.current_task.get("current_index", 1)
        
        self.get_logger().info(
            f"开始执行优化任务：{current_index}/{original_num} 个{color}物块 → {self.current_task['to']}区"
        )
        
        # 根据area_pos确定目标区域编号
        area_pos = self.current_assignment["area_pos"]
        area_code = 9  # 默认空闲
        
        # 比较坐标来确定是哪个区域
        for area_name, coords in self.AREA_COORDS.items():
            if abs(area_pos[0] - coords[0]) < 0.1 and abs(area_pos[1] - coords[1]) < 0.1:
                if area_name == "A":
                    area_code = 0
                elif area_name == "B":
                    area_code = 1
                elif area_name == "C":
                    area_code = 2
                break
        
        # 发布目标区域
        self.target_area_pub.publish(Int32(data=area_code))
        
        # 立即更新状态，确保颜色在任务开始时就显示
        self._update_foxglove()
        self._select_reachable_block()
    
    # 导航状态回调
    @staticmethod
    def _nav_status_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        status = msg.data.strip()
        if status == "succeeded":
            if self.current_step == "NAV_TO_BLOCK":
                self.get_logger().info("到达物块位置，准备抓取")
                self.current_step = "GRASP"
                self.trigger_grasp()
            elif self.current_step == "NAV_TO_AREA":
                if self.area_nav_target_index + 1 < len(self.area_nav_targets):
                    self.area_nav_target_index += 1
                    self.current_area_nav_retry = 0
                    self._send_current_area_target()
                else:
                    self.get_logger().info("到达目标区域，准备放置")
                    self.current_step = "PLACE"
                    self.trigger_place()
        elif status.startswith("failed:"):
            if self.current_step == "NAV_TO_BLOCK":
                self.attempted_candidate_indices.add(self.current_grasp_candidate_index)
                candidates = self.get_grasp_candidates(self.current_assignment["block_pos"])
                remaining = [i for i in range(len(candidates)) if i not in self.attempted_candidate_indices]
                if remaining:
                    self.current_grasp_candidate_index = remaining[0]
                    self.get_logger().warn(
                        f"抓取点不可达（{status}），尝试第{remaining[0] + 1}个接近方向"
                    )
                    self.navigate_to_block()
                else:
                    failed_idx = self.current_assignment["block_idx"]
                    self.failed_blocks_for_step.add(failed_idx)
                    self.get_logger().warn(
                        f"物块{self.selected_block[2]}的四个接近方向均不可达，重新选择同色物块"
                    )
                    self._select_reachable_block()
            elif self.current_step == "NAV_TO_AREA":
                self.current_area_nav_retry += 1
                if self.current_area_nav_retry <= 1:
                    self.get_logger().warn(f"区域导航失败（{status}），重试一次")
                    self._send_current_area_target()
                else:
                    self._stop_failed_task(f"目标区域导航失败：{status}")
        elif status in ("emergency_stop", "paused"):
            self._stop_failed_task(f"导航已停止：{status}")

    def _retry_area_navigation(self):
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer.destroy()
            self.retry_timer = None
        if self.current_step == "NAV_TO_AREA" and self.current_assignment:
            self._send_current_area_target()

    def _stop_failed_task(self, reason):
        """保留未完成货物状态，停止后续任务，等待人工重新下达命令。"""
        self.get_logger().error(reason)
        self.selection_id += 1
        self._clean_timers()
        self.current_task = None
        self.current_assignment = None
        self.optimized_tasks = []
        self.task_queue = []
        self.current_step = "WAIT_TASK"
        self.number_pick_pub.publish(Int32(data=0))
        self.target_area_pub.publish(Int32(data=9))
        self._update_foxglove()
    
    def _nav_status_callback(self, msg):
        self_ref = weakref.ref(self)
        self._nav_status_callback_impl(self_ref, msg)
        del self_ref
    
    # 机械臂状态回调
    @staticmethod
    def _arm_status_callback_impl(self_ref, msg):
        self = self_ref()
        if not self:
            return
        status = msg.data.strip()
        if status == "grasp_succeeded" and self.current_step == "GRASP":
            self.grasp_confirmed = True
            self.get_logger().info("机械臂抓取成功，立即前往目标区域")
            self._check_grasp(self_ref)
        elif status == "place_succeeded" and self.current_step == "PLACE":
            self.place_confirmed = True
            self.get_logger().info("机械臂放置成功，立即执行下一件")
            self._check_place(self_ref)
        elif status == "grasp_failed" and self.current_step == "GRASP":
            self.get_logger().warn("机械臂抓取失败，立即重试")
            self._check_grasp(self_ref)
        elif status.startswith("place_reposition:") and self.current_step == "PLACE":
            if self.place_timer:
                self.place_timer.cancel()
                self.place_timer.destroy()
                self.place_timer = None
            try:
                _, dx_text, dy_text = status.split(":")
                dx, dy = float(dx_text), float(dy_text)
                if not all(math.isfinite(v) for v in (dx, dy)):
                    raise ValueError("non-finite cube offset")
                if self.current_place_retry >= self.max_place_retry:
                    self._stop_failed_task("物块放置位置修正三次仍失败")
                    return
                x, y, yaw = self.area_nav_targets[-1]
                # Replan around obstacles with Nav2; never drive the base
                # directly or spin it beside the placement board.
                corrected = (x + max(-0.35, min(0.35, dx)),
                             y + max(-0.35, min(0.35, dy)), yaw)
                self.area_nav_targets[-1] = corrected
                self.area_nav_target_index = len(self.area_nav_targets) - 1
                self.current_area_nav_retry = 0
                self.get_logger().warn(f"物块偏离放置区，重新导航到 {corrected}")
                self._send_current_area_target()
            except (ValueError, IndexError) as exc:
                self._stop_failed_task(f"放置修正数据无效: {exc}")
        elif status == "place_lost" and self.current_step == "PLACE":
            self._stop_failed_task("物块分离后未落在目标区域，停止以避免误报完成")
        elif status == "place_failed" and self.current_step == "PLACE":
            self.get_logger().warn("机械臂放置失败，立即重试")
            self._check_place(self_ref)
    
    def _arm_status_callback(self, msg):
        self_ref = weakref.ref(self)
        self._arm_status_callback_impl(self_ref, msg)
        del self_ref
    
    # 导航到物块（使用优化路径）
    def navigate_to_block(self):
        self._clean_timers()
        if not self.current_assignment:
            return
        self.grasp_confirmed = False
        self.current_grasp_retry = 0
        self.closest_cache = None
        # 使用优化后的抓取位置
        grasp_pos = self.get_grasp_candidates(
            self.current_assignment["block_pos"]
        )[self.current_grasp_candidate_index]
        cube_name = self.selected_block[2]
        # 发布导航目标
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": grasp_pos[0], "y": grasp_pos[1], "yaw": grasp_pos[2]})
        self.nav_target_pub.publish(nav_msg)
        self.target_cube_pub.publish(String(data=cube_name))
        self.current_step = "NAV_TO_BLOCK"
        self._update_foxglove()
        self.get_logger().info(f"导航到物块：{cube_name}（抓取位置：{grasp_pos[0]:.2f}, {grasp_pos[1]:.2f}）")
    
    # 触发抓取
    def trigger_grasp(self):
        if self.current_grasp_retry >= self.max_grasp_retry:
            self.get_logger().error(f"抓取重试达{self.max_grasp_retry}次，换同色物块")
            self.failed_blocks_for_step.add(self.current_assignment["block_idx"])
            self.current_grasp_retry = 0
            self._select_reachable_block()
            return
        self.get_logger().info(f"触发抓取（重试次数：{self.current_grasp_retry}/{self.max_grasp_retry}）")
        self_ref = weakref.ref(self)
        self.grasp_timer = self.create_timer(self.GRASP_TIMEOUT, lambda: self._check_grasp(self_ref))
        self.arm_cargo_pub.publish(String(data="arrived_at_cargo"))
    
    # 检查抓取结果
    @staticmethod
    def _check_grasp(self_ref):
        self = self_ref()
        if not self:
            return
        # 清理定时器
        if self.grasp_timer:
            self.grasp_timer.cancel()
            self.grasp_timer.destroy()
            self.grasp_timer = None
        if self.grasp_confirmed:
            # 标记物块为已抓取
            block_idx = self.current_assignment["block_idx"]
            color = self.current_task["color"]
            blocks = self.RED_BLOCKS if color == "red" else self.BLUE_BLOCKS
            x, y, _ = blocks[block_idx]
            blocks[block_idx] = (x, y, True)
            self.get_logger().info(f"物块标记为已抓取：{color}_cube_{block_idx + 1}")
            # 抓取成功，导航到目标区域
            self.navigate_to_area()
        else:
            # 抓取失败，重试
            self.current_grasp_retry += 1
            self.get_logger().warn(f"抓取超时/失败，准备重试（{self.current_grasp_retry}/{self.max_grasp_retry}）")
            self.trigger_grasp()
    
    # 导航到目标区域
    def navigate_to_area(self):
        self._clean_timers()
        if not self.current_assignment:
            return
        self.place_confirmed = False
        self.current_place_retry = 0
        area_pos = self.current_assignment["area_pos"]
        self.area_nav_targets = [(area_pos[0], area_pos[1], 0.0)]
        # The B approach from blue_cube_5 crosses a narrow doorway.
        if (self.stable_layout_hints and self.current_task["to"] == "B" and self.selected_block
                and self.selected_block[2] == "blue_cube_5"):
            self.area_nav_targets.insert(0, (-1.8, -2.8, -math.pi / 2))
        self.area_nav_target_index = 0
        self.current_area_nav_retry = 0
        self._send_current_area_target()

    def _send_current_area_target(self):
        if not self.current_assignment or not self.area_nav_targets:
            return
        x, y, yaw = self.area_nav_targets[self.area_nav_target_index]
        nav_msg = String()
        nav_msg.data = json.dumps({"type": "custom", "x": x, "y": y, "yaw": yaw})
        self.nav_target_pub.publish(nav_msg)
        self.current_step = "NAV_TO_AREA"
        self._update_foxglove()
        if self.area_nav_target_index + 1 < len(self.area_nav_targets):
            self.get_logger().info(f"导航至门前引导点：({x:.2f}, {y:.2f})")
        else:
            self.get_logger().info(
                f"导航到目标区域 {self.current_task['to']}：({x:.2f}, {y:.2f})"
            )

    # 触发放置
    def trigger_place(self):
        if self.current_place_retry >= self.max_place_retry:
            self._stop_failed_task("放置连续失败，停止后续任务并检查货物状态")
            return
        self.current_place_retry += 1
        self.get_logger().info("触发放置")
        self.place_confirmed = False
        self_ref = weakref.ref(self)
        self.place_timer = self.create_timer(self.PLACE_TIMEOUT, lambda: self._check_place(self_ref))
        self.arm_area_pub.publish(String(data="arrived_at_area"))
        
        # 发布状态4：正在放置
        self.foxglove_pubs["cur"].publish(Int32(data=4))
        
    
    # 检查放置结果
    @staticmethod
    def _check_place(self_ref):
        self = self_ref()
        if not self:
            return
        # 清理定时器
        if self.place_timer:
            self.place_timer.cancel()
            self.place_timer.destroy()
            self.place_timer = None
        if self.place_confirmed:
            self.completed_num += 1
            original_num = self.current_task.get("original_num", 1)
            current_index = self.current_task.get("current_index", 1)
            
            self.get_logger().info(f"放置完成（{current_index}/{original_num}）")
            
            # 直接处理下一个优化任务
            self._process_next_optimized_task()
        else:
            # 重试放置
            self.get_logger().warn("放置超时/失败，准备重试")
            self.trigger_place()
        
    
    # 清理定时器
    def _clean_timers(self):
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer.destroy()
            self.retry_timer = None
        if self.grasp_timer:
            self.grasp_timer.cancel()
            self.grasp_timer.destroy()
            self.grasp_timer = None
        if self.place_timer:
            self.place_timer.cancel()
            self.place_timer.destroy()
            self.place_timer = None
    
    # 更新Foxglove状态
    def _update_foxglove(self):
        """更新Foxglove显示的状态信息"""
        # 根据当前步骤设置状态值
        step_to_status = {
            "WAIT_TASK": 0,      # 等待任务
            "SELECT_BLOCK": 1,   # 检查可达物块
            "NAV_TO_BLOCK": 1,   # 导航到物块
            "GRASP": 2,          # 抓取中
            "NAV_TO_AREA": 3,    # 导航到区域
            "PLACE": 4           # 放置中
        }
        
        # 发布当前状态
        status = step_to_status.get(self.current_step, 0)
        self.foxglove_pubs["cur"].publish(Int32(data=status))
        
        # 根据任务设置颜色信息
        if self.current_task:
            color = self.current_task["color"]
            # 按照原始配置：蓝色=0，红色=1
            color_code = 0 if color == "blue" else 1
            self.foxglove_pubs["color"].publish(Int32(data=color_code))
            self.foxglove_pubs["ask"].publish(Int32(data=color_code))  # ask与color完全相同
            
            # 根据当前步骤设置pick状态
            if self.current_step in ["WAIT_TASK", "SELECT_BLOCK", "NAV_TO_BLOCK"]:
                pick_code = -1
            elif self.current_step in ["GRASP", "NAV_TO_AREA"]:
                pick_code = 0
            else:  # PLACE
                pick_code = 1
            self.foxglove_pubs["pick"].publish(Int32(data=pick_code))
        else:
            # 无任务时
            self.foxglove_pubs["color"].publish(Int32(data=-1))
            self.foxglove_pubs["ask"].publish(Int32(data=-1))
            self.foxglove_pubs["pick"].publish(Int32(data=-1))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MainControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
