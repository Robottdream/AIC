import rclpy
import math
import time
import threading
import copy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_msgs.srv import GetEntityState, SetEntityState

class ArmGrabPlaceNode(Node):
    def __init__(self):
        super().__init__("arm_grab_place_node")
        # A/B/C 的真实放置区域中心按 [Ax, Ay, Bx, By, Cx, Cy] 配置。
        zone_centers = self.declare_parameter(
            "zone_centers",
            [value for zone in (0, 1, 2) for value in self.ZONES[zone]],
        ).value
        if len(zone_centers) != 6 or not all(
                math.isfinite(value) for value in zone_centers):
            raise ValueError("zone_centers must contain 6 finite coordinates")
        self.ZONES = {
            zone: (zone_centers[2 * zone], zone_centers[2 * zone + 1])
            for zone in (0, 1, 2)
        }
        zone_size = self.declare_parameter("zone_size", [1.0, 0.5]).value
        if len(zone_size) != 2 or not all(math.isfinite(v) and v >= 0.2 for v in zone_size):
            raise ValueError("zone_size must contain two finite dimensions >= 0.2m")
        self.zone_half_extents = tuple(v/2 for v in zone_size)
        
        # ===================== 1. 新增：初始化状态发布器（给主控发确认信号）=====================
        self.arm_status_pub = self.create_publisher(String, "/arm_status", 10)
        self.get_logger().info("已初始化/arm_status发布器，用于发送抓取/放置确认")

        # ===================== 2. 原有动作/服务客户端初始化（保留不变）=====================
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.arm_action_client.wait_for_server()
        self.get_logger().info("机械臂动作服务就绪！")

        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )
        self.gripper_action_client.wait_for_server()
        self.get_logger().info("夹爪动作服务就绪！")

        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/ATTACHLINK 服务未就绪，继续等待...")
        self.get_logger().info("吸附服务就绪！")

        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/DETACHLINK 服务未就绪，继续等待...")
        self.get_logger().info("分离服务就绪！")

        self.get_state_client = self.create_client(GetEntityState, "/get_entity_state")
        self.set_state_client = self.create_client(SetEntityState, "/set_entity_state")

        # ===================== 3. 原有订阅（保留不变）=====================
        self.target_cube_sub = self.create_subscription(
            String, "/current_target_cube", self.target_cube_callback, 10
        )
        self.cargo_sub = self.create_subscription(
            String, "/nav_done_cargo", self.cargo_callback, 10
        )
        self.area_sub = self.create_subscription(
            String, "/nav_done_area", self.area_callback, 10
        )

        # ===================== 4. 原有参数（保留不变）=====================
        self.arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint = ["finger_joint1"]
        self.arm_trajectory = {
            "init": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "bend": [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            "lift": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }
        self.gripper_trajectory = {
            "close": [0.0, 0.1, 0.2],
            "open": [0.3, 0.2, 0.1, 0.0]
        }
        self.attach_params = {
            "model1_name": "six_arm",
            "link1_name": "link6",
            "model2_name": "red_cube_1",
            "link2_name": "link"
        }

        # ===================== 5. 状态控制（保留不变）=====================
        self.is_executing = False
        self.current_joint_pos = None
        self.current_step = 0
        self.current_task = None  # "grab" / "place"
        self.latest_yaw = None
        self.latest_yaw_at = 0.0
        self.latest_angular_speed = 0.0
        self.latest_linear_speed = 0.0
        self.settle_timer = None
        self.settle_deadline = 0.0
        self.settle_ticks = 0
        self.place_release_lock = threading.Lock()
        self.place_arm_ready = False
        self.place_gripper_ready = False
        self.validated_release_pose = None
        self.latest_odom_yaw = None
        self.latest_odom_at = 0.0
        self.align_timer = None
        self.align_deadline = 0.0
        self.align_stable_ticks = 0
        self.align_tick_count = 0
        self.base_cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.static_map = None
        self.place_yaw_adjustments = 0
        self.create_subscription(OccupancyGrid, "/map", self._on_static_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.detach_attempts = 0
        self.target_area = 9
        self.create_subscription(Int32, "/target_area", self._on_target_area, 10)

    def _on_static_map(self, msg):
        self.static_map = msg

    def _on_target_area(self, msg):
        self.target_area = msg.data

    def _on_amcl_pose(self, msg):
        q = msg.pose.pose.orientation
        self.latest_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_yaw_at = time.monotonic()

    def _on_odom(self, msg):
        self.latest_angular_speed = msg.twist.twist.angular.z
        self.latest_linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        q = msg.pose.pose.orientation
        self.latest_odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_odom_at = time.monotonic()

    # The carried cube is not at a fixed offset from the chassis: attachment
    # geometry depends on where it was grasped. Measure its world pose before
    # release instead of rotating the chassis blindly to yaw=0.
    ZONES = {
        0: (3.143086, -5.807858),
        1: (-1.196544, -6.485499),
        2: (-6.873777, -7.785160),
    }

    def _stop_alignment(self):
        self.base_cmd_pub.publish(Twist())
        if self.align_timer is not None:
            self.align_timer.cancel()
            self.destroy_timer(self.align_timer)
            self.align_timer = None
        if self.settle_timer is not None:
            self.settle_timer.cancel()
            self.destroy_timer(self.settle_timer)
            self.settle_timer = None

    def _settle_before_place(self):
        # Nav2's success is based on AMCL pose. At higher speed the physical
        # chassis can still be rotating; bending the arm then sweeps the cube
        # across the board. Require measured chassis motion to stop first.
        self.settle_deadline = time.monotonic() + 5.0
        self.settle_ticks = 0
        self.settle_timer = self.create_timer(0.1, self._settle_tick)

    def _settle_tick(self):
        if not self.is_executing or self.current_task != "place":
            self._stop_alignment()
            return
        self.base_cmd_pub.publish(Twist())
        now = time.monotonic()
        fresh = now - self.latest_odom_at < 0.5
        stationary = (fresh and self.latest_linear_speed < 0.045
                      and abs(self.latest_angular_speed) < 0.055)
        self.settle_ticks = self.settle_ticks + 1 if stationary else 0
        if self.settle_ticks >= 4:
            self.settle_timer.cancel()
            self.destroy_timer(self.settle_timer)
            self.settle_timer = None
            self.get_logger().info("底盘实测停稳，开始放置")
            self.send_arm_action(self.arm_trajectory["bend"], self._after_place_bend)
        elif now >= self.settle_deadline:
            self.get_logger().error(
                f"放置前底盘未停稳: v={self.latest_linear_speed:.3f}, "
                f"w={self.latest_angular_speed:.3f}")
            self._reset_execution()

    def _check_cube_before_release(self):
        if self.target_area not in self.ZONES or not self.get_state_client.service_is_ready():
            self.get_logger().error("无法确认目标区域或 Gazebo 物块位置")
            self._reset_execution()
            return
        cube = self.attach_params["model2_name"]
        request = GetEntityState.Request()
        request.name = cube
        request.reference_frame = "world"
        future = self.get_state_client.call_async(request)
        future.add_done_callback(lambda result: self._on_cube_before_release(result, cube))

    def _on_cube_before_release(self, future, cube):
        if not self.is_executing or self.current_task != "place" or self.current_step != 3:
            return
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.status_message)
            position = response.state.pose.position
            center_x, center_y = self.ZONES[self.target_area]
            half_x, half_y = self.zone_half_extents
            dx, dy = center_x - position.x, center_y - position.y
            # The cube half-width is 0.015 m. Keep another 0.03 m margin
            # so small residual base motion cannot push it beyond the board.
            if abs(dx) <= half_x + 0.035 and abs(dy) <= half_y + 0.065:
                self.validated_release_pose = (cube, copy.deepcopy(response.state.pose), time.monotonic())
                self.get_logger().info(
                    f"放置前实测 {cube}: ({position.x:.3f},{position.y:.3f}), "
                    f"接近目标区域，分离后进行有限落点修正")
                self.send_detach_action(self._after_detach)
                return
            if abs(dx) > half_x + 0.8 or abs(dy) > half_y + 0.8:
                raise RuntimeError(f"物块距放置区过远: dx={dx:.3f}, dy={dy:.3f}")
            # Keep the chassis parked. Try a bounded joint1 adjustment using
            # the measured cube radius, not an assumed grasp offset.
            request = GetEntityState.Request()
            request.name = "six_arm"
            request.reference_frame = "world"
            future = self.get_state_client.call_async(request)
            future.add_done_callback(lambda result: self._adjust_place_yaw(
                result, cube, copy.deepcopy(position), dx, dy))
        except Exception as exc:
            self.get_logger().error(f"放置前位置检查失败: {exc}")
            self._reset_execution()

    @staticmethod
    def _placement_yaw_candidates(cube_xy, pivot_xy, center, half_extents, current_yaw):
        """Bounded yaw candidates whose circular arc ends inside the board."""
        values = (*cube_xy, *pivot_xy, *center, *half_extents, current_yaw)
        if not all(math.isfinite(v) for v in values):
            return []
        vx, vy = cube_xy[0] - pivot_xy[0], cube_xy[1] - pivot_xy[1]
        if not 0.1 <= math.hypot(vx, vy) <= 1.1:
            return []
        candidates = []
        for step in range(-180, 181):
            joint_yaw = step * 0.005  # +/-0.9 rad; URDF allows +/-3.14.
            delta = joint_yaw - current_yaw
            c, sin = math.cos(delta), math.sin(delta)
            x = pivot_xy[0] + c*vx - sin*vy
            y = pivot_xy[1] + sin*vx + c*vy
            if (abs(x-center[0]) <= half_extents[0]-0.08
                    and abs(y-center[1]) <= half_extents[1]-0.08):
                candidates.append((abs(delta), joint_yaw))
        return [yaw for _, yaw in sorted(candidates)]

    def _arm_sweep_clear(self, cube_xy, pivot_xy, delta):
        """Check the extended arm/cargo sweep against the static map."""
        grid = self.static_map
        if grid is None or grid.info.resolution <= 0:
            return False
        origin = grid.info.origin
        q = origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c0, s0 = math.cos(yaw), math.sin(yaw)
        vx, vy = cube_xy[0]-pivot_xy[0], cube_xy[1]-pivot_xy[1]
        radius = math.hypot(vx, vy)
        angle_steps = max(1, int(math.ceil(abs(delta)*radius/0.035)))
        radial_steps = max(1, int(math.ceil(radius/0.035)))
        for i in range(angle_steps+1):
            angle = delta*i/angle_steps
            c, sin = math.cos(angle), math.sin(angle)
            for j in range(1, radial_steps+1):
                fraction = j/radial_steps
                if fraction*radius < 0.32:
                    continue  # Within the already parked chassis footprint.
                x = pivot_xy[0]+fraction*(c*vx-sin*vy)
                y = pivot_xy[1]+fraction*(sin*vx+c*vy)
                for ox, oy in ((0,0), (.055,0), (-.055,0), (0,.055), (0,-.055)):
                    wx, wy = x+ox-origin.position.x, y+oy-origin.position.y
                    gx = int(math.floor((c0*wx+s0*wy)/grid.info.resolution))
                    gy = int(math.floor((-s0*wx+c0*wy)/grid.info.resolution))
                    if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
                        return False
                    value = grid.data[gy*grid.info.width+gx]
                    if value < 0 or value >= 65:
                        return False
        return True

    def _adjust_place_yaw(self, future, cube, position, dx, dy):
        if (not self.is_executing or self.current_task != "place"
                or self.current_step != 3 or cube != self.attach_params["model2_name"]):
            return
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError("无法读取机械臂转轴位置")
            # arm_base_link has zero XY offset from the model origin in this URDF.
            pivot = response.state.pose.position
            cube_xy, pivot_xy = (position.x, position.y), (pivot.x, pivot.y)
            current_yaw = self.current_joint_pos[0]
            options = self._placement_yaw_candidates(cube_xy, pivot_xy,
                self.ZONES[self.target_area], self.zone_half_extents, current_yaw)
            if self.place_yaw_adjustments < 2:
                for yaw in options:
                    if not self._arm_sweep_clear(cube_xy, pivot_xy, yaw-current_yaw):
                        continue
                    self.place_yaw_adjustments += 1
                    target = list(self.current_joint_pos)
                    target[0] = yaw
                    self.get_logger().info(
                        f"底盘保持停车，机械臂落点微调 joint1={yaw:.3f}rad，随后复核物块位置")
                    if self.align_timer is None:
                        self.align_timer = self.create_timer(0.1,
                            lambda: self.base_cmd_pub.publish(Twist()))
                    self.send_arm_action(target, self._check_cube_before_release)
                    return
            self._prepare_place_reposition(dx, dy)
        except Exception as exc:
            self.get_logger().error(f"放置姿态计算失败: {exc}")
            self._reset_execution()

    def _prepare_place_reposition(self, dx, dy):
        # Do not navigate with the arm extended and the gripper open.
        # Wait for both stow actions before notifying the task controller.
        self._stop_alignment()
        self.current_step = 6
        self.reposition_actions_remaining = 2
        def ready():
            with self.place_release_lock:
                if (not self.is_executing or self.current_task != "place"
                        or self.current_step != 6):
                    return
                self.reposition_actions_remaining -= 1
                if self.reposition_actions_remaining:
                    return
                self.is_executing = False
                self.current_step = 0
            self.get_logger().warn(
                f"已收臂并闭合夹爪，请求底盘位置修正 dx={dx:.3f}, dy={dy:.3f}")
            self.arm_status_pub.publish(String(data=f"place_reposition:{dx:.4f}:{dy:.4f}"))
        self.send_gripper_action("close", ready)
        self.send_arm_action(self.arm_trajectory["lift"], ready)

    # ===================== 6. 原有目标物块更新（保留不变）=====================
    def target_cube_callback(self, msg):
        new_cube_name = msg.data.strip()
        if "_cube_" in new_cube_name and (new_cube_name.startswith("red_") or new_cube_name.startswith("blue_")):
            self.attach_params["model2_name"] = new_cube_name
            self.get_logger().info(f"已更新目标物块：{new_cube_name}")
        else:
            self.get_logger().warn(f"无效物块名称：{new_cube_name}，保持当前物块：{self.attach_params['model2_name']}")

    # ===================== 7. 原有机械臂/夹爪动作（保留不变）=====================
    def send_arm_action(self, target_joint_positions, step_done_callback):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joints
        point_start = JointTrajectoryPoint()
        point_start.positions = self.current_joint_pos if self.current_joint_pos else self.arm_trajectory["init"]
        point_start.time_from_start.sec = 0
        point_start.time_from_start.nanosec = 500
        point_target = JointTrajectoryPoint()
        point_target.positions = target_joint_positions
        # Limit the base joint's sweep speed, including return-to-stow motions.
        duration = max(1.0, abs(target_joint_positions[0]-point_start.positions[0])/0.55)
        point_target.time_from_start.sec = int(duration)
        point_target.time_from_start.nanosec = int((duration-int(duration))*1_000_000_000)
        trajectory.points = [point_start, point_target]
        goal_msg.trajectory = trajectory
        self.current_joint_pos = target_joint_positions
        future = self.arm_action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._arm_goal_done_cb(f, step_done_callback)
        )

    def _arm_goal_done_cb(self, future, step_done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("机械臂动作请求被拒绝！")
                self._reset_execution()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda r: self._arm_result_done_cb(r, step_done_callback)
            )
        except Exception as e:
            self.get_logger().error(f"机械臂请求回调异常：{str(e)}")
            self._reset_execution()

    def _arm_result_done_cb(self, future, step_done_callback):
        try:
            result = future.result().result
            if result.error_code == 0:
                self.get_logger().info("机械臂动作执行完成！")
                step_done_callback()
            else:
                self.get_logger().error(f"机械臂动作失败，错误码：{result.error_code}")
                self._reset_execution()
        except Exception as e:
            self.get_logger().error(f"机械臂结果回调异常：{str(e)}")
            self._reset_execution()

    def send_gripper_action(self, action_type, step_done_callback):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint
        # 夹爪只负责展示开合，抓取由吸附服务确认。
        time_steps = [0.0, 0.2, 0.4] if action_type == "close" else [0.0, 0.13, 0.27, 0.4]

        target_positions = self.gripper_trajectory[action_type]
        points = []
        for i, pos in enumerate(target_positions):
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start.sec = int(time_steps[i])
            point.time_from_start.nanosec = int((time_steps[i] % 1.0) * 1_000_000_000) + 200
            points.append(point)
        trajectory.points = points
        goal_msg.trajectory = trajectory
        future = self.gripper_action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._gripper_goal_done_cb(f, step_done_callback)
        )

    def _gripper_goal_done_cb(self, future, step_done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("夹爪动作请求被拒绝！")
                self._reset_execution()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda r: self._gripper_result_done_cb(r, step_done_callback)
            )
        except Exception as e:
            self.get_logger().error(f"夹爪请求回调异常：{str(e)}")
            self._reset_execution()

    def _gripper_result_done_cb(self, future, step_done_callback):
        try:
            result = future.result().result
            if result.error_code == 0:
                self.get_logger().info("夹爪动作执行完成！")
                #time.sleep(1.0)
                step_done_callback()
            else:
                self.get_logger().error(f"夹爪动作失败，错误码：{result.error_code}")
                self._reset_execution()
        except Exception as e:
            self.get_logger().error(f"夹爪结果回调异常：{str(e)}")
            self._reset_execution()

    # ===================== 8. 修复：吸附/分离服务结果判断（移除success字段判断）=====================
    def send_attach_action(self, step_done_callback):
        self.get_logger().info(f"执行吸附动作：机械臂 → 物块（{self.attach_params['model2_name']}）")
        req = AttachLink.Request()
        req.model1_name = self.attach_params["model1_name"]
        req.link1_name = self.attach_params["link1_name"]
        req.model2_name = self.attach_params["model2_name"]
        req.link2_name = self.attach_params["link2_name"]
        future = self.attach_client.call_async(req)
        future.add_done_callback(
            lambda f: self._attach_done_cb(f, step_done_callback)
        )

    def _attach_done_cb(self, future, step_done_callback):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(f"物块{self.attach_params['model2_name']}吸附成功！")
            step_done_callback()
        except Exception as e:
            self.get_logger().error(f"吸附动作失败：{str(e)}")
            self._reset_execution()

    def send_detach_action(self, step_done_callback):
        self.detach_attempts += 1
        self.get_logger().info(f"执行分离动作：机械臂 → 物块（{self.attach_params['model2_name']}）")
        req = DetachLink.Request()
        req.model1_name = self.attach_params["model1_name"]
        req.link1_name = self.attach_params["link1_name"]
        req.model2_name = self.attach_params["model2_name"]
        req.link2_name = self.attach_params["link2_name"]
        future = self.detach_client.call_async(req)
        future.add_done_callback(
            lambda f: self._detach_done_cb(f, step_done_callback)
        )

    def _detach_done_cb(self, future, step_done_callback):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(f"物块{self.attach_params['model2_name']}分离成功！")
            self._lower_released_cube(step_done_callback)
        except Exception as e:
            self.get_logger().error(f"分离动作失败：{str(e)}")
            if self.detach_attempts < 3:
                self.send_detach_action(step_done_callback)
            else:
                self._reset_execution()

    def _lower_released_cube(self, step_done_callback):
        # Pickup cubes float at z=0.75 with gravity disabled. Only after a
        # successful detach, put the released cube on the 0.02 m zone surface.
        if not self.get_state_client.service_is_ready() or not self.set_state_client.service_is_ready():
            self.get_logger().warn("Gazebo 位姿服务不可用，保持原有放置行为")
            step_done_callback()
            return
        cube = self.attach_params["model2_name"]
        request = GetEntityState.Request()
        request.name = cube
        request.reference_frame = "world"
        future = self.get_state_client.call_async(request)
        future.add_done_callback(lambda result: self._on_released_cube_pose(result, cube, step_done_callback))

    def _on_released_cube_pose(self, future, cube, step_done_callback):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(f"读取 {cube} 位姿失败")
            state = response.state
            state.name = cube
            state.reference_frame = "world"
            # Removing Gazebo's attachment can impart a constraint impulse.
            # Cargo has gravity disabled, so preserve the measured release pose
            # when placing it on the board instead of sampling that impulse.
            release = self.validated_release_pose
            if release is None or release[0] != cube or time.monotonic() - release[2] > 2.0:
                raise RuntimeError("缺少新鲜的分离前物块位姿")
            state.pose = copy.deepcopy(release[1])
            # Gazebo has gravity disabled for cargo. Keep the released cube
            # fully on the 1.0 x 0.5m board if it landed just over an edge.
            cx, cy = self.ZONES[self.target_area]
            half_x, half_y = getattr(self, 'zone_half_extents', (0.5, 0.25))
            desired_x = max(cx - half_x + 0.045, min(cx + half_x - 0.045, state.pose.position.x))
            desired_y = max(cy - half_y + 0.045, min(cy + half_y - 0.045, state.pose.position.y))
            correction = math.hypot(desired_x - state.pose.position.x,
                                    desired_y - state.pose.position.y)
            if correction > 0.11:
                raise RuntimeError(f"释放后偏离放置板过远，拒绝瞬移: {correction:.3f}m")
            if correction > 0.001:
                self.get_logger().warn(f"物块紧贴板边，仿真落点修正 {correction:.3f}m")
            state.pose.position.x = desired_x
            state.pose.position.y = desired_y
            state.pose.position.z = 0.035
            state.twist.linear.x = state.twist.linear.y = state.twist.linear.z = 0.0
            state.twist.angular.x = state.twist.angular.y = state.twist.angular.z = 0.0
            request = SetEntityState.Request()
            request.state = state
            result = self.set_state_client.call_async(request)
            result.add_done_callback(lambda answer: self._on_cube_lowered(answer, cube, step_done_callback))
        except Exception as exc:
            self.get_logger().error(f"物块落地失败：{exc}")
            self.is_executing = False
            self.arm_status_pub.publish(String(data="place_lost"))

    def _on_cube_lowered(self, future, cube, step_done_callback):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(f"设置 {cube} 地面位姿失败")
            self.get_logger().info(f"物块{cube}已放在区域板上")
            step_done_callback()
        except Exception as exc:
            self.get_logger().error(f"物块落地失败：{exc}")
            self.is_executing = False
            self.arm_status_pub.publish(String(data="place_lost"))

    # ===================== 9. 修复：抓取/放置完成后发布状态给主控=====================
    def _grab_proceed(self):
        if not self.is_executing:
            return
            
        if self.current_step == 1:
            self.get_logger().info("步骤2：夹爪闭合...")
            self.current_step = 2
            self.send_gripper_action("close", self._after_gripper_close)
        elif self.current_step == 2:
            self.get_logger().info("步骤3：执行物块吸附...")
            self.current_step = 3
            self.send_attach_action(self._after_attach)
        elif self.current_step == 3:
            self.get_logger().info("步骤4：机械臂举高（回到初始位置）...")
            self.current_step = 4
            self.send_arm_action(self.arm_trajectory["lift"], self._after_arm_lift)
            # The cube is attached. Start driving while the arm retracts.
            self.arm_status_pub.publish(String(data="grasp_succeeded"))
        elif self.current_step == 4:
            self.get_logger().info(f"抓取流程全部完成！已吸附物块：{self.attach_params['model2_name']}")
            self.current_step = 5
            self.is_executing = False

    def _after_gripper_close(self):
        self._grab_proceed()

    def _after_attach(self):
        self._grab_proceed()

    def _after_arm_lift(self):
        self._grab_proceed()

    def _place_proceed(self):
        if not self.is_executing:
            return
            
        if self.current_step == 3:
            self.get_logger().info("步骤4：机械臂举高（回到初始位置）...")
            self.current_step = 4
            self.send_arm_action(self.arm_trajectory["lift"], self._after_arm_lift_place)
        elif self.current_step == 4:
            # Check the final settled pose, not only the pose while attached.
            # The cube can be pushed by the robot after the link is detached.
            self.current_step = 5
            request = GetEntityState.Request()
            request.name = self.attach_params["model2_name"]
            request.reference_frame = "world"
            future = self.get_state_client.call_async(request)
            future.add_done_callback(self._verify_released_cube)

    def _verify_released_cube(self, future):
        if not self.is_executing or self.current_task != "place":
            return
        try:
            result = future.result()
            if not result.success or self.target_area not in self.ZONES:
                raise RuntimeError("无法读取最终物块位置")
            p = result.state.pose.position
            cx, cy = self.ZONES[self.target_area]
            half_x, half_y = self.zone_half_extents
            if abs(p.x - cx) > half_x - 0.015 or abs(p.y - cy) > half_y - 0.015 or not 0.02 <= p.z <= 0.055:
                raise RuntimeError(f"物块最终位置不在区域内: ({p.x:.3f},{p.y:.3f},{p.z:.3f})")
            self.get_logger().info(f"放置完成并确认区域内: ({p.x:.3f},{p.y:.3f},{p.z:.3f})")
            self.is_executing = False
            self.arm_status_pub.publish(String(data="place_succeeded"))
        except Exception as exc:
            self.get_logger().error(f"放置后核验失败: {exc}")
            self.is_executing = False
            self.arm_status_pub.publish(String(data="place_lost"))

    def _try_release_cube(self):
        # The attachment holds the cube until detach. Open the decorative
        # gripper in parallel, but require both actions to finish first.
        with self.place_release_lock:
            if (not self.is_executing or self.current_task != "place"
                    or self.current_step != 1 or not self.place_arm_ready
                    or not self.place_gripper_ready):
                return
            self.current_step = 3
        self.get_logger().info("机械臂已到放置姿态且夹爪已张开，检查物块位置")
        self._check_cube_before_release()

    def _after_place_bend(self):
        self.place_arm_ready = True
        self._try_release_cube()

    def _after_gripper_open(self):
        self.place_gripper_ready = True
        self._try_release_cube()

    def _after_detach(self):
        self._place_proceed()

    def _after_arm_lift_place(self):
        self._stop_alignment()
        self._place_proceed()

    def _reset_execution(self):
        self.get_logger().info("重置执行状态...")
        self._stop_alignment()
        failed_task = self.current_task
        self.is_executing = False
        self.current_step = 0
        if failed_task in ("grab", "place"):
            status = "grasp_failed" if failed_task == "grab" else "place_failed"
            self.arm_status_pub.publish(String(data=status))

    # ===================== 10. 原有回调（保留不变）=====================
    def cargo_callback(self, msg):
        if not self.is_executing:
            self.get_logger().info("="*50)
            self.get_logger().info(f"收到到达货物位置信号，准备抓取物块：{self.attach_params['model2_name']}")
            self.is_executing = True
            self.current_task = "grab"
            self.current_step = 1
            self.current_joint_pos = self.arm_trajectory["init"]
            self.get_logger().info("步骤1：机械臂弯曲到抓取位置...")
            self.send_arm_action(self.arm_trajectory["bend"], self._grab_proceed)
        else:
            self.get_logger().warn("正在执行抓取/放置流程，忽略本次信号！")

    def area_callback(self, msg):
        if not self.is_executing:
            self.get_logger().info("="*50)
            self.get_logger().info(f"收到到达放置位置信号，准备分离物块：{self.attach_params['model2_name']}")
            self.is_executing = True
            self.current_task = "place"
            self.detach_attempts = 0
            self.place_yaw_adjustments = 0
            self.current_step = 1
            self.place_arm_ready = False
            self.place_gripper_ready = False
            self.validated_release_pose = None
            self.current_joint_pos = self.arm_trajectory["init"]
            self.get_logger().info("步骤1：夹爪张开，同时等待底盘停稳后弯曲机械臂...")
            self._settle_before_place()
            self.send_gripper_action("open", self._after_gripper_open)
        else:
            self.get_logger().warn("正在执行抓取/放置流程，忽略本次信号！")

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ArmGrabPlaceNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断！")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
