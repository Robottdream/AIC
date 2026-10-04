import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_msgs.srv import GetEntityState, SetEntityState

class ArmGrabPlaceNode(Node):
    def __init__(self):
        super().__init__("arm_grab_place_node")
        
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
        self.latest_odom_yaw = None
        self.latest_odom_at = 0.0
        self.align_timer = None
        self.align_deadline = 0.0
        self.align_stable_ticks = 0
        self.align_tick_count = 0
        self.base_cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.detach_attempts = 0
        self.target_area = 9
        self.create_subscription(Int32, "/target_area", self._on_target_area, 10)

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
            dx, dy = center_x - position.x, center_y - position.y
            # The cube half-width is 0.015 m. Keep another 0.03 m margin
            # so small residual base motion cannot push it beyond the board.
            if abs(dx) <= 0.535 and abs(dy) <= 0.315:
                self.get_logger().info(
                    f"放置前实测 {cube}: ({position.x:.3f},{position.y:.3f}), "
                    f"接近目标区域，分离后进行有限落点修正")
                self.send_detach_action(self._after_detach)
                return
            if abs(dx) > 0.8 or abs(dy) > 0.55:
                raise RuntimeError(f"物块距放置区过远: dx={dx:.3f}, dy={dy:.3f}")
            self.get_logger().warn(
                f"放置前实测 {cube}: ({position.x:.3f},{position.y:.3f}) "
                f"不在区域内，请求底盘修正 dx={dx:.3f}, dy={dy:.3f}")
            self.is_executing = False
            self.current_step = 0
            self.arm_status_pub.publish(String(data=f"place_reposition:{dx:.4f}:{dy:.4f}"))
        except Exception as exc:
            self.get_logger().error(f"放置前位置检查失败: {exc}")
            self._reset_execution()

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
        point_target.time_from_start.sec = 1
        point_target.time_from_start.nanosec = 0
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
            # Gazebo has gravity disabled for cargo. Keep the released cube
            # fully on the 1.0 x 0.5m board if it landed just over an edge.
            cx, cy = self.ZONES[self.target_area]
            desired_x = max(cx - 0.455, min(cx + 0.455, state.pose.position.x))
            desired_y = max(cy - 0.205, min(cy + 0.205, state.pose.position.y))
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
            if abs(p.x - cx) > 0.485 or abs(p.y - cy) > 0.235 or not 0.02 <= p.z <= 0.055:
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
            self.current_step = 1
            self.place_arm_ready = False
            self.place_gripper_ready = False
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
