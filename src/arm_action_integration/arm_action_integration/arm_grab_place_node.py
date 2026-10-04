import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
import time
import os
import math
import json
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus

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
        self.align_enabled=os.environ.get('AIC_ARM_ALIGNMENT','0')=='1'
        self.world_target=None; self.base_pose=None; self.base_received=0.
        self.yaw_position=None; self.yaw_received=0.; self.yaw_busy=False
        self.yaw_waiter=None; self.alignment_generation=0
        if self.align_enabled:
            self.yaw_client=ActionClient(self,FollowJointTrajectory,'/arm_yaw_controller/follow_joint_trajectory')
            if not self.yaw_client.wait_for_server(timeout_sec=15.):
                raise RuntimeError('Arm yaw controller unavailable')
            self.create_subscription(String,'/arm_world_target',self.world_target_callback,10)
            self.create_subscription(Odometry,'/odom',self.base_pose_callback,qos_profile_sensor_data)
            self.create_subscription(JointState,'/joint_states',self.yaw_state_callback,qos_profile_sensor_data)
            self.create_subscription(String,'/nav_status',self.alignment_nav_status,10)
            self.create_timer(.2,self.prealign)

    def alignment_nav_status(self,msg):
        if msg.data in ('paused','emergency_stop'):
            self.world_target=None; self.alignment_generation+=1
            if getattr(self,'yaw_handle',None): self.yaw_handle.cancel_goal_async()
            self.yaw_waiter=None
            if self.yaw_busy and self.is_executing: self._reset_execution()

    def world_target_callback(self,msg):
        try:
            target=json.loads(msg.data)
            if target['kind'] not in ('grab','place'): return
            if not all(math.isfinite(float(target[k])) for k in ('x','y')): return
            self.world_target=target; self.alignment_generation+=1
        except (ValueError,KeyError,TypeError):
            self.get_logger().error('Invalid arm world target')

    def base_pose_callback(self,msg):
        p=msg.pose.pose; q=p.orientation
        self.base_pose=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.base_received=time.monotonic()

    def yaw_state_callback(self,msg):
        if 'arm_yaw_joint' in msg.name:
            self.yaw_position=msg.position[msg.name.index('arm_yaw_joint')]
            self.yaw_received=time.monotonic()

    def desired_yaw(self):
        if self.world_target is None or self.base_pose is None or self.yaw_position is None:
            raise RuntimeError('Alignment target or feedback missing')
        if time.monotonic()-min(self.base_received,self.yaw_received)>.6:
            raise RuntimeError('Alignment feedback stale')
        x,y,heading=self.base_pose
        a=math.atan2(self.world_target['y']-y,self.world_target['x']-x)-heading
        return math.atan2(math.sin(a),math.cos(a))

    def prealign(self):
        # Only the lifted pose may sweep while navigating. The bend sequence
        # cannot run until the final measured yaw alignment succeeds.
        if self.is_executing or self.yaw_busy or self.world_target is None or self.base_pose is None: return
        x,y,_=self.base_pose
        if math.hypot(self.world_target['x']-x,self.world_target['y']-y)>1.1: return
        try:
            a=self.desired_yaw()
            if abs(a-self.yaw_position)>.12: self.rotate_yaw(a,None)
        except RuntimeError: pass

    def alignment_failed(self,reason):
        self.get_logger().error('回转对准失败：'+str(reason))
        self.arm_status_pub.publish(String(data='grasp_failed' if self.current_task=='grab' else 'place_failed'))
        self._reset_execution()

    def align_then_bend(self,callback):
        if not self.align_enabled:
            self.send_arm_action(self.arm_trajectory['bend'],callback); return
        if self.world_target is None or self.world_target['kind'] != self.current_task:
            self.alignment_failed('Missing matching task target'); return
        if self.yaw_busy:
            self.yaw_waiter=lambda:self.align_then_bend(callback); return
        try:
            a=self.desired_yaw()
            if abs(a-self.yaw_position)<.04:
                self.send_arm_action(self.arm_trajectory['bend'],callback)
            else:
                self.rotate_yaw(a,lambda:self.align_then_bend(callback))
        except RuntimeError as e: self.alignment_failed(e)

    def rotate_yaw(self,target,callback):
        duration=max(.35,abs(target-self.yaw_position)/1.2+.15)
        goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=['arm_yaw_joint']
        point=JointTrajectoryPoint(); point.positions=[target]
        point.time_from_start.sec=int(duration); point.time_from_start.nanosec=int((duration%1)*1e9)
        goal.trajectory.points=[point]
        self.yaw_busy=True; generation=self.alignment_generation
        self.get_logger().info(f'回转座对准 {target:.3f} rad，底盘保持朝向')
        def accepted(f):
            try:
                handle=f.result()
                if not handle.accepted: raise RuntimeError('Yaw goal rejected')
                self.yaw_handle=handle
                if generation != self.alignment_generation: handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(finished)
            except Exception as e: finish(False,e)
        def finished(f):
            try:
                result=f.result()
                finish(result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0,'Controller failure')
            except Exception as e: finish(False,e)
        def finish(ok,reason):
            self.yaw_busy=False; self.yaw_handle=None
            waiter=self.yaw_waiter; self.yaw_waiter=None
            if generation != self.alignment_generation:
                if waiter: waiter()
                return
            if not ok:
                if callback or waiter: self.alignment_failed(reason)
                else: self.get_logger().error(str(reason))
                return
            if waiter: waiter()
            elif callback: callback()
        self.yaw_client.send_goal_async(goal).add_done_callback(accepted)

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
        point_target.time_from_start.sec = 0
        point_target.time_from_start.nanosec = 600_000_000
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
                time.sleep(0.05)  # 动作稳定延迟
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
            time.sleep(0.05)
            step_done_callback()
        except Exception as e:
            self.get_logger().error(f"吸附动作失败：{str(e)}")
            self._reset_execution()

    def send_detach_action(self, step_done_callback):
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
            time.sleep(0.05)
            step_done_callback()
        except Exception as e:
            self.get_logger().error(f"分离动作失败：{str(e)}")
            self.send_detach_action(step_done_callback)  # 重试分离

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

        if self.current_step == 1:
            self.get_logger().info("步骤2：夹爪张开...")
            self.current_step = 2
            self.send_gripper_action("open", self._after_gripper_open)
        elif self.current_step == 2:
            self.get_logger().info("步骤3：执行物块分离...")
            self.current_step = 3
            self.send_detach_action(self._after_detach)
        elif self.current_step == 3:
            self.get_logger().info("步骤4：机械臂举高（回到初始位置）...")
            self.current_step = 4
            self.send_arm_action(self.arm_trajectory["lift"], self._after_arm_lift_place)
            # The cube is detached and stationary. Start the next route now.
            self.arm_status_pub.publish(String(data="place_succeeded"))
        elif self.current_step == 4:
            self.get_logger().info(f"放置流程全部完成！已分离物块：{self.attach_params['model2_name']}")
            self.current_step = 5
            self.is_executing = False

    def _after_gripper_open(self):
        self._place_proceed()

    def _after_detach(self):
        self._place_proceed()

    def _after_arm_lift_place(self):
        self._place_proceed()

    def _reset_execution(self):
        self.get_logger().info("重置执行状态...")
        self.is_executing = False
        self.current_step = 0

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
            self.align_then_bend(self._grab_proceed)
        else:
            self.get_logger().warn("正在执行抓取/放置流程，忽略本次信号！")

    def area_callback(self, msg):
        if not self.is_executing:
            self.get_logger().info("="*50)
            self.get_logger().info(f"收到到达放置位置信号，准备分离物块：{self.attach_params['model2_name']}")
            self.is_executing = True
            self.current_task = "place"
            self.current_step = 1
            self.current_joint_pos = self.arm_trajectory["init"]
            self.get_logger().info("步骤1：机械臂弯曲到放置位置...")
            self.align_then_bend(self._place_proceed)
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
