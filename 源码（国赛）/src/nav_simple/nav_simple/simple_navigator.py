import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
import json

class SimpleNav2Navigator(Node):
    def __init__(self):
        super().__init__("simple_nav2_navigator")
        
        # 初始化Nav2动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2服务未启动！请先启动导航栈")
            rclpy.shutdown()
            return
        
        if not self.plan_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2路径规划服务未启动！")
            rclpy.shutdown()
            return

        # 订阅话题
        self.target_sub = self.create_subscription(
            String, "/manual_nav_target", self.target_callback, 10
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool, "/emergency_stop", self.emergency_stop_callback, 10
        )
        
        # 发布话题
        self.status_pub = self.create_publisher(String, "/nav_status", 10)
        
        # 预设区域坐标
        self.area_coords = {
            "A": {"x": 3.143086, "y": -5.807858, "yaw": 0.0},
            "B": {"x": -1.196544, "y": -6.485499, "yaw": 0.0},
            "C": {"x": -6.873777, "y": -7.785160, "yaw": 0.0}
        }
        
        # 导航状态
        self.current_goal_handle = None
        self.emergency_stop_active = False
        self.request_id = 0
        
        self.get_logger().info("增强版Nav2导航节点启动成功！")
        self.get_logger().info("支持：紧急停止、路径中断")
        self.get_logger().info("用法：发布目标到/manual_nav_target")

    def emergency_stop_callback(self, msg):
        """处理紧急停止指令"""
        self.emergency_stop_active = msg.data
        if msg.data:
            self.request_id += 1
            self.get_logger().warn("接收到紧急停止指令")
            if self.current_goal_handle:
                try:
                    self.current_goal_handle.cancel_goal_async()
                    self.current_goal_handle = None
                except Exception as e:
                    self.get_logger().error(f"取消导航时出错：{str(e)}")
            self.status_pub.publish(String(data="emergency_stop"))
        else:
            self.status_pub.publish(String(data="emergency_stop_cleared"))

    def target_callback(self, msg):
        """处理导航目标"""
        try:
            data = json.loads(msg.data)
            
            # 处理暂停指令
            if data["type"] == "pause":
                self.request_id += 1
                self.get_logger().info("接收到暂停指令")
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                    self.current_goal_handle = None
                self.status_pub.publish(String(data="paused"))
                return
            
            # 如果处于紧急停止状态，忽略目标
            if self.emergency_stop_active:
                self.get_logger().warn("处于紧急停止状态，忽略导航目标")
                self.status_pub.publish(String(data="failed: emergency_stop_active"))
                return
            
            # 新目标使旧预规划与导航回调失效。
            self.request_id += 1
            request_id = self.request_id
            # 取消当前导航（如果有）
            if self.current_goal_handle:
                self.get_logger().info("取消当前导航任务")
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            
            # 处理正常导航目标
            if data["type"] == "area":
                area = data.get("target", "")
                if area not in self.area_coords:
                    self.get_logger().error(f"无效区域：{area}")
                    self.status_pub.publish(String(data=f"failed: invalid_area_{area}"))
                    return
                coord = self.area_coords[area]
                self.get_logger().info(f"收到区域目标：{area}区（x={coord['x']}, y={coord['y']}）")
                self.send_goal(coord["x"], coord["y"], coord["yaw"], request_id)
            
            elif data["type"] == "custom":
                x = data.get("x", 0.0)
                y = data.get("y", 0.0)
                yaw = data.get("yaw", 0.0)
                self.get_logger().info(f"收到自定义目标：x={x:.2f}, y={y:.2f}, 朝向={yaw:.2f}rad")
                self.send_goal(x, y, yaw, request_id)
            
            else:
                self.get_logger().error("目标类型错误！")
                self.status_pub.publish(String(data="failed: invalid_type"))
        
        except json.JSONDecodeError:
            self.get_logger().error("目标格式错误！")
            self.status_pub.publish(String(data="failed: invalid_format"))
        except KeyError as e:
            self.get_logger().error(f"缺少必要字段：{str(e)}")
            self.status_pub.publish(String(data=f"failed: missing_field_{str(e)}"))

    def send_goal(self, x, y, yaw, request_id):
        """发送导航目标"""
        # 构造位姿消息
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 位置信息
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 朝向信息
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        goal_msg.pose = pose

        # 先在当前代价地图上验证目标是否有路径，避免盲目跑向固定抓取点。
        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = pose
        plan_goal.use_start = False
        self.get_logger().info("检查目标点是否可达...")
        future = self.plan_client.send_goal_async(plan_goal)
        future.add_done_callback(
            lambda f: self.plan_response_callback(f, goal_msg, request_id)
        )

    def plan_response_callback(self, future, nav_goal, request_id):
        if request_id != self.request_id or self.emergency_stop_active:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self.status_pub.publish(String(data="failed: no_path"))
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.plan_result_callback(f, nav_goal, request_id)
            )
        except Exception as e:
            self.get_logger().error(f"路径检查失败：{e}")
            self.status_pub.publish(String(data="failed: planning_error"))

    def plan_result_callback(self, future, nav_goal, request_id):
        if request_id != self.request_id or self.emergency_stop_active:
            return
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
                self.get_logger().warn("目标点没有有效路径")
                self.status_pub.publish(String(data="failed: no_path"))
                return
            self.get_logger().info("路径检查通过，开始导航")
            nav_future = self.nav_client.send_goal_async(
                nav_goal, feedback_callback=self.feedback_callback
            )
            nav_future.add_done_callback(
                lambda f: self.goal_response_callback(f, request_id)
            )
        except Exception as e:
            self.get_logger().error(f"读取规划结果失败：{e}")
            self.status_pub.publish(String(data="failed: planning_error"))

    def feedback_callback(self, feedback_msg):
        """导航反馈"""
        try:
            feedback = feedback_msg.feedback
            x = feedback.current_pose.pose.position.x
            y = feedback.current_pose.pose.position.y
            self.get_logger().debug(f"当前位置：x={x:.2f}, y={y:.2f}")
        except Exception as e:
            self.get_logger().error(f"处理反馈时出错：{str(e)}")

    def goal_response_callback(self, future, request_id):
        """目标响应回调；过期目标即使刚被接受，也要立即取消。"""
        try:
            goal_handle = future.result()
            if request_id != self.request_id or self.emergency_stop_active:
                if goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                return
            self.current_goal_handle = goal_handle
            
            if not goal_handle.accepted:
                self.get_logger().error("导航目标被Nav2拒绝！")
                self.status_pub.publish(String(data="failed: rejected"))
                self.current_goal_handle = None
                return
            
            self.get_logger().info("导航目标已被接受，等待完成...")
            self._result_future = goal_handle.get_result_async()
            self._result_future.add_done_callback(
                lambda f: self.result_callback(f, request_id)
            )
            
        except Exception as e:
            self.get_logger().error(f"处理目标响应时出错：{str(e)}")
            self.status_pub.publish(String(data=f"failed: response_error"))
            self.current_goal_handle = None

    def result_callback(self, future, request_id):
        """导航结果回调"""
        if request_id != self.request_id:
            return
        try:
            result = future.result()
            status = result.status
            self.current_goal_handle = None
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("导航成功！已到达目标点")
                self.status_pub.publish(String(data="succeeded"))
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("导航失败：路径规划失败/避障中断")
                self.status_pub.publish(String(data="failed: aborted"))
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn("导航已被取消")
                self.status_pub.publish(String(data="canceled"))
            else:
                self.get_logger().error(f"导航结果未知，状态码：{status}")
                self.status_pub.publish(String(data=f"unknown: {status}"))
        
        except Exception as e:
            self.get_logger().error(f"处理结果时出错：{str(e)}")
            self.status_pub.publish(String(data=f"failed: result_error"))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNav2Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，退出节点")
    finally:
        if node.current_goal_handle:
            try:
                node.current_goal_handle.cancel_goal_async()
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()