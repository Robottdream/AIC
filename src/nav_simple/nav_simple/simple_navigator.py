import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from rcl_interfaces.srv import SetParametersAtomically
from tf_transformations import quaternion_from_euler
import json
import math
import time
from collections import deque
from nav_simple.trajectory_prediction import crossing_conflicts
from nav_simple.crossing_passage import choose_passage, track_distance, static_clearance

class SimpleNav2Navigator(Node):
    def __init__(self):
        super().__init__("simple_nav2_navigator")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        
        # 初始化Nav2动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        if not self.nav_client.wait_for_server(timeout_sec=40.0):
            self.get_logger().error("Nav2服务未启动！请先启动导航栈")
            rclpy.shutdown()
            return
        
        if not self.plan_client.wait_for_server(timeout_sec=40.0):
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
        self.create_subscription(
            String, "/current_target_cube", self.cube_callback, 10
        )
        self.create_subscription(
            Pose, "/obstacle3/current_pose", self.obstacle_callback,
            qos_profile_sensor_data
        )
        
        # 发布话题
        self.status_pub = self.create_publisher(String, "/nav_status", 10)
        
        # 预设区域坐标
        self.area_coords = {
            "A": {"x": 2.593086, "y": -5.807858, "yaw": 0.0},
            "B": {"x": -1.746544, "y": -6.485499, "yaw": 0.0},
            "C": {"x": -6.873777, "y": -7.785160, "yaw": 0.0}
        }
        # A/B/C 的停车位置按 [Ax, Ay, Bx, By, Cx, Cy] 配置。
        parking_positions = self.declare_parameter(
            "area_parking_positions",
            [self.area_coords[area][axis]
             for area in ("A", "B", "C") for axis in ("x", "y")],
        ).value
        if len(parking_positions) != 6 or not all(
                math.isfinite(value) for value in parking_positions):
            raise ValueError("area_parking_positions must contain 6 finite coordinates")
        for index, area in enumerate(("A", "B", "C")):
            self.area_coords[area]["x"] = parking_positions[2 * index]
            self.area_coords[area]["y"] = parking_positions[2 * index + 1]
        
        # 导航状态
        self.current_goal_handle = None
        self.emergency_stop_active = False
        self.request_id = 0
        self.goal_pose = None
        self.near_goal_started = None
        self.current_target_cube = ""
        self.obstacle_samples = deque(maxlen=50)
        self.pending_start_timer = None
        self.crossing_max_wait = max(0.0, min(30.0, float(
            self.declare_parameter("crossing_max_wait", 12.0).value)))
        self.crossing_cruise_speed = max(0.1, float(
            self.declare_parameter("crossing_cruise_speed", 2.2).value))
        self.crossing_acceleration = max(0.1, float(
            self.declare_parameter("crossing_acceleration", 3.5).value))
        self.slow_near_static_obstacles = self.declare_parameter(
            "slow_near_static_obstacles", True).value
        names = self.declare_parameter("obstacle_names", Parameter.Type.STRING_ARRAY).value or []
        tracks = self.declare_parameter("obstacle_tracks", Parameter.Type.DOUBLE_ARRAY).value or []
        sizes = self.declare_parameter("obstacle_sizes", Parameter.Type.DOUBLE_ARRAY).value or []
        speeds = self.declare_parameter("obstacle_speeds", Parameter.Type.DOUBLE_ARRAY).value or []
        if len(tracks) != 4*len(names) or len(sizes) != len(names) or len(speeds) != len(names):
            raise ValueError("Each obstacle requires four track coordinates, size and speed")
        if len(set(names)) != len(names) or not all(math.isfinite(v) for v in [*tracks, *sizes, *speeds]):
            raise ValueError("Obstacle names must be unique and track values finite")
        self.scene_tracks = []
        self.scene_obstacle_samples = {}
        self.static_map = None
        self.world_pose = None
        self.world_motion = None
        self.localization_pose = None
        self.navigation_retries = 0
        self.recovery_context = None
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._localization_callback, qos_profile_sensor_data)
        self.safe_route = []
        self.route_stage = None
        self.routed_tracks = set()
        self.guard_canceling = False
        self.crossing_retry = 0
        self.controller_parameters = self.create_client(
            SetParametersAtomically, '/controller_server/set_parameters_atomically')
        self.controller_profile = None
        self.controller_profile_pending = False
        self.controller_profile_retry_at = 0.
        self.create_timer(.2, self._update_controller_profile)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(ModelStates, '/model_states', self._world_callback,
                                 qos_profile_sensor_data)
        self.prediction_last_log = -math.inf
        for i, name in enumerate(names):
            points = tuple(tracks[4*i:4*i+4])
            if sizes[i] <= 0 or speeds[i] <= 0 or math.hypot(points[2]-points[0], points[3]-points[1]) <= .4:
                raise ValueError("Obstacle size/speed must be positive and track longer than 0.4 m")
            self.scene_tracks.append(dict(name=name, points=points, size=sizes[i], speed=speeds[i]))
            self.scene_obstacle_samples[name] = deque(maxlen=50)
            self.create_subscription(Pose, f"/{name}/current_pose",
                lambda msg, name=name: self.scene_obstacle_callback(name, msg), qos_profile_sensor_data)
            self.get_logger().info(f"已接入障碍轨迹预测: {name} {points}, 速度={speeds[i]:.2f}m/s")
        
        self.get_logger().info("增强版Nav2导航节点启动成功！")
        self.get_logger().info("支持：紧急停止、路径中断")
        self.get_logger().info("用法：发布目标到/manual_nav_target")

    def cube_callback(self, msg):
        self.current_target_cube = msg.data.strip()

    def _map_callback(self, msg):
        self.static_map = msg

    def _world_callback(self, msg):
        if 'six_arm' in msg.name:
            index = msg.name.index('six_arm')
            pose = msg.pose[index]
            p, q = pose.position, pose.orientation
            now = self.get_clock().now().nanoseconds*1e-9
            self.world_pose = (now, p.x, p.y)
            if index < len(msg.twist):
                twist = msg.twist[index]
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                self.world_motion = (now, yaw, math.hypot(twist.linear.x, twist.linear.y),
                                     abs(twist.angular.z))

    def _localization_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.localization_pose = (stamp, p.x, p.y, yaw)

    def _recover_navigation(self, request_id, reason):
        # FollowPath has already terminated. Refresh AMCL using new scans while
        # stopped; never inject simulator coordinates or relax collision checks.
        if (request_id != self.request_id or self.emergency_stop_active
                or self.goal_pose is None or self.navigation_retries >= 2):
            return False
        self.navigation_retries += 1
        self._cancel_pending_start()
        self.recovery_context = dict(request_id=request_id, deadline=time.monotonic()+10.,
                                     last_update=-math.inf, future=None, reason=reason,
                                     began=self.get_clock().now().nanoseconds*1e-9,
                                     plan_pending=False, plan_handle=None,
                                     plan_good_until=-math.inf, next_plan=time.monotonic())
        self.stop_pub.publish(Twist())
        self.get_logger().warn(f'导航恢复 {self.navigation_retries}/2：{reason}；停车刷新定位后重规划')
        self.pending_start_timer = self.create_timer(.1, self._navigation_recovery_tick)
        return True

    def _navigation_recovery_tick(self):
        context = self.recovery_context
        if context is None:
            return
        if context['request_id'] != self.request_id or self.emergency_stop_active:
            self._cancel_pending_start()
            return
        self.stop_pub.publish(Twist())
        now = self.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        if wall >= context['deadline']:
            self._cancel_pending_start()
            self.get_logger().error('导航恢复超时：未取得新鲜定位和可执行的当前路线')
            self.status_pub.publish(String(data='failed: localization_recovery_timeout'))
            return
        measured, motion, localized = self.world_pose, self.world_motion, self.localization_pose
        ready = False
        if measured is not None and motion is not None and localized is not None:
            heading_error = math.atan2(math.sin(localized[3]-motion[1]),
                                       math.cos(localized[3]-motion[1]))
            ready = (all(math.isfinite(v) for v in (*measured, *motion, *localized))
                     and 0 <= now-measured[0] <= .25 and 0 <= now-motion[0] <= .25
                     and -.1 <= now-localized[0] <= .5
                     and localized[0] > context['began']
                     and motion[2] < .05 and motion[3] < .1
                     and math.hypot(localized[1]-measured[1], localized[2]-measured[2]) < .3
                     and abs(heading_error) < .25)
        if ready and wall <= context['plan_good_until']:
            request_id = context['request_id']
            self._cancel_pending_start()
            goal = NavigateToPose.Goal()
            goal.pose = self.goal_pose
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info('停车后定位及当前路线核验通过，重新导航当前目标')
            self._start_navigation(goal, request_id)
            return
        if ready and not context['plan_pending'] and wall >= context['next_plan']:
            # Localization may recover while the moving actor still blocks all
            # routes. Do not consume another navigation attempt in that phase.
            goal = ComputePathToPose.Goal()
            goal.goal = self.goal_pose
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.use_start = False
            context['plan_pending'] = True
            future = self.plan_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self._recovery_plan_response(f, context))
        if (wall-context['last_update'] >= 1. and self.nomotion_client.service_is_ready()
                and (context['future'] is None or context['future'].done())):
            context['last_update'] = wall
            context['future'] = self.nomotion_client.call_async(Empty.Request())

    def _recovery_plan_response(self, future, context):
        try:
            handle = future.result()
            if context is not self.recovery_context:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                context['plan_pending'] = False
                context['next_plan'] = time.monotonic()+.75
                return
            context['plan_handle'] = handle
            handle.get_result_async().add_done_callback(
                lambda f: self._recovery_plan_result(f, context))
        except Exception as error:
            if context is self.recovery_context:
                context['plan_pending'] = False
                context['next_plan'] = time.monotonic()+.75
                self.get_logger().warn(f'恢复路线查询失败，将有限重试：{error}')

    def _recovery_plan_result(self, future, context):
        if context is not self.recovery_context:
            return
        context['plan_pending'] = False
        context['plan_handle'] = None
        context['next_plan'] = time.monotonic()+.75
        try:
            result = future.result()
            path = result.result.path
            valid = (result.status == GoalStatus.STATUS_SUCCEEDED and bool(path.poses)
                     and path.header.frame_id == 'map')
            if valid:
                end = path.poses[-1].pose.position
                target = self.goal_pose.pose.position
                valid = math.hypot(end.x-target.x, end.y-target.y) < .05
            if valid:
                context['plan_good_until'] = time.monotonic()+.5
            else:
                self.get_logger().warn('定位已恢复，当前路线暂不可用；保持停车并有限重查')
        except Exception as error:
            self.get_logger().warn(f'恢复路线结果无效，将有限重试：{error}')

    def _update_controller_profile(self, ready=None):
        now = self.get_clock().now().nanoseconds*1e-9
        if (self.world_pose is None or now-self.world_pose[0] > .25
                or self.static_map is None or self.goal_pose is None):
            if ready is not None:
                ready()
            return
        clearance = static_clearance(self.static_map, self.world_pose[1:])
        # Hysteresis prevents rapid parameter changes at a corridor entrance.
        near = clearance < (1.05 if self.controller_profile == 'near' else .80)
        crossing = self.route_stage is not None and self.route_stage['kind'] == 'cross'
        # Set the crossing profile before departure, rather than changing it
        # midway. Actual cruise now matches the conservative timing model.
        profile = 'cross' if crossing else ('near' if near else 'open')
        if profile == self.controller_profile:
            if ready is not None:
                ready()
            return
        if self.controller_profile_pending:
            # A start request must wait for the outstanding controller update.
            if ready is not None:
                self.controller_profile_future.add_done_callback(
                    lambda _: self._update_controller_profile(ready))
            return
        if (now < self.controller_profile_retry_at
                or not self.controller_parameters.service_is_ready()):
            if ready is not None:
                ready()
            return
        constrained = near or crossing
        # Near-wall lookahead remains independent of the optional speed cap.
        wall_speed_cap = near and self.slow_near_static_obstacles
        values = {'FollowPath.desired_linear_vel': .8 if crossing else (1.2 if wall_speed_cap else self.crossing_cruise_speed),
                  'FollowPath.max_lookahead_dist': .45 if constrained else 1.5,
                  'FollowPath.lookahead_time': .4 if constrained else 1.0}
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in values.items()]
        self.controller_profile_pending = True
        self.controller_profile_future = self.controller_parameters.call_async(request)
        def applied(future):
            self.controller_profile_pending = False
            try:
                result = future.result().result
                if not result.successful:
                    raise RuntimeError(result.reason)
                self.controller_profile = profile
                self.get_logger().info(f'导航转弯配置={profile}, 静态净距={clearance:.2f}m, '
                                       f'直行目标={values["FollowPath.desired_linear_vel"]:.2f}m/s')
            except Exception as error:
                self.controller_profile_retry_at = now+2.
                self.get_logger().warn(f'导航转弯参数更新失败: {error}')
            if ready is not None:
                ready()
        self.controller_profile_future.add_done_callback(applied)

    def _install_crossing_route(self, path, nav_goal):
        if self.route_stage is not None and self.route_stage['kind'] != 'final':
            return False
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        for track in self.scene_tracks:
            if track['name'] in self.routed_tracks:
                continue
            passage = choose_passage(points, track, self.static_map)
            if passage is None:
                continue
            self.routed_tracks.add(track['name'])
            p, q = nav_goal.pose.pose.position, nav_goal.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            entry, exit = passage['entry'], passage['exit']
            final = dict(kind='final', target=(p.x,p.y,yaw))
            remaining = self.safe_route[1:] if self.safe_route else []
            self.safe_route = [dict(kind='entry', target=(*entry,passage['yaw']), passage=passage),
                dict(kind='cross', target=(*exit,passage['yaw']), passage=passage), final, *remaining]
            mode = '端点外绕行' if passage.get('bypass') else '安全穿越'
            self.get_logger().info(f"{mode} {track['name']}: 轨道外等待点={entry}, 出口={exit}")
            self._send_route_stage()
            return True
        return False

    def _send_route_stage(self):
        self.route_stage = self.safe_route[0]
        self.send_goal(*self.route_stage['target'], self.request_id)

    def _check_crossing_entry(self):
        stage = self.route_stage
        if (stage is None or stage['kind'] != 'cross' or self.guard_canceling
                or self.current_goal_handle is None or self.emergency_stop_active):
            return
        now = self.get_clock().now().nanoseconds*1e-9
        if self.world_pose is None or now-self.world_pose[0] > .25:
            return
        position = self.world_pose[1:]
        passage = stage['passage']
        # Revalidate while braking still leaves the chassis outside the swept
        # area. Never use an intentional waiting stop inside that area.
        if track_distance(position, passage['track']) < 1.10:
            return
        if not crossing_conflicts([position, passage['exit']], [passage['track']],
                self.scene_obstacle_samples, now, 0., .8, self.crossing_acceleration):
            return
        self.crossing_retry += 1
        self.guard_canceling = True
        self.request_id += 1
        request_id = self.request_id
        self.get_logger().warn('穿越前预测更新，取消进入并回到轨道外等待点')
        self.current_goal_handle.cancel_goal_async()
        def stopped(_):
            if request_id != self.request_id:
                return
            self.current_goal_handle = None
            self.guard_canceling = False
            if self.crossing_retry > 4:
                self.status_pub.publish(String(data='failed: crossing_entry_unstable'))
                return
            self.safe_route.insert(0, dict(kind='entry',
                target=(*passage['entry'], passage['yaw']), passage=passage))
            self._send_route_stage()
        self._result_future.add_done_callback(stopped)

    def obstacle_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not math.isfinite(msg.position.y):
            return
        if self.obstacle_samples and now < self.obstacle_samples[-1][0]:
            self.obstacle_samples.clear()  # Gazebo reset invalidates old velocities.
        if self.obstacle_samples and now - self.obstacle_samples[-1][0] < 0.1:
            return
        self.obstacle_samples.append((now, msg.position.y))

    def _cancel_pending_start(self):
        context = self.recovery_context
        if context is not None and context.get('plan_handle') is not None:
            context['plan_handle'].cancel_goal_async()
        self.recovery_context = None
        if self.pending_start_timer is not None:
            self.pending_start_timer.cancel()
            self.destroy_timer(self.pending_start_timer)
            self.pending_start_timer = None

    def scene_obstacle_callback(self, name, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        x, y = msg.position.x, msg.position.y
        if not math.isfinite(x) or not math.isfinite(y):
            return
        samples = self.scene_obstacle_samples[name]
        if samples and now < samples[-1][0]:
            samples.clear()
        if not samples or now-samples[-1][0] >= .05:
            samples.append((now, x, y))

    def _scene_crossing_wait(self, path):
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        now = self.get_clock().now().nanoseconds*1e-9
        crossing = self.route_stage is not None and self.route_stage['kind'] == 'cross'
        straight = None
        if crossing:
            # Obstacle samples are Gazebo world coordinates. Use the same
            # measured start and straight corridor as the live entry guard.
            if self.world_pose is None or now-self.world_pose[0] > .25:
                return .25
            straight = [self.world_pose[1:], self.route_stage['passage']['exit']]
        tracks = [self.route_stage['passage']['track']] if crossing else self.scene_tracks
        speed = .8 if crossing else self.crossing_cruise_speed
        conflicts = crossing_conflicts(points, tracks, self.scene_obstacle_samples,
            now, 0., speed, self.crossing_acceleration)
        if straight is not None:
            conflicts |= crossing_conflicts(straight, tracks,
                self.scene_obstacle_samples, now, 0., speed, self.crossing_acceleration)
        if not conflicts:
            return 0.
        for tick in range(1, int(self.crossing_max_wait/.25)+1):
            delay = tick*.25
            if (not crossing_conflicts(points, tracks, self.scene_obstacle_samples,
                    now, delay, speed, self.crossing_acceleration)
                    and (straight is None or not crossing_conflicts(straight, tracks,
                        self.scene_obstacle_samples, now, delay, speed, self.crossing_acceleration))):
                if now-self.prediction_last_log >= 1.:
                    self.get_logger().info(f"二维轨迹预测冲突: {','.join(sorted(conflicts))}, 建议等待={delay:.2f}s")
                    self.prediction_last_log = now
                return delay
        # Keep the start gate closed until it is safe or its bounded timer fails.
        if now-self.prediction_last_log >= 1.:
            self.get_logger().warn(f"二维轨迹交叉暂被占用: {','.join(sorted(conflicts))}")
            self.prediction_last_log = now
        return .25

    @staticmethod
    def _predicted_obstacle_y(y, velocity, seconds):
        # obstacle3 travels between y=-2 and y=3 and reverses near each end.
        # The Gazebo controller reverses within 0.2 m of an endpoint.
        lower, upper = -1.8, 2.8
        predicted = y + velocity * seconds
        for _ in range(8):
            if predicted > upper:
                predicted = 2.0 * upper - predicted
            elif predicted < lower:
                predicted = 2.0 * lower - predicted
            else:
                break
        return predicted

    def _crossing_wait(self, nav_goal, path):
        """Estimate a short wait when a planned path crosses obstacle3's track."""
        if getattr(self, 'scene_tracks', None):
            return self._scene_crossing_wait(path)
        if len(self.obstacle_samples) < 2:
            return 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.obstacle_samples[-1][0] > 1.0:
            return 0.0
        old_time, old_y = next(
            ((t, y) for t, y in reversed(self.obstacle_samples)
             if now - t >= 0.35), self.obstacle_samples[0]
        )
        dt = self.obstacle_samples[-1][0] - old_time
        if dt < 0.2:
            return 0.0
        current_y = self.obstacle_samples[-1][1]
        velocity = max(-0.40, min(0.40, (current_y - old_y) / dt))
        if abs(velocity) < 0.03:
            return 0.0
        travelled = 0.0
        crossing = None
        for before, after in zip(path.poses, path.poses[1:]):
            x1, y1 = before.pose.position.x, before.pose.position.y
            x2, y2 = after.pose.position.x, after.pose.position.y
            segment = math.hypot(x2 - x1, y2 - y1)
            if (x1 + 2.5) * (x2 + 2.5) <= 0 and abs(x2 - x1) > 1e-5:
                fraction = (-2.5 - x1) / (x2 - x1)
                crossing = (y1 + fraction * (y2 - y1), travelled + segment * fraction)
                break
            travelled += segment
        if crossing is None or not -2.5 <= crossing[0] <= 3.5:
            return 0.0
        crossing_y, distance = crossing
        # An obstacle several metres ahead is already handled by Nav2's live
        # costmap while driving. Holding a newly grasped block here parks the
        # robot at the pickup point even though the path ahead is free.
        if distance > 2.0:
            return 0.0
        speed = getattr(self, 'crossing_cruise_speed', 2.2)
        acceleration = getattr(self, 'crossing_acceleration', 3.5)
        ramp_distance = speed * speed / (2.0 * acceleration)
        travel_seconds = (math.sqrt(2.0 * distance / acceleration)
                          if distance <= ramp_distance else
                          speed / acceleration + (distance - ramp_distance) / speed)
        self.get_logger().info(
            f"交叉预测评估: 交叉y={crossing_y:.2f} 路程={distance:.2f}m "
            f"障碍y={current_y:.2f} 速度={velocity:.2f}m/s")
        for delay_ticks in range(int(self.crossing_max_wait / 0.5) + 1):
            delay = delay_ticks * 0.5
            predicted = [self._predicted_obstacle_y(
                current_y, velocity, max(0.0, travel_seconds + delay + offset)
            ) for offset in (-2.0, 0.0, 2.0)]
            obstacle_y = predicted[1]
            if min(abs(y - crossing_y) for y in predicted) >= 1.35:
                self.get_logger().info(
                    f"回程交叉预测: 交叉 y={crossing_y:.2f}, "
                    f"障碍当前 y={current_y:.2f}, 速度={velocity:.2f}, "
                    f"预计 y={obstacle_y:.2f}, 等待={delay:.1f}s")
                return delay
        self.get_logger().warn("交叉点短时持续被占，交由 Nav2 实时避障")
        return 0.0

    def emergency_stop_callback(self, msg):
        """处理紧急停止指令"""
        self.emergency_stop_active = msg.data
        if msg.data:
            self._cancel_pending_start()
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
                self._cancel_pending_start()
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
            self._cancel_pending_start()
            self.request_id += 1
            request_id = self.request_id
            self.safe_route = []
            self.route_stage = None
            self.routed_tracks = set()
            self.crossing_retry = 0
            self.navigation_retries = 0
            self.guard_canceling = False
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
        self.goal_pose = pose
        self.near_goal_started = None

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
            # The timed planner uses live actor forecasts while driving. Send
            # the original task goal directly; no waiting/crossing waypoints.
            self._start_navigation(nav_goal, request_id)
        except Exception as e:
            self.get_logger().error(f"读取规划结果失败：{e}")
            self.status_pub.publish(String(data="failed: planning_error"))

    def _start_navigation(self, nav_goal, request_id):
        self._update_controller_profile(
            lambda: self._launch_navigation(nav_goal, request_id))

    def _launch_navigation(self, nav_goal, request_id):
        if request_id != self.request_id or self.emergency_stop_active:
            return
        self.get_logger().info("路径检查通过，开始导航")
        nav_future = self.nav_client.send_goal_async(
            nav_goal, feedback_callback=lambda msg: self.feedback_callback(msg, request_id)
        )
        nav_future.add_done_callback(
            lambda future: self.goal_response_callback(future, request_id)
        )

    def feedback_callback(self, feedback_msg, request_id):
        """导航反馈"""
        try:
            if request_id != self.request_id or self.goal_pose is None:
                return
            feedback = feedback_msg.feedback
            x = feedback.current_pose.pose.position.x
            y = feedback.current_pose.pose.position.y
            distance = math.hypot(x - self.goal_pose.pose.position.x,
                                  y - self.goal_pose.pose.position.y)
            if distance <= 0.5 and self.near_goal_started is None:
                self.near_goal_started = time.monotonic()
                self.get_logger().info(f"进入目标0.5米范围，距离={distance:.3f}m")
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
                if self.safe_route:
                    self.safe_route.pop(0)
                    if self.safe_route:
                        self._send_route_stage()
                        return
                    self.route_stage = None
                # Humble can report a reached goal after a failed TF lookup.
                # In this simulator the task/map coordinates share world XY;
                # confirm fresh measured arrival before permitting a grasp.
                measured = self.world_pose
                now = self.get_clock().now().nanoseconds*1e-9
                if measured is None or now-measured[0] > .5 or self.goal_pose is None:
                    self.get_logger().error('到达核验失败：没有新鲜的仿真底盘位置')
                    if self._recover_navigation(request_id, 'arrival_pose_unavailable'):
                        return
                    self.status_pub.publish(String(data='failed: arrival_pose_unavailable'))
                    return
                target = self.goal_pose.pose.position
                arrival_error = math.hypot(measured[1]-target.x, measured[2]-target.y)
                if arrival_error > .35:
                    self.get_logger().error(f'拒绝到达误报：底盘距任务目标仍有{arrival_error:.2f}m')
                    if self._recover_navigation(request_id, 'arrival_position_mismatch'):
                        return
                    self.status_pub.publish(String(data='failed: arrival_position_mismatch'))
                    return
                if self.near_goal_started is not None:
                    elapsed = time.monotonic() - self.near_goal_started
                    self.get_logger().info(f"目标末段耗时：{elapsed:.2f}s")
                self.get_logger().info("导航成功！已到达目标点")
                self.status_pub.publish(String(data="succeeded"))
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("导航失败：路径规划失败/避障中断")
                if self._recover_navigation(request_id, 'aborted'):
                    return
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
