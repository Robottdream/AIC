#!/usr/bin/env python3
"""Nav2 导航链路体检：确认 planner_server 眼里的“机器人当前位置”与定位一致。

为什么需要这个检查（2026-09-28 本机实测）：
Nav2 容器内部的 TF 视图一旦停更，planner_server 会一直拿停更时刻的位姿当起点规划。
实测症状：机器人真实位姿 (8.71,-0.81)，而 /compute_path_to_pose 无论给什么目标，
返回路径的起点永远是 (2.70,-5.85)——即停更瞬间的位姿（相差 7.4 m）。
DWB 收到“起点在 5 m 外”的路径后，transformGlobalPlan 找不到可用近端航点，
每个控制周期都抛 "Resulting plan has 0 poses in it."，控制器输出 0 速度，
BT 反复 wait/spin/backup 后 Goal failed（实测 1571 次报错、280 次 patience exceeded）。
上层 main_controller 把它理解成“抓取点不可达”，逐一切换 4 个接近方向再换同色物块：
现场表现就是“小车在目标附近徘徊很久迟迟不下爪”（实测浪费 12 分钟、一件没抓）。

注意：新起的外部进程可能看见正常 TF；规划器起点检查只能覆盖规划器。
控制器还要检查 map→odom 变换年龄及近期 TF 过期日志，防止规划正常但控制器 TF 已停更。
仿真默认 AIC_ODOM_MAP=1 时，静态 map→odom 的时间戳为零，视为有效；
动态 AMCL 模式（AIC_ODOM_MAP=0）则必须持续更新。近期日志仅在持续滞后超过 10 s 时判故障。
路径长度只展示，不以绕墙后的路径/直线距离之比判断故障。

用法：
    python3 preflight_check.py                      # 体检；健康时退出码 0
    python3 preflight_check.py --json               # 机器可读
    python3 preflight_check.py --no-ground-truth    # 跳过与 Gazebo 真值的比对
    python3 preflight_check.py --expect-offset 2,0  # 自检：故意把期望位姿挪 2 m，验证检查确实会失败

退出码：0=健康 2=规划器起点与定位不一致 4=规划器不可用
        5=TF 与 AMCL/定位参考不一致 6=定位与 Gazebo 真值不一致（跑飞/未初始化）
        7=TF 里没有 map 帧（定位链断了）
        8=导航日志出现持续控制器 TF 过期 9=Gazebo 真位姿越出地图、离地或严重倾翻，需重启仿真
        10=规划请求有明确终态但到体检目标无路径，不因此重启导航栈
"""

import argparse
import json
import math
import os
import re
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener

# 三个停车位（room.world 坐标）；体检时优先取离机器人最远的那个当目标
ZONE_CANDIDATES = [(2.593086, -5.727858), (-1.746544, -6.485499), (-7.423777, -7.785160)]
HEAD_TOL = 0.75       # 路径起点与定位参考的最大允许偏差（m）
TRUTH_TOL = 0.75      # TF 定位与 Gazebo 真值的最大允许偏差（m）
ROBOT_MODEL = 'six_arm'


class Preflight(Node):
    def __init__(self):
        super().__init__('nav_preflight')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    def _amcl_cb(self, msg):
        self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def spin_for(self, seconds):
        """按**墙钟**等待（不依赖 /clock，sim 时间不可用时也能正常判断）。"""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_frames(self, names, timeout_s):
        """等到 TF 缓冲里出现指定坐标系（节点刚起时 DDS 发现可能要十几秒）。"""
        end = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                yaml = self.tf_buffer.all_frames_as_yaml()
            except Exception:                                        # noqa: BLE001
                continue
            if all(re.search(r'(?m)^' + re.escape(n) + r':', yaml) for n in names):
                return True
        return False

    def tf_pose(self, target='map'):
        try:
            tr = self.tf_buffer.lookup_transform(target, 'base_link', Time())
            return (tr.transform.translation.x, tr.transform.translation.y), None
        except Exception as exc:                                     # noqa: BLE001
            return None, str(exc)

    def map_odom_age(self):
        """Planner start position can look right even when AMCL's TF stamp froze."""
        try:
            tr = self.tf_buffer.lookup_transform('map', 'odom', Time())
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if stamp == 0 and os.environ.get('AIC_ODOM_MAP', '').lower() in ('1', 'true', 'yes'):
                return 0.0, None  # static transform is valid at every time
            return self.get_clock().now().nanoseconds * 1e-9 - stamp, None
        except Exception as exc:                                     # noqa: BLE001
            return None, str(exc)

    def plan_to(self, x, y):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False
        if not self.plan_client.wait_for_server(timeout_sec=10.0):
            return 'NO_SERVER', None
        fut = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return 'REJECTED', None
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=15.0)
        result = res_fut.result()
        if result is None:
            return 'TIMEOUT', None
        # Aborted planners may carry a partial path. Its head cannot be used
        # as evidence of a frozen TF view, or watchdog recovery misfires.
        if result.status != 4:
            return 'STATUS_%d' % result.status, None
        return 'OK', result.result.path


def path_length(path):
    total = 0.0
    prev = None
    for ps in path.poses:
        cur = (ps.pose.position.x, ps.pose.position.y)
        if prev is not None:
            total += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        prev = cur
    return total


def gazebo_truth():
    """Gazebo 里机器人的真位姿（仅仿真环境可用；拿不到就返回 None）。"""
    try:
        out = subprocess.run(['gz', 'model', '-m', ROBOT_MODEL, '-p'],
                             capture_output=True, text=True, timeout=25)
        fields = out.stdout.split()
        if len(fields) >= 6:
            return tuple(float(v) for v in fields[:6])
    except Exception:                                                # noqa: BLE001
        pass
    return None


def broken_physics(pose):
    """Only flat-floor simulation is used; these poses cannot be driven."""
    return (not all(math.isfinite(v) for v in pose)
            or max(abs(pose[0]), abs(pose[1])) > 50.0
            or abs(pose[2]) > 0.5
            or max(abs(pose[3]), abs(pose[4])) > math.pi / 3)


def recent_stale_tf(log_path, sim_now):
    """Read only recent TF errors; historic errors must not restart a healthy stack."""
    try:
        with Path(log_path).open('rb') as handle:
            handle.seek(0, 2)
            handle.seek(max(0, handle.tell() - 262144))
            text = handle.read().decode('utf-8', errors='replace')
    except OSError:
        return None
    pattern = r'Data time: (\d+)s (\d+)ns, Transform time: (\d+)s (\d+)ns'
    for match in reversed(list(re.finditer(pattern, text))):
        data_s, data_ns, tf_s, tf_ns = map(int, match.groups())
        data_time = data_s + data_ns * 1e-9
        transform_time = tf_s + tf_ns * 1e-9
        age = data_time - transform_time
        # The watchdog runs every 30 wall seconds; a 5-second lookback missed
        # short controller failures between checks. Restart overwrites this log.
        # A few seconds of TF lag can clear under load. Reserve a full Nav2
        # restart for a sustained stall, not a transient recovery.
        if -1.0 <= sim_now - data_time <= 45.0 and age > 10.0:
            return {'data_time': round(data_time, 3),
                    'transform_time': round(transform_time, 3),
                    'lag_seconds': round(age, 3)}
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--nav-log', default=str(Path(__file__).resolve().parent.parent / 'log/run/03_nav2.log'))
    ap.add_argument('--no-ground-truth', action='store_true')
    ap.add_argument('--expect-offset', default='0,0',
                    help='自检用：把期望位姿偏移 dx,dy（米），用于验证检查确实会失败')
    args = ap.parse_args()
    off = tuple(float(v) for v in args.expect_offset.split(','))

    rclpy.init()
    node = Preflight()
    out = {'checks': {}, 'fail': None, 'code': 0}
    try:
        # 先等 TF 链路就绪（含 DDS 发现时间），再取各参考位姿
        node.wait_for_frames(['odom', 'base_footprint'], 30.0)
        # AMCL 在机器人静止时可能长时间不发 /amcl_pose（实测空闲时约 0.8 Hz 甚至更少），
        # 所以只当“参考之一”，拿不到不算失败；定位参考以 TF 为准。
        node.spin_for(6.0)
        if node.amcl is None:
            out['checks']['amcl_pose'] = '6 s 内无新数据（机器人静止时属正常）'
        else:
            out['checks']['amcl_pose'] = [round(v, 3) for v in node.amcl]

        ref, err = node.tf_pose()
        if ref is None:
            out['checks']['tf_map_base_link'] = 'ERR: %s' % err
            odom_xy, odom_err = node.tf_pose('odom')
            out['checks']['tf_odom_base_link'] = ([round(v, 3) for v in odom_xy]
                                                  if odom_xy else 'ERR: %s' % odom_err)
            if odom_xy is not None and 'does not exist' in err and 'map' in err:
                out['fail'] = ('TF 里根本没有 map 帧：AMCL 没有在发布 map->odom'
                               '（激光被 TF 滤波器丢弃 / AMCL 停止更新）—— 定位链已断，导航必然空转')
                out['code'] = 7
            else:
                out['fail'] = 'TF map->base_link 查询失败：%s' % err
                out['code'] = 5
        else:
            out['checks']['tf_map_base_link'] = [round(v, 3) for v in ref]

        tf_age, tf_age_error = node.map_odom_age()
        out['checks']['map_odom_age_s'] = (round(tf_age, 3) if tf_age is not None
                                           else 'ERR: %s' % tf_age_error)
        if out['code'] == 0 and tf_age is not None and not -2.0 <= tf_age <= 2.0:
            out['fail'] = 'AMCL 的 map->odom 变换已过期 %.3f s，控制器可能误报到达' % tf_age
            out['code'] = 8

        if out['code'] == 0 and node.amcl is not None:
            d_tf = math.hypot(ref[0] - node.amcl[0], ref[1] - node.amcl[1])
            out['checks']['tf_vs_amcl'] = round(d_tf, 3)
            if d_tf > HEAD_TOL:
                out['fail'] = ('TF map->base_link (%.2f, %.2f) 与 /amcl_pose (%.2f, %.2f) 差 %.2f m'
                               % (ref[0], ref[1], node.amcl[0], node.amcl[1], d_tf))
                out['code'] = 5

        # Physical failure takes priority over TF/localization discrepancies:
        # a flipped or ejected robot cannot be repaired by restarting Nav2.
        truth = None if args.no_ground_truth else gazebo_truth()
        if truth is None:
            out['checks']['gazebo_truth'] = 'skip'
        else:
            out['checks']['gazebo_truth'] = [round(v, 3) for v in truth]
            if broken_physics(truth):
                out['fail'] = 'Gazebo 机器人真位姿越出地图、离地或严重倾翻；请停止并重启仿真场景'
                out['code'] = 9
            elif out['code'] == 0:
                # gz model is a blocking query. Drain queued TF updates before
                # comparing so normal robot motion is not misread as drift.
                node.spin_for(0.2)
                live_ref, _ = node.tf_pose()
                if live_ref is not None:
                    ref = live_ref
                    out['checks']['tf_map_base_link'] = [round(v, 3) for v in ref]
                d_truth = math.hypot(truth[0] - ref[0], truth[1] - ref[1])
                out['checks']['loc_vs_truth'] = round(d_truth, 3)
                if d_truth > TRUTH_TOL:
                    out['fail'] = ('定位 (%.2f, %.2f) 与 Gazebo 真值 (%.2f, %.2f) 差 %.2f m'
                                   '（阈值 %.2f）—— 定位跑飞，或 Nav2 重启后没重设初始位姿'
                                   % (ref[0], ref[1], truth[0], truth[1], d_truth, TRUTH_TOL))
                    out['code'] = 6

        if out['code'] == 0:
            exp = (ref[0] + off[0], ref[1] + off[1])
            zones = sorted(ZONE_CANDIDATES,
                           key=lambda z: -math.hypot(z[0] - exp[0], z[1] - exp[1]))
            last_status = 'NONE'
            for gx, gy in zones:
                # Use a current reference immediately before the request;
                # the Gazebo query may have taken time while driving.
                live_ref, _ = node.tf_pose()
                if live_ref is not None:
                    exp = (live_ref[0] + off[0], live_ref[1] + off[1])
                status, path = node.plan_to(gx, gy)
                last_status = status
                if path is None or not path.poses:
                    continue
                head = (path.poses[0].pose.position.x, path.poses[0].pose.position.y)
                d_head = math.hypot(head[0] - exp[0], head[1] - exp[1])
                plen = path_length(path)
                straight = math.hypot(gx - exp[0], gy - exp[1])
                out['checks']['plan_goal'] = [round(gx, 3), round(gy, 3)]
                out['checks']['plan_poses'] = len(path.poses)
                out['checks']['plan_head'] = [round(v, 3) for v in head]
                out['checks']['plan_head_dist'] = round(d_head, 3)
                out['checks']['plan_length'] = round(plen, 2)
                out['checks']['straight_dist'] = round(straight, 2)
                if d_head > HEAD_TOL:
                    out['fail'] = ('规划器认为机器人在 (%.2f, %.2f)，与定位 (%.2f, %.2f) 差 %.2f m'
                                   ' —— Nav2 的位姿视图已停更，导航会全程空转'
                                   % (head[0], head[1], exp[0], exp[1], d_head))
                    out['code'] = 2
                break
            if out['code'] == 0 and 'plan_head' not in out['checks']:
                out['fail'] = '规划器没能给出可用路径（最后一次状态 %s）' % last_status
                out['code'] = 10 if last_status == 'STATUS_6' else 4
        stale = recent_stale_tf(args.nav_log, node.get_clock().now().nanoseconds / 1e9)
        out['checks']['controller_tf_log'] = stale or 'no recent stale TF error (passive log check)'
        if stale and out['code'] in (0, 10):
            out['fail'] = '导航控制器 TF 已过期 %.3f s；规划起点正常不代表控制链正常' % stale['lag_seconds']
            out['code'] = 8
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                            # noqa: BLE001
            pass

    if args.json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        for key, val in out['checks'].items():
            print('  %-18s %s' % (key, val))
        if out['fail']:
            print('PREFLIGHT=FAIL:%d  %s' % (out['code'], out['fail']))
        else:
            print('PREFLIGHT=OK  规划起点与定位一致，导航链路可用')
    return out['code']


if __name__ == '__main__':
    sys.exit(main())
