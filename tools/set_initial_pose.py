#!/usr/bin/env python3
"""给 AMCL 重设初始位姿，并验证它真的生效了。

为什么不用 ros2 topic pub：那个 CLI 的 YAML 解析对这种嵌套消息很脆（实测直接抛
yaml.composer 异常，消息根本没发出去），而且发了不管，无法知道 AMCL 有没有接受。

什么时候必须用它：Nav2 整组重启后，AMCL 会回到参数里的 initial_pose = (0,0,0)
（也就是机器人出生点）。机器人当时在别处时定位就整体错了 —— 实测 AMCL 报
(4.67,-0.05) 而真实位姿是 (8.71,-0.81)，于是规划出的目标点全错，小车会开去无关的地方。

用法：
    python3 set_initial_pose.py X Y YAW        # YAW 单位 rad
    python3 set_initial_pose.py --from-gazebo  # 直接用 Gazebo 真位姿（仿真专用）
退出码：0=已生效 1=发布后 /amcl_pose 仍未收敛到目标 2=参数/环境问题
"""

import argparse
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

TOPIC = '/initialpose'
TOL = 0.6          # 认为“已生效”的偏差阈值（m）
WAIT_S = 25.0      # 等 /amcl_pose 收敛的最长时间（s）


def gazebo_pose(model='six_arm'):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                         capture_output=True, text=True, timeout=25)
    f = out.stdout.split()
    if len(f) < 6:
        raise RuntimeError('拿不到 Gazebo 位姿: %r' % out.stdout[:80])
    return float(f[0]), float(f[1]), float(f[5])


class Setter(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, TOPIC, qos)
        self.pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._cb, qos)

    def _cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def send(self, x, y, yaw, times=5):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25          # AMCL 会忽略协方差过小的初始位姿
        cov[35] = 0.0685
        msg.pose.covariance = cov
        # 连发几次：DDS 发现还没完成时只发一次会静默丢掉
        end = time.monotonic() + 3.0
        n = 0
        while time.monotonic() < end and n < times:
            self.pub.publish(msg)
            n += 1
            rclpy.spin_once(self, timeout_sec=0.4)
        return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', nargs='?', type=float)
    ap.add_argument('y', nargs='?', type=float)
    ap.add_argument('yaw', nargs='?', type=float, default=0.0)
    ap.add_argument('--from-gazebo', action='store_true')
    ap.add_argument('--tolerance', type=float, default=TOL)
    args = ap.parse_args()

    if args.from_gazebo or args.x is None:
        try:
            args.x, args.y, args.yaw = gazebo_pose()
            print('[定位] 取自 Gazebo 真位姿：x=%.3f y=%.3f yaw=%.3f' % (args.x, args.y, args.yaw))
        except Exception as exc:
            print('无法取得 Gazebo 位姿：%s' % exc)
            return 2

    rclpy.init()
    node = Setter()
    try:
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        n = node.send(args.x, args.y, args.yaw)
        print('[定位] 已发布 %d 次 /initialpose：x=%.3f y=%.3f yaw=%.3f' % (n, args.x, args.y, args.yaw))
        deadline = time.monotonic() + WAIT_S
        last = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose is not None:
                last = node.pose
                if math.hypot(last[0] - args.x, last[1] - args.y) < args.tolerance:
                    print('[定位] AMCL 已收敛：/amcl_pose=(%.3f, %.3f)，偏差 %.3f m'
                          % (last[0], last[1], math.hypot(last[0] - args.x, last[1] - args.y)))
                    return 0
        if last is None:
            print('[定位] 失败：%s s 内没收到 /amcl_pose（AMCL 没在更新？）' % WAIT_S)
        else:
            print('[定位] 失败：/amcl_pose=(%.3f, %.3f) 与目标 (%.3f, %.3f) 相差 %.3f m'
                  % (last[0], last[1], args.x, args.y, math.hypot(last[0] - args.x, last[1] - args.y)))
        return 1
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
