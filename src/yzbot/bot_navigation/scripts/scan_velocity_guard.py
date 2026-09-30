#!/usr/bin/env python3
"""Fence stale scan/command data after the Humble collision monitor.

Humble ignores an expired source instead of stopping. Wall timers also protect
against a stopped simulation clock; no command bypasses the monitor.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from dynamic_obstacle_tracker import Tracker, clusters
import json


class ScanVelocityGuard(Node):
    def __init__(self):
        super().__init__('scan_velocity_guard')
        self.scan_received = None
        self.scan_stamp = None
        self.command_received = None
        self.costmap_stamps = {}
        # Historical parameter name: these are TF-derived pose timestamps,
        # not direct measurements of either costmap node's clock.
        self.require_costmap_clock = self.declare_parameter('require_costmap_clock', True).value
        self.predictive_enabled = self.declare_parameter('predictive_enabled', False).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracker = Tracker()
        self.robot_pose = None
        self.tracking_stamp = None
        self.pending_scan = None
        self.hold_until = 0.0
        self.prediction_stopped = False
        self.dynamic_status = {'reason': 'disabled'}
        self.status_pub = self.create_publisher(String, '/navigation/dynamic_guard', 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_guarded', self.command, 10)
        for scope in ('local', 'global'):
            self.create_subscription(PolygonStamped, '/'+scope+'_costmap/published_footprint',
                lambda msg, key=scope: self.costmap_stamps.update({key:
                    (msg.header.stamp.sec + msg.header.stamp.nanosec/1e9, time.monotonic())}),
                qos_profile_sensor_data)
        self.create_timer(0.05, self.check, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.5, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def scan(self, msg):
        # Infinity represents clear space, NaN/empty scans are not usable.
        if msg.ranges and any(not math.isnan(v) for v in msg.ranges):
            self.scan_received = time.monotonic()
            self.scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if self.predictive_enabled:
                self.pending_scan = msg
                self.track(msg)

    @staticmethod
    def pose(transform):
        q = transform.rotation
        return (transform.translation.x, transform.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def track(self, msg):
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            sensor = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, stamp)
            robot = self.tf_buffer.lookup_transform('odom', 'base_footprint', stamp)
        except TransformException:
            # TF can arrive one transport cycle after its matching scan.
            # Retry briefly instead of destroying every in-progress track.
            self.dynamic_status = {'reason': 'tracking_tf_pending'}
            return
        self.pending_scan = None
        self.robot_pose = self.pose(robot.transform)
        self.tracking_stamp = self.scan_stamp
        observations = clusters(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, self.pose(sensor.transform))
        self.tracker.update(self.scan_stamp, observations)

    def filter_dynamic(self, msg):
        if not self.predictive_enabled or self.tracking_stamp is None:
            return msg
        now = time.monotonic()
        age = self.get_clock().now().nanoseconds/1e9 - self.tracking_stamp
        if not -0.1 <= age < 0.25:
            self.dynamic_status = {'reason': 'prediction_expired'}
            return msg
        try:
            robot = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            robot_age = self.get_clock().now().nanoseconds/1e9 - (
                robot.header.stamp.sec+robot.header.stamp.nanosec/1e9)
            if not -0.1 <= robot_age < 0.25:
                self.dynamic_status = {'reason': 'robot_tf_expired'}
                return msg
        except TransformException:
            self.dynamic_status = {'reason': 'tracking_tf_unavailable'}
            return msg
        risk = self.tracker.collision(self.pose(robot.transform), msg.linear.x, msg.angular.z,
                                      observation_age=max(0.0, age))
        tracks = [{'id': t.ident, 'velocity': [round(v, 3) for v in t.velocity],
                   'center': [round(v, 3) for v in t.center], 'span': round(t.span, 3),
                   'residual': round(t.residual, 4)} for t in self.tracker.tracks if t.stable]
        if risk:
            self.hold_until = now + 0.35
        held = risk is not None or now < self.hold_until
        self.dynamic_status = {'reason': 'predicted_stop' if held else 'clear',
                               'risk': risk, 'tracks': tracks}
        if held != self.prediction_stopped:
            self.prediction_stopped = held
            self.get_logger().info('Dynamic prediction: '+json.dumps(self.dynamic_status))
            self.status_pub.publish(String(data=json.dumps(self.dynamic_status)))
        return Twist() if held else msg

    def status(self):
        self.status_pub.publish(String(data=json.dumps(self.dynamic_status)))

    def healthy(self):
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds / 1e9
        return (self.scan_received is not None and self.command_received is not None
                and now - self.scan_received < 0.6 and now - self.command_received < 0.3
                and -0.1 <= sim - self.scan_stamp < 0.6
                and (not self.require_costmap_clock or
                     all(key in self.costmap_stamps and
                         -0.1 <= sim - self.costmap_stamps[key][0] < 1.0 and
                         now - self.costmap_stamps[key][1] < 1.0
                         for key in ('local', 'global'))))

    def command(self, msg):
        self.command_received = time.monotonic()
        self.pub.publish(self.filter_dynamic(msg) if self.healthy() else Twist())

    def check(self):
        if self.pending_scan is not None:
            stamp = self.pending_scan.header.stamp
            age = self.get_clock().now().nanoseconds/1e9 - (stamp.sec+stamp.nanosec/1e9)
            if -0.1 <= age < 0.25:
                self.track(self.pending_scan)
            else:
                self.pending_scan = None
                self.tracker = Tracker()
                self.tracking_stamp = None
                self.dynamic_status = {'reason': 'tracking_tf_unavailable'}
        if not self.healthy():
            self.pub.publish(Twist())


def main():
    rclpy.init()
    node = ScanVelocityGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
