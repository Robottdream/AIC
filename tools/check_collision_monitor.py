"""Exercise installed Humble Collision Monitor on isolated scan/command topics.

Does not publish commands to the robot. Run from a sourced ROS environment.
"""
import json
import math
import subprocess
import time
import tempfile
from pathlib import Path
import yaml

import rclpy
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import LaserScan


def main():
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/yzbot/bot_navigation/param/collision_monitor.yaml').read_text())
    config_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump({'/collision_test/collision_monitor': config['collision_monitor']}, config_file)
    config_file.close()
    args = ['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor',
            '--ros-args', '--params-file', config_file.name,
            '-r', '__node:=collision_monitor', '-r', '__ns:=/collision_test',
            '-p', 'use_sim_time:=false', '-p', 'base_shift_correction:=false',
            '-p', 'odom_frame_id:=base_footprint',
            '-p', 'cmd_vel_in_topic:=/collision_test/input',
            '-p', 'cmd_vel_out_topic:=/collision_test/guarded',
            '-p', 'scan.topic:=/collision_test/scan',
            '-p', 'FootprintApproach.footprint_topic:=/collision_test/footprint']
    logdir = root / 'log/collision-20260930'
    logdir.mkdir(parents=True, exist_ok=True)
    log = (logdir / 'isolated-monitor.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
    guard = subprocess.Popen([
        'python3', str(root / 'src/yzbot/bot_navigation/scripts/scan_velocity_guard.py'),
        '--ros-args', '-r', '__ns:=/collision_test', '-p', 'use_sim_time:=false',
        '-r', '/local_costmap/published_footprint:=/collision_test/local_clock',
        '-r', '/global_costmap/published_footprint:=/collision_test/global_clock',
        '-r', '/cmd_vel_guarded:=/collision_test/guarded',
        '-r', '/cmd_vel:=/collision_test/output',
        '-r', '/scan:=/collision_test/scan'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('collision_test_driver')
    outputs = []
    node.create_subscription(Twist, '/collision_test/output',
                             lambda m: outputs.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
    scan_pub = node.create_publisher(LaserScan, '/collision_test/scan', 10)
    cmd_pub = node.create_publisher(Twist, '/collision_test/input', 10)
    footprint_pub = node.create_publisher(PolygonStamped, '/collision_test/footprint', 10)
    clocks = {key: node.create_publisher(PolygonStamped, '/collision_test/'+key+'_clock', 10)
              for key in ('local', 'global')}
    client = node.create_client(ChangeState, '/collision_test/collision_monitor/change_state')
    report = {}
    try:
        assert client.wait_for_service(timeout_sec=15), 'Lifecycle service missing'
        for transition in (1, 3):
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.result() and future.result().success, 'Lifecycle transition failed'
        for name, distance, expected in [('clear', 2.0, 0.6), ('approach', 0.6, 0.135),
                                         ('stop', 0.37, 0.0), ('clear_again', 2.0, 0.6),
                                         ('stale_scan_guarded', None, 0.0),
                                         ('rotate_clear', 0.6, 0.0),
                                         ('local_clock_frozen', 2.0, 0.0),
                                         ('global_clock_frozen_rotate', 2.0, 0.0),
                                         ('clocks_recovered', 2.0, 0.6),
                                         ('monitor_lost', 2.0, 0.0)]:
            if name == 'monitor_lost':
                process.terminate()
                process.wait(timeout=5)
            outputs.clear()
            start = time.monotonic()
            while time.monotonic() - start < 1.5:
                for key, pub in clocks.items():
                    heartbeat = PolygonStamped()
                    stamp = node.get_clock().now().nanoseconds / 1e9
                    if name.startswith(key+'_clock_frozen'):
                        stamp -= 5.0
                    heartbeat.header.stamp.sec = int(stamp)
                    heartbeat.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
                    pub.publish(heartbeat)
                if distance is not None:
                    footprint = PolygonStamped()
                    footprint.header.frame_id = 'base_footprint'
                    footprint.header.stamp = node.get_clock().now().to_msg()
                    footprint.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in
                        [(0.32, 0.276), (0.32, -0.276), (-0.26, -0.276), (-0.26, 0.276)]]
                    footprint_pub.publish(footprint)
                    scan = LaserScan()
                    scan.header.frame_id = 'base_footprint'
                    scan.header.stamp = node.get_clock().now().to_msg()
                    scan.angle_min = -0.02
                    scan.angle_max = 0.02
                    scan.angle_increment = 0.01
                    scan.range_min = 0.01
                    scan.range_max = 10.0
                    scan.ranges = [distance] * 5
                    scan_pub.publish(scan)
                cmd = Twist()
                rotating = name in ('rotate_clear', 'global_clock_frozen_rotate')
                cmd.linear.x = 0.0 if rotating else 0.6
                cmd.angular.z = 0.6 if rotating else 0.0
                cmd_pub.publish(cmd)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.02)
            values = [v for t, v, w in outputs if t > start + 1.0]
            assert values, f'{name}: no output'
            assert all(math.isclose(v, expected, abs_tol=1e-5) for v in values), (name, values)
            report[name] = {'expected': expected, 'observed': values[-1], 'pass': True}
            if name == 'rotate_clear':
                angular = [w for t, v, w in outputs if t > start + 1.0]
                assert all(math.isclose(w, 0.6, abs_tol=1e-5) for w in angular), angular
                report[name]['angular_velocity'] = angular[-1]
            if name == 'global_clock_frozen_rotate':
                assert all(w == 0.0 for t, v, w in outputs if t > start+1.0)
        (logdir / 'monitor-check.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        guard.terminate()
        try:
            guard.wait(timeout=5)
        except subprocess.TimeoutExpired:
            guard.kill()
            guard.wait()
        Path(config_file.name).unlink()


if __name__ == '__main__':
    main()
