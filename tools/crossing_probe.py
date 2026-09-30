"""Controlled obstacle3 crossing regression after a fresh one-click start.

Repositions obstacle3 clear of the robot, then requests a crossing route.
Writes raw/final velocity, obstacle pose, odometry and IMU; no truth fed to Nav2.
"""
import json
import math
import subprocess
import time
import argparse
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('/mnt/e/workspace/AIC/log/collision-20260930/crossing'))
    parser.add_argument('--return-route', action='store_true')
    parser.add_argument('--approach-route', action='store_true')
    parser.add_argument('--offset', type=float, default=2.4)
    args = parser.parse_args()
    assert not (args.return_route and args.approach_route), 'Choose one route'
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('crossing_regression')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    state = {}
    subscriptions = []
    for topic, key in [('/cmd_vel_unfiltered', 'raw'), ('/cmd_vel', 'final')]:
        subscriptions.append(node.create_subscription(Twist, topic,
            lambda m, k=key: state.update({k: [m.linear.x, m.angular.z]}), 10))
    subscriptions.append(node.create_subscription(Pose, '/obstacle3/current_pose',
        lambda m: state.update(obstacle=[m.position.x, m.position.y, m.position.z]), 10))
    subscriptions.append(node.create_subscription(Odometry, '/odom',
        lambda m: state.update(robot=[m.pose.pose.position.x, m.pose.pose.position.y,
                                     m.pose.pose.position.z], actual=m.twist.twist.linear.x),
        qos_profile_sensor_data))
    def imu(m):
        q = m.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
        state['tilt_deg'] = [math.degrees(roll), math.degrees(pitch)]
    subscriptions.append(node.create_subscription(Imu, '/imu', imu, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String, '/nav_status',
        lambda m: state.update(status=m.data), 10))
    subscriptions.append(node.create_subscription(String, '/navigation/dynamic_guard',
        lambda m: state.update(dynamic=json.loads(m.data)), 10))
    pub = node.create_publisher(String, '/manual_nav_target', 10)
    deadline = time.monotonic()+15
    while ('obstacle' not in state or 'robot' not in state or pub.get_subscription_count() < 1) and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    assert 'obstacle' in state and 'robot' in state, 'Missing telemetry'
    expected = (-4.5,0.) if args.return_route else (0.,0.)
    assert math.dist(state['robot'][:2], expected) < .3, 'Robot not at expected starting point'
    # Choose the opposite side of the crossing from the plugin's current target.
    def direction():
        lines = Path('/home/polarbear/ws_aic/log/run/01_gazebo.log').read_text().splitlines()
        return next(line for line in reversed(lines) if '[obstacle3] Time:' in line)
    latest = direction()
    if args.approach_route:
        deadline = time.monotonic()+25
        while 'ToEND:' not in latest and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            time.sleep(.2)
            latest = direction()
        assert 'ToEND:' in latest, 'Obstacle did not enter approaching phase'
    y = 3.0 if args.approach_route else (args.offset if 'ToEND:' in latest else -args.offset)
    subprocess.run(['gz', 'model', '-m', 'obstacle3', '-x', '-2.5', '-y', str(y),
                    '-z', '.375', '-R', '0', '-P', '0', '-Y', '0'], check=True, timeout=10)
    pub.publish(String(data=json.dumps({'type':'custom',
        'x':-2.5 if args.approach_route else (0.0 if args.return_route else -4.5),
        'y':2.0 if args.approach_route else 0.0,
        'yaw':math.pi/2 if args.approach_route else (0.0 if args.return_route else math.pi)})))
    start = time.monotonic()
    rows = []
    with (out/'trace.jsonl').open('w') as f:
        while time.monotonic()-start < 90:
            until = time.monotonic()+.05
            while time.monotonic()<until:
                rclpy.spin_once(node, timeout_sec=.01)
            row = dict(wall=time.monotonic()-start, sim=node.get_clock().now().nanoseconds/1e9, **state)
            rows.append(row)
            f.write(json.dumps(row)+'\n')
            if state.get('status') in ('succeeded', 'failed: aborted', 'failed: no_path'):
                break
    summary = {'status':state.get('status', 'timeout'), 'seconds':time.monotonic()-start,
               'route':'approach' if args.approach_route else ('return' if args.return_route else 'crossing'),
               'initial_obstacle_y':y,
               'max_actual_speed':max(abs(r.get('actual',0.)) for r in rows),
               'min_center_distance':min(math.dist(r['robot'][:2],r['obstacle'][:2]) for r in rows),
               'prediction_stop_samples':sum(r.get('dynamic',{}).get('reason')=='predicted_stop' for r in rows),
               'max_tilt_deg':max(max(abs(v) for v in r.get('tilt_deg',[0])) for r in rows),
               'limited_samples':sum(abs(r.get('raw',[0])[0])>.05 and
                    abs(r.get('final',[0])[0]) < abs(r['raw'][0])*.8 for r in rows),
               'stopped_samples_with_drive_request':sum(abs(r.get('raw',[0])[0])>.05 and
                    abs(r.get('final',[0])[0])<.001 for r in rows)}
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
