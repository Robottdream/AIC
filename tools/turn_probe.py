"""Measure same-position Nav2 yaw goals through the full collision safety chain."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--direction', type=float, default=1.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('turn_probe')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    state = {}
    def odom(m):
        q = m.pose.pose.orientation
        state.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                     yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                     actual=m.twist.twist.angular.z, z=m.pose.pose.position.z)
    subs = [node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)]
    for topic in ('cmd_vel_nav', 'cmd_vel_unfiltered', 'cmd_vel'):
        subs.append(node.create_subscription(Twist, '/'+topic,
            lambda m, k=topic: state.update({k:m.angular.z}), 10))
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    def wait(future, seconds):
        deadline = time.monotonic()+seconds
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not future.done():
            raise TimeoutError('Nav2 action timeout')
        return future.result()
    try:
        assert client.wait_for_server(timeout_sec=10)
        deadline = time.monotonic()+10
        while 'yaw' not in state and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert 'yaw' in state, 'No odometry'
        initial = dict(state)
        target = initial['yaw'] + args.direction*math.pi/2
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = initial['x']
        goal.pose.pose.position.y = initial['y']
        goal.pose.pose.orientation.z = math.sin(target/2)
        goal.pose.pose.orientation.w = math.cos(target/2)
        start = time.monotonic()
        handle = wait(client.send_goal_async(goal), 10)
        assert handle.accepted, 'Goal rejected'
        result = handle.get_result_async()
        rows = []
        deadline = start+35
        while not result.done() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            rows.append(dict(wall=time.monotonic()-start, **state))
        if not result.done():
            wait(handle.cancel_goal_async(), 5)
        error = math.atan2(math.sin(target-state['yaw']), math.cos(target-state['yaw']))
        summary = dict(status=result.result().status if result.done() else 'timeout',
                       seconds=time.monotonic()-start, final_yaw_error=error,
                       position_drift=math.hypot(state['x']-initial['x'], state['y']-initial['y']),
                       peak_actual=max(abs(r.get('actual', 0)) for r in rows),
                       peaks={k:max(abs(r.get(k,0)) for r in rows)
                              for k in ('cmd_vel_nav','cmd_vel_unfiltered','cmd_vel')})
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        (args.output/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        print(json.dumps(summary), flush=True)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
