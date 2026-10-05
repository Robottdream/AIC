"""Compare actual Nav2 straight/return/turn motion at a fresh spawn."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PolygonStamped
from nav2_msgs.action import NavigateToPose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('aic_motion_probe')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    state = {}
    samples = []
    instability = {}

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        state.update(sim=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                     x=p.x, y=p.y, z=p.z, yaw=yaw,
                     v=math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
                     w=msg.twist.twist.angular.z,
                     tilt_deg=math.degrees(max(abs(roll), abs(pitch))))
        samples.append(dict(wall=time.monotonic(), **state))
        if state['tilt_deg'] > 5.0 or abs(p.x) > 20 or abs(p.y) > 20 or p.z > .3:
            if not instability:
                instability.update(state)

    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel',
        lambda m: state.update(cmd_v=m.linear.x, cmd_w=m.angular.z), qos_profile_sensor_data)
    def amcl(msg):
        p = msg.pose.pose.position
        state.update(amcl_x=p.x, amcl_y=p.y,
                     amcl_sim=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl, qos_profile_sensor_data)
    def footprint(msg):
        points = msg.polygon.points
        if points:
            state.update(local_pose_sim=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                local_x=sum(p.x for p in points)/len(points), local_y=sum(p.y for p in points)/len(points))
    node.create_subscription(PolygonStamped, '/local_costmap/published_footprint', footprint, qos_profile_sensor_data)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    def wait(future, seconds):
        rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
        if not future.done():
            raise TimeoutError('Navigation response timed out')
        return future.result()

    def check_motion(name, handle):
        if not instability:
            return
        (args.output/(name+'.json')).write_text(json.dumps(samples, indent=2)+'\n')
        (args.output/'instability.json').write_text(json.dumps(instability, indent=2)+'\n')
        handle.cancel_goal_async()
        print('ABORT_INSTABILITY', instability, flush=True)
        subprocess.run(['python3', str(Path(__file__).resolve().parent/'project_launcher.py'),
                        'stop'], check=True, timeout=30)
        raise RuntimeError('Motion probe aborted for instability')

    try:
        assert client.wait_for_server(timeout_sec=20), 'Nav2 missing'
        until = time.monotonic()+15
        while 'x' not in state and time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.1)
        assert 'x' in state and math.hypot(state['x'], state['y'])<.3, 'Probe requires fresh origin spawn'
        summaries = {}
        for name, x, y, yaw in [('straight', 5., 0., 0.), ('return', 0., 0., 0.),
                                ('turn90', 0., 0., math.pi/2), ('turnback', 0., 0., 0.)]:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            samples.clear()
            began, sim_began = time.monotonic(), state['sim']
            handle = wait(client.send_goal_async(goal), 10)
            assert handle.accepted, name
            result = handle.get_result_async()
            until = began+90
            while not result.done() and time.monotonic()<until:
                rclpy.spin_once(node, timeout_sec=.05)
                check_motion(name, handle)
            if not result.done():
                wait(handle.cancel_goal_async(), 5)
                raise TimeoutError(name)
            wall_seconds, sim_seconds = time.monotonic()-began, state['sim']-sim_began
            settle = time.monotonic()+1.5
            while time.monotonic()<settle:
                rclpy.spin_once(node, timeout_sec=.05)
                check_motion(name, handle)
            peak = max(s['v'] for s in samples)
            accelerating = [s for s in samples if s['v']>=peak*.9] if peak>.2 else []
            summaries[name] = dict(status=result.result().status, wall_seconds=wall_seconds,
                sim_seconds=sim_seconds, peak_mps=peak, peak_radps=max(abs(s['w']) for s in samples),
                max_tilt_deg=max(s['tilt_deg'] for s in samples),
                overshoot_m=max(0., max(s['x'] for s in samples)-x) if name=='straight' else None,
                max_amcl_age_seconds=max((s['sim']-s['amcl_sim'] for s in samples if 'amcl_sim' in s), default=None),
                max_local_pose_age_seconds=max((s['sim']-s['local_pose_sim'] for s in samples if 'local_pose_sim' in s), default=None),
                rise_to_90_sim_seconds=accelerating[0]['sim']-sim_began if accelerating else None,
                physical_position_error=math.hypot(state['x']-x, state['y']-y),
                physical_yaw_error=abs(math.atan2(math.sin(state['yaw']-yaw), math.cos(state['yaw']-yaw))))
            (args.output/(name+'.json')).write_text(json.dumps(samples, indent=2)+'\n')
            (args.output/'summary.json').write_text(json.dumps(summaries, indent=2)+'\n')
            print(name, summaries[name], flush=True)
            assert summaries[name]['status']==4, summaries[name]
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
