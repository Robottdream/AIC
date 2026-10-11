#!/usr/bin/env python3
"""Measure actual Gazebo obstacle attitude, including optional recovery injection."""
import argparse
import copy
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetEntityState
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--layout', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=300.)
    parser.add_argument('--inject-tilt', action='store_true')
    args = parser.parse_args()
    tracks = json.loads(Path(args.layout).read_text())['obstacles']
    rclpy.init()
    node = Node('obstacle_upright_probe')
    clock, poses, samples, last = [None], {}, [], {}
    injected, injection_time, start = [False], [None], [None]
    injection_result = [None]

    def on_clock(msg):
        clock[0] = msg.clock.sec + msg.clock.nanosec / 1e9

    def on_pose(msg, track):
        name = track['name']
        poses[name] = copy.deepcopy(msg)
        if clock[0] is None or clock[0] - last.get(name, -1.) < .02:
            return
        last[name] = clock[0]
        q, p = msg.orientation, msg.position
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        sx, sy = track['start']; ex, ey = track['end']
        dx, dy = ex-sx, ey-sy
        length = math.hypot(dx, dy)
        samples.append(dict(name=name, sim=clock[0], x=p.x, y=p.y, z=p.z,
            tilt_deg=math.degrees(max(abs(roll), abs(pitch))),
            cross_track_m=abs((p.x-sx)*dy-(p.y-sy)*dx)/length,
            along_m=((p.x-sx)*dx+(p.y-sy)*dy)/length))

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Clock, '/clock', on_clock, qos)
    for track in tracks:
        node.create_subscription(Pose, f'/{track["name"]}/current_pose',
            lambda msg, track=track: on_pose(msg, track), qos)
    setter = node.create_client(SetEntityState, '/set_entity_state')
    deadline = time.monotonic() + args.duration*3 + 60
    last_report = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if start[0] is None:
                if clock[0] is not None and len(poses) == len(tracks):
                    start[0] = clock[0]
                    print(f'START sim={start[0]:.3f}', flush=True)
                continue
            elapsed = clock[0]-start[0]
            if args.inject_tilt and not injected[0] and elapsed >= 30:
                if not setter.service_is_ready():
                    raise RuntimeError('SetEntityState unavailable for recovery test')
                request = SetEntityState.Request()
                request.state.name = 'design_obstacle_right'
                request.state.reference_frame = 'world'
                request.state.pose = copy.deepcopy(poses[request.state.name])
                request.state.pose.position.z = .32
                # A deliberate small tilt, lifted clear of the floor.
                angle = math.radians(12)
                request.state.pose.orientation.x = math.sin(angle/2)
                request.state.pose.orientation.y = 0.
                request.state.pose.orientation.z = 0.
                request.state.pose.orientation.w = math.cos(angle/2)
                future = setter.call_async(request)
                def applied(future):
                    response = future.result()
                    injection_result[0] = response.success
                    injection_time[0] = clock[0]
                future.add_done_callback(applied)
                injected[0] = True
            if time.monotonic()-last_report >= 30:
                print(f'PROGRESS sim_elapsed={elapsed:.1f}s', flush=True)
                last_report = time.monotonic()
            if elapsed >= args.duration:
                break
        else:
            raise RuntimeError('No complete simulation interval received')
        result = dict(sim_duration=clock[0]-start[0], injected=args.inject_tilt,
                      injection_success=injection_result[0], obstacles={})
        for track in tracks:
            records = [s for s in samples if s['name']==track['name']]
            normal = [s for s in records if injection_time[0] is None
                      or s['sim'] < injection_time[0]-.2
                      or s['sim'] > injection_time[0]+6]
            expected_height = track.get('height', track['size'])/2
            directions, previous = 0, 0
            for a, b in zip(records, records[1:]):
                delta = b['along_m']-a['along_m']
                sign = 1 if delta > .0001 else -1 if delta < -.0001 else 0
                if sign and previous and sign != previous:
                    directions += 1
                if sign: previous = sign
            peak = max(s['tilt_deg'] for s in normal)
            height_error = max(abs(s['z']-expected_height) for s in normal)
            result['obstacles'][track['name']] = dict(
                samples=len(records), max_normal_tilt_deg=peak,
                max_normal_height_error_m=height_error,
                max_cross_track_m=max(s['cross_track_m'] for s in normal),
                max_including_injection_tilt_deg=max(s['tilt_deg'] for s in records),
                direction_changes=directions,
                passed=peak<.1 and height_error<.005 and directions>=4)
        result['passed'] = all(o['passed'] for o in result['obstacles'].values())
        if args.inject_tilt:
            right = result['obstacles']['design_obstacle_right']
            result['passed'] &= bool(injection_result[0]) and right['max_including_injection_tilt_deg']>10
        output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2))
        output.with_suffix('.samples.json').write_text(json.dumps(samples))
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
