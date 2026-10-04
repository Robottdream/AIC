"""Run the natural-language five-cube task; measure only Gazebo /clock time."""
import argparse
import json
import math
import os
import shutil
import subprocess
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32

p = argparse.ArgumentParser()
p.add_argument('--trial', type=int, required=True)
args = p.parse_args()
root = Path('/home/polarbear/aic_three_trials_20261003')
out = root / f'trial-{args.trial}'
out.mkdir(exist_ok=False)
native = Path('/home/polarbear/ws_aic_mecanum_20261002')
rclpy.init()
n = rclpy.create_node(f'five_cube_trial_{args.trial}')
state = {'clock': None, 'clock_resets': 0, 'peak_speed': 0.0}
events = []
started = False
completion = None
placements = []
cube = None
area = None
trace = (out / 'events.jsonl').open('w')

def event(topic, data):
    row = {'sim': state['clock'], 'topic': topic, 'data': data}
    events.append(row)
    trace.write(json.dumps(row, ensure_ascii=False) + '\n')
    trace.flush()

def clock(m):
    value = m.clock.sec + m.clock.nanosec / 1e9
    if state['clock'] is not None and value < state['clock']:
        state['clock_resets'] += 1
    state['clock'] = value

def odom(m):
    state['pose'] = [m.pose.pose.position.x, m.pose.pose.position.y]
    state['speed'] = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
    if started:
        state['peak_speed'] = max(state['peak_speed'], state['speed'])

def message(topic, data):
    global cube, area
    if topic == '/current_target_cube': cube = data
    if topic == '/target_area': area = data
    if topic == '/cur': state['stage'] = data
    if topic == '/chat': state['parsed'] = json.loads(data)
    if not started: return
    event(topic, data)
    if topic == '/arm_status' and data == 'place_succeeded':
        placements.append({'cube': cube, 'area': area, 'sim': state['clock']})
        print('PLACED ' + json.dumps(placements[-1]), flush=True)

def log(m):
    global completion
    if not started or m.name != 'main_controller_node': return
    event('/rosout/main', m.msg)
    if '所有优化任务执行完成' in m.msg:
        completion = state['clock']

subs = [n.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
        n.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
        n.create_subscription(Log, '/rosout', log, 100)]
for topic in ('/chat', '/current_target_cube', '/arm_status', '/nav_status', '/manual_nav_target'):
    subs.append(n.create_subscription(String, topic, lambda m, t=topic: message(t, m.data), 100))
for topic in ('/cur', '/target_area', '/number_pick'):
    subs.append(n.create_subscription(Int32, topic, lambda m, t=topic: message(t, m.data), 100))
pub = n.create_publisher(String, '/command', 10)

def spin(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until: rclpy.spin_once(n, timeout_sec=.02)

def poses():
    env = dict(os.environ, GAZEBO_IP='127.0.0.1')
    snapshot = root / 'pose_snapshot'
    if snapshot.exists():
        return json.loads(subprocess.check_output([str(snapshot)], text=True, timeout=15, env=env))
    values = {}
    for name in ['six_arm'] + [f'{color}_cube_{i}' for color in ('red', 'blue') for i in range(1, 6)]:
        for attempt in range(3):
            try:
                pose = [float(v) for v in subprocess.check_output(
                    ['gz', 'model', '-m', name, '-p'], text=True, timeout=8, env=env).split()]
                assert len(pose) == 6, 'Incomplete Gazebo pose response'
                values[name] = pose
                break
            except (subprocess.TimeoutExpired, AssertionError):
                print(f'POSE_READ_RETRY model={name} attempt={attempt+1}', flush=True)
                if attempt == 2: raise
                spin(.5)
    return values

report = {'trial': args.trial, 'command': '抓取3个红色物块去A区，2个蓝色物块去B区',
          'clock_source': '/clock', 'status': 'not_started'}
try:
    spin(3)
    assert state['clock'] is not None and pub.get_subscription_count() >= 1, 'Clock/parser unavailable'
    assert state.get('speed', 1) < .03, 'Robot is moving before trial'
    before = poses()
    assert math.hypot(*before['six_arm'][:2]) < .1, 'Robot was not reset'
    report['initial_poses'] = before
    # Confirm the tested production speed profile rather than assuming YAML was loaded.
    client = n.create_client(GetParameters, '/velocity_smoother/get_parameters')
    assert client.wait_for_service(timeout_sec=10)
    f = client.call_async(GetParameters.Request(names=['max_velocity', 'max_accel', 'max_decel']))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    assert f.done() and f.result()
    report['smoother'] = {key: list(v.double_array_value) for key, v in zip(
        ('max_velocity', 'max_accel', 'max_decel'), f.result().values)}
    assert report['smoother']['max_velocity'][0] == 4
    assert report['smoother']['max_accel'][0] == 2
    assert report['smoother']['max_decel'][0] == -5
    spin(.3)
    start = state['clock']
    wall_start = time.monotonic()
    report['start_sim'] = start
    started = True
    event('/command', report['command'])
    pub.publish(String(data=report['command']))
    print(f"START trial={args.trial} sim={start:.3f}", flush=True)
    next_print = wall_start
    while completion is None and state['clock'] - start < 900 and time.monotonic() - wall_start < 1800:
        spin(.05)
        assert state['clock_resets'] == 0, 'Simulation clock reset during trial'
        if time.monotonic() >= next_print:
            print(f"PROGRESS sim_elapsed={state['clock']-start:.1f} placed={len(placements)}/5 stage={state.get('stage')} pose={state.get('pose')}", flush=True)
            next_print = time.monotonic() + 25
    report.update(end_sim=completion or state['clock'], sim_seconds=(completion or state['clock'])-start,
                  placements=placements, parsed=state.get('parsed'), peak_speed=state['peak_speed'])
    expected = [('red', 0)] * 3 + [('blue', 1)] * 2
    actual = [(item['cube'].split('_')[0], item['area']) for item in placements]
    report['status'] = 'success' if completion is not None and actual == expected else 'failed_or_timeout'
    spin(2)  # Wait for arm lift/physical placement to settle; outside measured interval.
    report['final_poses'] = poses()
    zones = {0: (3.143086, -5.807858), 1: (-1.196544, -6.485499)}
    report['placement_distances_to_zone_center'] = [math.dist(report['final_poses'][item['cube']][:2], zones[item['area']]) for item in placements]
    assert report['status'] == 'success', report['status']
except Exception as exc:
    report['error'] = str(exc)
    report['status'] = 'failed'
finally:
    report['clock_resets'] = state['clock_resets']
    (out / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    shutil.copytree(native / 'log/run', out / 'run-logs', dirs_exist_ok=True)
    print('RESULT ' + json.dumps(report, ensure_ascii=False), flush=True)
    trace.close()
    n.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if report['status'] == 'success' else 1)
