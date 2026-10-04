"""Exercise the installed DWB controller in an isolated ROS domain, without a robot.

The controller receives the same odometry/global path in all cases; only the
timestamped obstacle forecast changes. No command is connected to Gazebo.
"""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock as ClockMessage
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

OUT = Path(os.environ.get('AIC_DWB_CHECK_OUTPUT', '/home/polarbear/aic_predictive_planner_evidence_20261003/isolated'))
OUT.mkdir(parents=True, exist_ok=True)
assert os.environ.get('ROS_DOMAIN_ID') == '164', 'This test must not share the running robot domain'
native = Path('/home/polarbear/ws_aic_mecanum_20261002')
config_path = Path(os.environ.get('AIC_DWB_CHECK_CONFIG', str(native/'src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml')))
params = yaml.safe_load(config_path.read_text())
controller = copy.deepcopy(params['controller_server']['ros__parameters'])
controller.update(use_sim_time=True, odom_topic='/odom')
controller['progress_checker']['movement_time_allowance'] = 60.0
costmap = copy.deepcopy(params['local_costmap']['local_costmap']['ros__parameters'])
costmap.update(use_sim_time=True, robot_base_frame='base_link', width=6, height=6,
               track_unknown_space=False, plugins=['inflation_layer'], update_frequency=20.0)
config = {'controller_server': {'ros__parameters': controller},
          'local_costmap': {'local_costmap': {'ros__parameters': costmap}}}
(OUT/'params.yaml').write_text(yaml.safe_dump(config))
rclpy.init()
n = rclpy.create_node('isolated_predictive_dwb_probe')
n.set_parameters([Parameter('use_sim_time', value=True)])
clock_pub = n.create_publisher(ClockMessage, '/clock', 10)
clock_start = time.monotonic()
tf = TransformBroadcaster(n)
odom = n.create_publisher(Odometry, '/odom', 10)
forecast = n.create_publisher(String, '/navigation/predicted_obstacles', 1)
state = {'mode': 'baseline', 'commands': [], 'diagnostics': []}
def tick():
    clock_msg = ClockMessage()
    seconds = 100.0 + time.monotonic() - clock_start
    clock_msg.clock.sec = int(seconds)
    clock_msg.clock.nanosec = int((seconds-int(seconds))*1e9)
    clock_pub.publish(clock_msg)
    stamp = n.get_clock().now().to_msg()
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    msg = Odometry()
    msg.header = transform.header
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.0 if state['mode']=='near_goal' else 0.5
    odom.publish(msg)
    data = {'version': 1, 'enabled': True, 'frame': 'odom',
            'stamp': stamp.sec+stamp.nanosec/1e9, 'tracks': []}
    if state['mode'] not in ('baseline','near_goal'):
        away = state['mode'] == 'away'
        data['tracks'] = [{'id': 1, 'center': [2.0, 2.0] if away else [1.25, 1.3],
                           'velocity': [0.0, 0.75 if away else -0.75], 'radius': 0.25}]
        if state['mode'] == 'crossing_slow':
            data['tracks'] = [{'id': 2, 'center': [1.5, .8],
                               'velocity': [0.0, -.35], 'radius': .18}]
        if state['mode'] == 'approaching':
            data['tracks'] = [{'id': 3, 'center': [.8, 0.0],
                               'velocity': [-.15, 0.0], 'radius': .18}]
        if state['mode'] in ('known_linear', 'known_turn'):
            data['tracks'] = [{'id': 4, 'center': [.8, 0.0],
                               'velocity': [.15, 0.0], 'radius': .18}]
            if state['mode'] == 'known_turn':
                data['tracks'][0]['samples'] = [
                    {'t': 0., 'center': [.8, 0.]}, {'t': .7, 'center': [.9, 0.]},
                    {'t': 2.2, 'center': [.45, 0.]}, {'t': 4., 'center': [.99, 0.]}]
        if state['mode'] == 'stale':
            data['stamp'] -= 2.0
    forecast.publish(String(data=json.dumps(data)))
timer = n.create_timer(0.05, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
subscriptions = [
    n.create_subscription(Twist, '/probe_cmd_vel', lambda m: state['commands'].append(
        [m.linear.x, m.linear.y, m.angular.z]), 10),
    n.create_subscription(String, '/navigation/predictive_planner',
                          lambda m: state['diagnostics'].append(json.loads(m.data)), 10)]
log = (OUT/'controller.log').open('w')
process = subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server',
    '--ros-args', '--params-file', str(OUT/'params.yaml'), '-r', 'cmd_vel:=/probe_cmd_vel'],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
def spin(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Isolated controller exited; see controller.log'
        rclpy.spin_once(n, timeout_sec=0.02)
def wait(future, seconds=10):
    deadline = time.monotonic()+seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=0.02)
    assert future.done(), 'Controller operation timeout'
    return future.result()
report = {}
try:
    lifecycle = n.create_client(ChangeState, '/controller_server/change_state')
    deadline = time.monotonic()+15
    while not lifecycle.service_is_ready() and time.monotonic() < deadline:
        spin(.1)
    assert lifecycle.service_is_ready(), 'Controller lifecycle service absent'
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        request = ChangeState.Request()
        request.transition.id = transition
        assert wait(lifecycle.call_async(request)).success, 'Lifecycle transition failed'
    action = ActionClient(n, FollowPath, '/follow_path')
    spin(1)
    assert action.wait_for_server(timeout_sec=5), 'FollowPath action absent'
    for mode in ('baseline', 'crossing', 'crossing_slow', 'approaching', 'known_linear', 'known_turn', 'away', 'stale', 'near_goal'):
        state['mode'] = mode
        spin(.4)
        state['commands'] = []
        state['diagnostics'] = []
        goal = FollowPath.Goal()
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        goal.path.header.frame_id = 'odom'
        goal.path.header.stamp = n.get_clock().now().to_msg()
        for i in range(61):
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = i * .05 if mode!='near_goal' else i/60*.13
            pose.pose.position.y = 0.0 if mode!='near_goal' else i/60*.12
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)
        handle = wait(action.send_goal_async(goal))
        assert handle.accepted, 'FollowPath rejected'
        spin(3)
        report[mode] = {'commands': state['commands'][:], 'diagnostics': state['diagnostics'][:]}
        wait(handle.cancel_goal_async())
        spin(.2)
        assert report[mode]['diagnostics'], 'No actual DWB forecast diagnostics'
    base = report['baseline']['diagnostics'][-1]['chosen']
    cross = report['crossing']['diagnostics'][-1]['chosen']
    stale = report['stale']['diagnostics'][-1]['chosen']
    approach = report['approaching']['diagnostics'][-1]['chosen']
    linear = report['known_linear']['diagnostics'][-1]['chosen']
    turn = report['known_turn']['diagnostics'][-1]['chosen']
    near = report['near_goal']['diagnostics'][-1]['chosen']
    assert abs(base[1]) < .01 and base[0] > .4, base
    assert any(m['active_tracks'] > 0 and m['rejected'] > 0
               for m in report['crossing']['diagnostics']), 'Forecast never rejected a DWB candidate'
    assert abs(cross[1]-base[1]) > .02 or cross[0] < base[0]-.05, (base, cross)
    approaching_all_rejected = all(m['rejected'] == m['candidates'] and m['candidates'] > 0
                                  for m in report['approaching']['diagnostics'])
    # With a larger planning envelope this deliberately late encounter can
    # leave no legal trajectory. A zero command is the required safe outcome.
    approaching_safe_stop = approaching_all_rejected and all(abs(v) < .01 for v in approach)
    assert approach[0] < -.02 or abs(approach[1]) > .02 or approaching_safe_stop, ('No safe retreat/stop selected', approach)
    assert linear[0] >= -.01 and (turn[0] < -.02 or abs(turn[1]) > .02), (linear, turn)
    assert abs(stale[0]-base[0]) < .01 and abs(stale[1]-base[1]) < .01, (base, stale)
    assert all(m['active_tracks'] == 0 for m in report['stale']['diagnostics']), 'Expired obstacle persists'
    if controller['FollowPath'].get('trajectory_generator_name')=='bot_navigation::FineOmniTrajectoryGenerator':
        assert near[0]>0 and near[1]>0 and math.hypot(*near[:2])<.12,('Missing low-speed goal approach',near)
    report['summary'] = {'baseline': base, 'crossing': cross, 'approaching': approach, 'stale': stale,
                         'approaching_all_candidates_rejected_stop': approaching_safe_stop,
                         'near_goal': near,
                         'known_linear': linear, 'known_turn': turn,
                         'crossing_rejected': max(m['rejected'] for m in report['crossing']['diagnostics']),
                         'isolated_domain': 164, 'robot_motion': False, 'passed': True}
    print(json.dumps(report['summary'], indent=2), flush=True)
finally:
    (OUT/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    import signal
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
    log.close()
    n.destroy_node()
    rclpy.shutdown()
