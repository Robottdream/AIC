"""Actual NavFn path comparison with known-route forecast costs, isolated domain."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '165'
OUT = Path('/home/polarbear/aic_predictive_planner_evidence_20261003/global-isolated')
OUT.mkdir(parents=True, exist_ok=True)
native = Path('/home/polarbear/ws_aic_mecanum_20261002')
params = yaml.safe_load((native/'src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml').read_text())
planner = copy.deepcopy(params['planner_server']['ros__parameters'])
costmap = copy.deepcopy(params['global_costmap']['global_costmap']['ros__parameters'])
costmap.update(plugins=['known_trajectory_layer'], rolling_window=True, width=6, height=6,
               track_unknown_space=False, robot_base_frame='base_link', update_frequency=10.0)
config = {'planner_server': {'ros__parameters': planner},
          'global_costmap': {'global_costmap': {'ros__parameters': costmap}}}
(OUT/'params.yaml').write_text(yaml.safe_dump(config))
rclpy.init(); n = rclpy.create_node('known_global_plan_probe')
n.set_parameters([Parameter('use_sim_time', value=True)])
clock = n.create_publisher(ClockMessage, '/clock', 10)
tf = TransformBroadcaster(n)
forecasts = n.create_publisher(String, '/navigation/known_scene_predictions', 1)
start = time.monotonic(); state = {'predicted': False}
def tick():
    seconds = 200+time.monotonic()-start
    msg = ClockMessage(); msg.clock.sec = int(seconds)
    msg.clock.nanosec = int((seconds-int(seconds))*1e9); clock.publish(msg)
    stamp = n.get_clock().now().to_msg()
    transform = TransformStamped(); transform.header.frame_id = 'map'
    transform.header.stamp = stamp; transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.; tf.sendTransform(transform)
    tracks = []
    if state['predicted']:
        tracks = [{'id': 3, 'radius': math.hypot(.375,.375),
                   'samples': [{'t': i*.1, 'center': [0., 1.-.035*i]} for i in range(41)]}]
    payload = {'version': 1, 'enabled': True, 'frame': 'map',
               'stamp': stamp.sec+stamp.nanosec/1e9, 'tracks': tracks}
    forecasts.publish(String(data=json.dumps(payload)))
timer = n.create_timer(.05, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
log = (OUT/'planner.log').open('w')
p = subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server', '--ros-args',
    '--params-file', str(OUT/'params.yaml')], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
def spin(seconds):
    until = time.monotonic()+seconds
    while time.monotonic()<until:
        assert p.poll() is None, 'Planner exited'
        rclpy.spin_once(n, timeout_sec=.02)
def wait(future, seconds=10):
    until = time.monotonic()+seconds
    while not future.done() and time.monotonic()<until:
        rclpy.spin_once(n, timeout_sec=.02)
    assert future.done(), 'Operation timeout'
    return future.result()
report = {}
try:
    life = n.create_client(ChangeState, '/planner_server/change_state')
    until = time.monotonic()+15
    while not life.service_is_ready() and time.monotonic()<until:
        spin(.1)
    assert life.service_is_ready()
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        req = ChangeState.Request(); req.transition.id = transition
        assert wait(life.call_async(req)).success
    action = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
    assert action.wait_for_server(timeout_sec=5)
    for name, predicted in [('baseline', False), ('predicted', True), ('cleared', False)]:
        state['predicted'] = predicted; spin(1.5)
        goal = ComputePathToPose.Goal(); goal.use_start = True; goal.planner_id = 'GridBased'
        for pose, x in [(goal.start, -2.), (goal.goal, 2.)]:
            pose.header.frame_id = 'map'; pose.header.stamp = n.get_clock().now().to_msg()
            pose.pose.position.x = x; pose.pose.orientation.w = 1.
        handle = wait(action.send_goal_async(goal)); assert handle.accepted
        result = wait(handle.get_result_async())
        assert result.status == 4, (name, result.status)
        xy = [[v.pose.position.x, v.pose.position.y] for v in result.result.path.poses]
        report[name] = {'path': xy, 'max_y': max(abs(v[1]) for v in xy),
                        'length': sum(math.dist(a,b) for a,b in zip(xy,xy[1:]))}
    assert report['baseline']['max_y'] < .1
    assert report['predicted']['max_y'] > .25, 'Known forecast never influenced the actual global path'
    assert report['cleared']['max_y'] < .1, 'Forecast clearing left a ghost cost'
    report['summary'] = {name: {k:v for k,v in data.items() if k!='path'} for name,data in report.items()}
    report['summary'].update(passed=True, domain=165, robot_motion=False)
    print(json.dumps(report['summary'], indent=2), flush=True)
finally:
    (OUT/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    log.close(); n.destroy_node(); rclpy.shutdown()
