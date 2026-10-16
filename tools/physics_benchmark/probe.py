"""ROS-side command probe; invoked by run.py inside a sourced WSL environment."""
from pathlib import Path
import argparse, json, time, math, copy, xml.etree.ElementTree as ET
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from gazebo_msgs.srv import GetEntityState, SetEntityState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, WrenchStamped
parser = argparse.ArgumentParser()
parser.add_argument('--speed', type=float, required=True)
parser.add_argument('--folder', required=True)
parser.add_argument('--omega', type=float, default=2.0)
parser.add_argument('--hold', type=float, default=3.2)
parser.add_argument('--brake', type=float, default=2.2)
parser.add_argument('--expected-body-speed-limit',type=float,default=3.5)
parser.add_argument('--expected-drive', choices=['wheel', 'body_force'], default='wheel')
parser.add_argument('--expected-torque', type=float, default=1000.0)
parser.add_argument('--expected-mu', type=float, default=1.0)
args = parser.parse_args()
out = Path(args.folder)
rclpy.init(args=[])
node = rclpy.create_node('linear_speed_limit_audit')
node.set_parameters([Parameter('use_sim_time', value=True)])
state = {}
samples = []
commands = []
wrenches = []
issued = []
results = []
mode = 'smoothed'
active_trial = {}
last_emit = -1000000000.0
iv = 0.0
iw = 0.0

def odom(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
    state.update(sim=m.header.stamp.sec + m.header.stamp.nanosec / 1000000000.0, x=p.x, y=p.y, z=p.z, yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), v=m.twist.twist.linear.x, w=m.twist.twist.angular.z, tilt_deg=math.degrees(max(abs(roll), abs(pitch))))
    samples.append(dict(state, wall=time.monotonic()))

def command(m):
    commands.append(dict(sim=state.get('sim', 0), wall=time.monotonic(), v=m.linear.x, w=m.angular.z))
def wrench(m):
    wrenches.append(dict(sim=m.header.stamp.sec+m.header.stamp.nanosec/1e9, frame=m.header.frame_id, fx=m.wrench.force.x, fy=m.wrench.force.y, tz=m.wrench.torque.z))
node.create_subscription(WrenchStamped, '/body_drive/wrench', wrench, 100)
node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
node.create_subscription(Twist, '/cmd_vel', command, 100)
nav_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
base_pub = node.create_publisher(Twist, '/cmd_vel', 10)

def emit(v=0.0, w=0.0):
    global iv, iw, last_emit
    iv = float(v)
    iw = float(w)
    last_emit = state.get('sim', 0)
    m = Twist()
    m.linear.x = iv
    m.angular.z = iw
    (nav_pub if mode == 'smoothed' else base_pub).publish(m)
    issued.append(dict(sim=last_emit, wall=time.monotonic(), v=iv, w=iw, mode=mode))

def spin_for(seconds, v=0.0, w=0.0):
    began = state['sim']
    deadline = time.monotonic() + max(20, seconds * 6)
    while state['sim'] - began < seconds:
        if time.monotonic() > deadline:
            raise TimeoutError('Simulation clock stopped')
        if state['sim'] - last_emit >= 0.025:
            dt = min(0.1, max(0.001, state['sim'] - last_emit))
            emit(v if mode == 'smoothed' else iv + max(-5 * dt, min(3.5 * dt, v - iv)), w if mode == 'smoothed' else iw + max(-10 * dt, min(10 * dt, w - iw)))
        rclpy.spin_once(node, timeout_sec=0.005)
        if not all(math.isfinite(state[k]) for k in ['x','y','z','yaw','v','w','tilt_deg']):
            raise RuntimeError('Non-finite odometry')
        if state['tilt_deg'] > 5 or abs(state['x']) > 30 or abs(state['y']) > 11 or (state['z'] > 0.3):
            raise RuntimeError('Instability or field boundary reached')

def call_once(service, kind, request):
    c = node.create_client(kind, service)
    try:
        assert c.wait_for_service(timeout_sec=10 if kind is GetParameters else 30), service
        f = c.call_async(request)
        rclpy.spin_until_future_complete(node, f, timeout_sec=10 if kind is GetParameters else 30)
        assert f.done() and f.result() is not None, service
        return f.result()
    finally:
        node.destroy_client(c)

def call(service, kind, request):
    # Read-only parameter queries are safe to retry. State-changing requests run once.
    attempts = 3 if kind is GetParameters else 1
    for attempt in range(1, attempts + 1):
        try:
            return call_once(service, kind, request)
        except AssertionError as exc:
            if attempts == 1:
                raise
            path = out / 'parameter-query-retries.json'
            records = json.loads(path.read_text()) if path.exists() else []
            records.append(dict(service=service, attempt=attempt, error=repr(exc), wall=time.monotonic()))
            path.write_text(json.dumps(records, indent=2))
            print('PARAMETER_QUERY_RETRY', service, attempt, flush=True)
            if attempt == attempts:
                raise

def reset():
    spin_for(0.8)
    request = GetEntityState.Request()
    request.name = 'six_arm'
    request.reference_frame = 'world'
    response = call('/get_entity_state', GetEntityState, request)
    assert response.success
    request = SetEntityState.Request()
    request.state = copy.deepcopy(response.state)
    request.state.name = 'six_arm'
    request.state.reference_frame = 'world'
    request.state.pose.position.x = 0.0
    request.state.pose.position.y = 0.0
    request.state.pose.orientation.x = 0.0
    request.state.pose.orientation.y = 0.0
    request.state.pose.orientation.z = 0.0
    request.state.pose.orientation.w = 1.0
    request.state.twist = Twist()
    assert call('/set_entity_state', SetEntityState, request).success
    emit()
    spin_for(0.7)

def trial(name, v, w, duration, repeat):
    global active_trial
    reset()
    samples.clear()
    commands.clear()
    wrenches.clear()
    issued.clear()
    start = state['sim']
    active_trial = dict(name=name, target_v=v, target_w=w, start_sim=start, planned_duration=duration, stop_request_sim=None)
    spin_for(duration, v, w)
    stop = state['sim']
    active_trial['stop_request_sim'] = stop
    spin_for(args.brake)
    if args.expected_drive == 'body_force':
        assert len(wrenches) > 10, 'No applied-wrench telemetry'
        assert all(math.hypot(r['fx'],r['fy']) <= 250.0001 and abs(r['tz']) <= 100.0001 for r in wrenches)
    window = [r for r in commands if start + duration - 0.5 <= r['sim'] <= stop - 0.1]
    assert len(window) >= 5, 'Too few actual command observations'
    expected_v = args.speed
    assert max((abs(r['v'] - expected_v) for r in window)) < 0.05, (name, 'Input plateau mismatch', window)
    assert max((abs(r['w'] - w) for r in window)) < 0.05, (name, 'Angular input mismatch')
    trace = dict(name=name, repeat=repeat, mode=mode, target_v=v, target_w=w, start_sim=start, stop_request_sim=stop, window_start=start + duration - 0.5, window_end=stop - 0.1, motion=samples.copy(), commands=commands.copy(), issued=issued.copy(), wrenches=wrenches.copy(), input_valid=True)
    filename = f'{name}-r{repeat}.json'
    (out / filename).write_text(json.dumps(trace))
    row = {k: trace[k] for k in ['name', 'repeat', 'mode', 'target_v', 'target_w', 'start_sim', 'stop_request_sim', 'window_start', 'window_end', 'input_valid']}
    row['file'] = filename
    row['max_tilt_deg'] = max((s['tilt_deg'] for s in samples))
    results.append(row)
    (out / 'trials.json').write_text(json.dumps(results, indent=2))
    print('TRIAL', args.speed, name, repeat, 'input valid; peak tilt', round(row['max_tilt_deg'], 4), flush=True)
def servo_checks():
    global mode
    assert call('/velocity_smoother/change_state', ChangeState, ChangeState.Request(transition=Transition(id=Transition.TRANSITION_DEACTIVATE))).success
    mode = 'direct'
    checks = []
    for name, v, w in [('in-place',0.,2.), ('reverse',-1.,0.)]:
        reset()
        samples.clear()
        spin_for(2., v, w)
        late = [r for r in samples if state['sim']-.4 <= r['sim']]
        actual_v = sum(r['v'] for r in late)/len(late)
        actual_w = sum(r['w'] for r in late)/len(late)
        checks.append(dict(name=name, target_v=v, target_w=w, actual_v=actual_v, actual_w=actual_w, passed=abs(actual_v-v)<.15 and abs(actual_w-w)<.15))
        spin_for(1.)
    reset()
    spin_for(1.5, 1., 0.)
    last_command = last_emit
    start = state['sim']
    deadline = time.monotonic()+20
    # Deliberately publish nothing: bypass the inactive smoother to exercise
    # the driver's own stale-command brake, not an upstream zero command.
    while state['sim']-start < 1.8:
        if time.monotonic()>deadline: raise TimeoutError('Watchdog clock stalled')
        rclpy.spin_once(node, timeout_sec=.005)
    checks.append(dict(name='command-timeout', no_publish_sim_seconds=state['sim']-last_command, actual_v=state['v'], actual_w=state['w'], passed=abs(state['v'])<.08 and abs(state['w'])<.08))
    (out/'servo-checks.json').write_text(json.dumps(checks, indent=2))
    assert all(r['passed'] for r in checks), checks
    print('SERVO_CHECKS', json.dumps(checks), flush=True)

try:
    deadline = time.monotonic() + 45
    while (not state or not nav_pub.get_subscription_count() or (not base_pub.get_subscription_count())) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert state and nav_pub.get_subscription_count() and base_pub.get_subscription_count(), 'Odometry/command subscription missing'
    desc = call('/robot_state_publisher/get_parameters', GetParameters, GetParameters.Request(names=['robot_description'])).values[0].string_value
    (out / 'live-robot.urdf').write_text(desc)
    root = ET.fromstring(desc)
    motor = root.find("gazebo/plugin[@name='differential_drive_controller']")
    body = root.find("gazebo/plugin[@name='body_force_controller']")
    if args.expected_drive == 'wheel':
        assert motor is not None and body is None
        assert float(motor.findtext('max_wheel_torque')) == args.expected_torque
    else:
        assert body is not None and motor is None
        assert float(body.findtext('max_linear_speed')) == args.expected_body_speed_limit
        assert args.expected_body_speed_limit >= args.speed
        for key, expected in [('forward_limit',150), ('lateral_limit',200), ('yaw_limit',100), ('command_timeout',.5)]:
            assert float(body.findtext(key)) == expected
    (out / 'drive-verification.json').write_text(json.dumps(dict(drive=args.expected_drive, mutually_exclusive=True), indent=2))
    for wheel in ['left_wheel_link', 'right_wheel_link']:
        for mu in ['mu1', 'mu2']:
            assert float(root.find(f"gazebo[@reference='{wheel}']/{mu}").text) == args.expected_mu
    runtime = {}
    for (target, names) in [('/velocity_smoother', ['max_velocity', 'max_accel', 'max_decel', 'feedback', 'scale_velocities']), ('/controller_server', ['FollowPath.desired_linear_vel'])]:
        reply = call(target + '/get_parameters', GetParameters, GetParameters.Request(names=names))
        runtime[target] = {k: {'type': v.type, 'double_array': list(v.double_array_value), 'double': v.double_value, 'string': v.string_value, 'bool': v.bool_value} for (k, v) in zip(names, reply.values)}
    (out / 'runtime-parameters.json').write_text(json.dumps(runtime, indent=2))
    assert runtime['/velocity_smoother']['max_velocity']['double_array'] == [args.speed, 0.0, 4.0]
    assert runtime['/controller_server']['FollowPath.desired_linear_vel']['double'] == args.speed
    assert call('/velocity_smoother/get_state', GetState, GetState.Request()).current_state.id == State.PRIMARY_STATE_ACTIVE
    for repeat in [1]:
        trial('straight', args.speed, 0.0, args.hold, repeat)
        trial('arc-w2', args.speed, args.omega, args.hold, repeat)
    if args.expected_drive == 'body_force':
        servo_checks()
except BaseException as exc:
    (out / 'failed-trial.json').write_text(json.dumps(dict(active_trial, motion=samples, commands=commands, issued=issued, error=repr(exc)), indent=2))
    (out / 'error.json').write_text(json.dumps(dict(error=repr(exc), state=state, mode=mode), indent=2))
    raise
finally:
    for _ in range(10):
        emit()
        rclpy.spin_once(node, timeout_sec=0.01)
    (out / 'last-motion.json').write_text(json.dumps(samples))
    node.destroy_node()
    rclpy.shutdown()
