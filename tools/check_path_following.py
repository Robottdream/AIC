"""Before/after actual DWB replay and closed-loop curved-path test, isolated domain 167."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock as ClockMessage
from tf2_ros import TransformBroadcaster
from dwb_msgs.msg import LocalPlanEvaluation

assert os.environ.get('ROS_DOMAIN_ID') == '167'
captures = Path('/home/polarbear/aic_path_following_evidence_20261003')
out = Path(os.environ.get('AIC_PATH_CHECK_OUT',str(captures)));out.mkdir(parents=True,exist_ok=True)
native = Path('/home/polarbear/ws_aic_mecanum_20261002')
relative = Path('src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml')
captured = json.loads((captures/'before.json').read_text())['state']
configs = {'before': yaml.safe_load((captures/'before-files'/relative).read_text()),
           'after': yaml.safe_load(Path(os.environ.get('AIC_PATH_CHECK_CONFIG',str(native/relative))).read_text())}
if os.environ.get('AIC_PATH_AFTER_ONLY')=='1':configs.pop('before')
curve = [[i*.05,0.] for i in range(21)]
curve += [[1.+.8*math.sin(i/40*math.pi/2),.8*(1.-math.cos(i/40*math.pi/2))] for i in range(1,41)]
curve += [[1.8,.8+i*.05] for i in range(1,45)]

def distance(point, path):
    p = np.asarray(point); a = np.asarray(path[:-1]); b = np.asarray(path[1:]); d = b-a
    t = np.clip(np.sum((p-a)*d,axis=1)/np.maximum(np.sum(d*d,axis=1),1e-12),0,1)
    return float(np.min(np.linalg.norm(p-(a+t[:,None]*d),axis=1)))

report = {}
for mode in os.environ.get('AIC_PATH_TEST_MODES','replay,near_tail,curve').split(','):
  for version, params in configs.items():
    label = mode+'-'+version
    controller = copy.deepcopy(params['controller_server']['ros__parameters'])
    controller.update(use_sim_time=True, odom_topic='/odom')
    controller['progress_checker']['movement_time_allowance'] = 60.
    costmap = copy.deepcopy(params['local_costmap']['local_costmap']['ros__parameters'])
    size=costmap['width'] if mode=='straight' else 6
    costmap.update(use_sim_time=True, robot_base_frame='base_link', width=size, height=size,
                   track_unknown_space=False, plugins=['inflation_layer'],update_frequency=20.)
    if mode in ('replay','near_tail'):
        costmap['plugins']=['static_layer','inflation_layer']
        costmap['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/probe_map',
                                'map_subscribe_transient_local':True}
    config={'controller_server':{'ros__parameters':controller},
            'local_costmap':{'local_costmap':{'ros__parameters':costmap}}}
    (out/(label+'-params.yaml')).write_text(yaml.safe_dump(config))
    rclpy.init(); n=rclpy.create_node('path_following_comparison')
    n.set_parameters([Parameter('use_sim_time',value=True)])
    clock=n.create_publisher(ClockMessage,'/clock',10)
    odom=n.create_publisher(Odometry,'/odom',10); tf=TransformBroadcaster(n)
    scene=json.loads((captures/'near-goal-stall.json').read_text())['state'] if mode=='near_tail' else captured
    path=scene['transformed_global_plan'] if mode in ('replay','near_tail') else curve
    if mode=='straight':path=[[i*.05,0.] for i in range(401)]
    state={'pose':scene['pose'][:] if mode in ('replay','near_tail') else [0.,0.],
           'velocity':[0.,0.], 'cmd':[0.,0.], 'time':100.,'moving':False,
           'trace':[],'evaluation':{},'commands':[]}
    def cmd(m):
        state['cmd']=[m.linear.x,m.linear.y]
        if state['moving']:state['commands'].append(state['cmd'][:])
    sub=n.create_subscription(Twist,'/probe_cmd_vel',cmd,10)
    def evaluation(m):
        if m.best_index >= len(m.twists): return
        best=m.twists[m.best_index]
        state['evaluation']={'v':[best.traj.velocity.x,best.traj.velocity.y],
            'poses':[[p.x,p.y] for p in best.traj.poses],
            'scores':[{'name':s.name,'raw':s.raw_score,'scale':s.scale} for s in best.scores]}
    evalsub=n.create_subscription(LocalPlanEvaluation,'/evaluation',evaluation,10)
    def tick():
        state['time']+=.05
        if state['moving'] and mode!='replay':
            for axis in range(2):
                v=state['velocity'][axis]; target=state['cmd'][axis]
                limits=controller['FollowPath']
                limit=limits['acc_lim_x' if axis==0 else 'acc_lim_y'] if abs(target)>abs(v) and v*target>=0 else abs(limits['decel_lim_x' if axis==0 else 'decel_lim_y'])
                state['velocity'][axis]+=max(-limit*.05,min(limit*.05,target-v))
                state['pose'][axis]+=state['velocity'][axis]*.05
            state['trace'].append({'t':state['time'],'pose':state['pose'][:],
                                   'cmd':state['cmd'][:], 'deviation':distance(state['pose'],path)})
        m=ClockMessage();m.clock.sec=int(state['time']);m.clock.nanosec=int(state['time']%1*1e9);clock.publish(m)
        transform=TransformStamped();transform.header.stamp=m.clock;transform.header.frame_id='odom'
        transform.child_frame_id='base_link';transform.transform.translation.x=state['pose'][0]
        transform.transform.translation.y=state['pose'][1];transform.transform.rotation.w=1.;tf.sendTransform(transform)
        d=Odometry();d.header=transform.header;d.child_frame_id='base_link';d.pose.pose.orientation.w=1.
        d.pose.pose.position.x=state['pose'][0];d.pose.pose.position.y=state['pose'][1]
        d.twist.twist.linear.x=state['velocity'][0];d.twist.twist.linear.y=state['velocity'][1];odom.publish(d)
    timer=n.create_timer(.025,tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    if mode in ('replay','near_tail'):
        map_pub=n.create_publisher(OccupancyGrid,'/probe_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        m=OccupancyGrid();g=scene['local_grid'];m.header.frame_id='odom';m.info.resolution=g['resolution']
        m.info.width=g['width'];m.info.height=g['height'];m.info.origin.position.x=g['origin'][0]
        m.info.origin.position.y=g['origin'][1];m.info.origin.orientation.w=1.
        # OccupancyGrid 99 is inflated/inscribed cost 253, not a lethal wall.
        grid='local-near-goal-stall.npy' if mode=='near_tail' else 'local-before.npy'
        m.data=np.where(np.load(captures/grid)>=100,100,0).flatten().astype(int).tolist();map_pub.publish(m)
    log=(out/(label+'.log')).open('w')
    process=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args',
            '--params-file',str(out/(label+'-params.yaml')),'-r','cmd_vel:=/probe_cmd_vel'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert process.poll() is None,label+' controller exited'
            rclpy.spin_once(n,timeout_sec=.01)
    def wait(f,timeout=12):
        end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        assert f.done(),label+' timeout'
        return f.result()
    try:
        lifecycle=n.create_client(ChangeState,'/controller_server/change_state')
        end=time.monotonic()+15
        while not lifecycle.service_is_ready() and time.monotonic()<end:spin(.1)
        assert lifecycle.service_is_ready()
        for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
            req=ChangeState.Request();req.transition.id=transition;assert wait(lifecycle.call_async(req)).success
        action=ActionClient(n,FollowPath,'/follow_path');spin(.5);assert action.wait_for_server(timeout_sec=5)
        goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
        goal.path.header.frame_id='odom';goal.path.header.stamp=n.get_clock().now().to_msg()
        for x,y in path:
            p=PoseStamped();p.header=goal.path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.
            goal.path.poses.append(p)
        handle=wait(action.send_goal_async(goal));assert handle.accepted
        state['moving']=True;result=handle.get_result_async()
        end=time.monotonic()+(3 if mode=='replay' else 25)
        while not result.done() and time.monotonic()<end:spin(.05)
        state['moving']=False
        status=result.result().status if result.done() else None
        if not result.done():wait(handle.cancel_goal_async())
        report[label]={**state,'status':status,'max_deviation':max((p['deviation'] for p in state['trace']),default=0.),
                       'peak_speed':max((math.hypot(*p['cmd']) for p in state['trace']),default=0.)}
        print(label,json.dumps({k:report[label][k] for k in ('pose','cmd','status','max_deviation')}),flush=True)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();n.destroy_node();rclpy.shutdown()
(out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
if 'curve-after' in report:
    assert report['curve-after']['status']==4, 'New controller failed curved-path goal'
    assert report['curve-after']['max_deviation']<.15, 'Excessive curved-path deviation'
if 'curve-before' in report:
    assert report['curve-after']['max_deviation']<report['curve-before']['max_deviation'], 'Tracking did not improve'
if 'replay-after' in report:assert math.hypot(*report['replay-after']['cmd'])>.075, 'Captured corner still creeping'
if 'near_tail-after' in report:assert report['near_tail-after']['status']==4, 'Curved short goal tail stalls'
if 'straight-after' in report:assert report['straight-after']['status']==4, 'Long straight goal failed'
