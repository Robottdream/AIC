"""Matched navigation circuit on the idle running robot. Yield to any external task."""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

p=argparse.ArgumentParser();p.add_argument('--speed',type=float,required=True);p.add_argument('--straight-only',action='store_true');p.add_argument('--follow-straight',action='store_true');p.add_argument('--follow-long',action='store_true');p.add_argument('--tag',default='');a=p.parse_args()
out=Path('/home/polarbear/aic_speed_ladder_20261003');out.mkdir(exist_ok=True)
rclpy.init();n=rclpy.create_node('speed_ladder_navigation_probe');n.set_parameters([Parameter('use_sim_time',value=True)])
state={'interrupted':False,'recoveries':0};subs=[];handle=None
def odom(m):
    q=m.pose.pose.orientation
    state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y],
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
        speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
subs.append(n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data))
for side in ('left','right'):
    def scan(m,k=side):
        sign=1 if k=='left' else -1
        state['clearance_'+k]=min((math.hypot(d*math.cos(m.angle_min+i*m.angle_increment+sign*math.pi/2),
            sign*.225+d*math.sin(m.angle_min+i*m.angle_increment+sign*math.pi/2))
            for i,d in enumerate(m.ranges) if math.isfinite(d) and m.range_min<=d<=m.range_max),default=99.)
    subs.append(n.create_subscription(LaserScan,'/scan_'+side,scan,qos_profile_sensor_data))
for topic in ('cmd_vel_nav','cmd_vel'):
    subs.append(n.create_subscription(Twist,'/'+topic,lambda m,k=topic:state.update({k:[m.linear.x,m.linear.y,m.angular.z]}),10))
for topic in ('/command','/chat','/manual_nav_target'):
    subs.append(n.create_subscription(String,topic,lambda m:state.update(interrupted=True),qos_profile_sensor_data))
subs.append(n.create_subscription(Int32,'/cur',lambda m:state.update(stage=m.data),qos_profile_sensor_data))
subs.append(n.create_subscription(NavPath,'/transformed_global_plan',lambda m:state.update(path=[[v.pose.position.x,v.pose.position.y] for v in m.poses]),10))
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose');planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
follower=ActionClient(n,FollowPath,'/follow_path')
def wait(f,seconds=15):
    end=time.monotonic()+seconds
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert f.done(),'ROS request timed out'
    return f.result()
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
def deviation(xy,path):
    if len(path)<2:return None
    q=np.asarray(xy);b=np.asarray(path[:-1]);d=np.asarray(path[1:])-b
    t=np.clip(np.sum((q-b)*d,axis=1)/np.maximum(np.sum(d*d,axis=1),1e-12),0,1)
    return float(np.min(np.linalg.norm(q-b-t[:,None]*d,axis=1)))
report={'profile_speed':a.speed,'legs':[],'parameters':{}}
filename=('calibration-long-' if a.follow_long else 'calibration-' if a.follow_straight else 'straight-' if a.straight_only else 'route-')+str(int(a.speed))+'.json'
if a.tag:
    assert a.tag.replace('-','').isalnum(),'Invalid evidence tag'
    filename=filename[:-5]+'-'+a.tag+'.json'
try:
    assert nav.wait_for_server(timeout_sec=15) and planner.wait_for_server(timeout_sec=15)
    spin(2)
    main=Path('/home/polarbear/ws_aic_mecanum_20261002/log/run/09_main.log').read_text().splitlines()[-1]
    assert '所有优化任务执行完成' in main and state.get('stage',0)==0 and state['speed']<.03 and not state['interrupted'],'Business task active; do not preempt it'
    for node,names in [('controller_server',['FollowPath.max_speed_xy','FollowPath.PathFollow.lookahead_distance','FollowPath.PathFollow.max_path_chord_deviation','FollowPath.sim_time','FollowPath.forward_prune_distance']),
        ('velocity_smoother',['max_velocity','max_accel','max_decel']),('scan_velocity_guard',['max_linear_speed','predictive_enabled','prediction_horizon']),
        ('collision_monitor',['NearStop.points','FootprintApproach.time_before_collision']),('local_costmap/local_costmap',['width','height'])]:
        c=n.create_client(GetParameters,'/'+node+'/get_parameters');assert c.wait_for_service(timeout_sec=10)
        values=wait(c.call_async(GetParameters.Request(names=names))).values
        report['parameters'][node]={key:v.double_value if v.type==3 else list(v.double_array_value) if v.type==8 else v.bool_value if v.type==1 else v.integer_value for key,v in zip(names,values)}
    assert report['parameters']['scan_velocity_guard']['max_linear_speed']==a.speed
    pts=report['parameters']['collision_monitor']['NearStop.points']
    assert abs(max(math.hypot(*pts[i:i+2]) for i in range(0,len(pts),2))-.405)<1e-5
    # A 15.5 m horizontal corridor, then A doorway in/out. All profiles return to
    # the same anchor; preposition is logged separately and excluded from timing.
    route=[('preposition',-11.5,1.1),('long_corridor',4.,1.1),
           ('enter_A',2.593086,-5.727858),('exit_A_return',-11.5,1.1)]
    if a.straight_only or a.follow_straight:route=[('preposition',0.,-.4),('straight_out',8.8,-.4),('straight_back',0.,-.4)]
    if a.follow_long:route=[('preposition',-11.5,1.1),('straight_out',4.,1.1),('straight_back',-11.5,1.1)]
    for label,x,y in route:
        if label=='preposition' and math.dist(state['pose'],[x,y])<.2:continue
        assert not state['interrupted'],'External task arrived'
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
        goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
        goal.pose.pose.orientation.z=math.sin(state['yaw']/2);goal.pose.pose.orientation.w=math.cos(state['yaw']/2)
        check=ComputePathToPose.Goal();check.goal=goal.pose;check.planner_id='GridBased'
        h=wait(planner.send_goal_async(check));assert h.accepted
        r=wait(h.get_result_async());assert r.status==4,('No route',label)
        planned=[[v.pose.position.x,v.pose.position.y] for v in r.result.path.poses]
        length=sum(math.dist(b,c) for b,c in zip(planned,planned[1:]))
        state['recoveries']=0
        def feedback(m):state['recoveries']=m.feedback.number_of_recoveries
        if (a.follow_straight or a.follow_long) and label!='preposition':
            assert follower.wait_for_server(timeout_sec=5)
            direct=FollowPath.Goal();direct.controller_id='FollowPath';direct.goal_checker_id='general_goal_checker'
            direct.path.header=goal.pose.header
            origin=state['pose'][:]
            for i in range(181):
                point=PoseStamped();point.header=direct.path.header
                point.pose.position.x=origin[0]+(x-origin[0])*i/180
                point.pose.position.y=origin[1]+(y-origin[1])*i/180
                point.pose.orientation=goal.pose.pose.orientation;direct.path.poses.append(point)
            planned=[[p.pose.position.x,p.pose.position.y] for p in direct.path.poses]
            length=math.dist(origin,[x,y])
            handle=wait(follower.send_goal_async(direct))
        else:handle=wait(nav.send_goal_async(goal,feedback_callback=feedback))
        assert handle.accepted
        f=handle.get_result_async();start=time.monotonic();simstart=n.get_clock().now().nanoseconds/1e9
        samples=[];last_sample=start;last_print=start
        while not f.done() and time.monotonic()-start<100 and not state['interrupted']:
            rclpy.spin_once(n,timeout_sec=.02);now=time.monotonic()
            if now-last_sample>=.1:
                sample={k:v for k,v in state.items() if k!='path'}
                sample.update(wall=now-start,sim=n.get_clock().now().nanoseconds/1e9-simstart,
                    deviation=deviation(state['pose'],state.get('path',[])))
                samples.append(sample);last_sample=now
                if min(state.get('clearance_left',99),state.get('clearance_right',99))<.405:
                    report['safety_abort']='Entered original NearStop circle';break
            if now-last_print>10:
                print(json.dumps({'profile':a.speed,'leg':label,'elapsed':now-start,'pose':state['pose'],'speed':state['speed'],'recoveries':state['recoveries']}),flush=True);last_print=now
        if not f.done() or state['interrupted'] or report.get('safety_abort'):
            wait(handle.cancel_goal_async());status=None
        else:status=f.result().status
        row={'leg':label,'status':status,'wall_seconds':time.monotonic()-start,
             'sim_seconds':n.get_clock().now().nanoseconds/1e9-simstart,'planned_length':length,
             'peak_speed':max((v['speed'] for v in samples),default=0.),'recoveries':state['recoveries'],
             'min_clearance':min((v[k] for v in samples for k in ('clearance_left','clearance_right') if k in v),default=None),
             'max_deviation':max((v['deviation'] or 0 for v in samples),default=None),'samples':samples}
        report['legs'].append(row);print(json.dumps({k:v for k,v in row.items() if k!='samples'}),flush=True)
        (out/filename).write_text(json.dumps(report,indent=2)+'\n')
        assert status==4,('Route failed or interrupted',label,status)
        spin(1)
    report['passed']=True
finally:
    if handle is not None and not report.get('passed'):
        try:wait(handle.cancel_goal_async(),5)
        except Exception:pass
    (out/filename).write_text(json.dumps(report,indent=2)+'\n')
    n.destroy_node();rclpy.try_shutdown()
