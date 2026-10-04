"""Navigate an idle robot over one checked straight route and return; no grasp commands."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

p=argparse.ArgumentParser()
p.add_argument('--label', required=True)
p.add_argument('--reuse-origin', action='store_true')
a=p.parse_args()
out=Path('/home/polarbear/aic_round_speed_evidence_20261003')
out.mkdir(parents=True, exist_ok=True)
rclpy.init()
n=rclpy.create_node('round_speed_route_probe')
n.set_parameters([Parameter('use_sim_time', value=True)])
state={'interrupted':False, 'recoveries':0}
def odom(m):
    q=m.pose.pose.orientation
    state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y,
                       math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
                 speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
subs=[n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data),
      n.create_subscription(Twist,'/cmd_vel',lambda m:state.update(command=[m.linear.x,m.linear.y,m.angular.z]),10),
      n.create_subscription(String,'/navigation/predictive_planner',lambda m:state.update(planner=json.loads(m.data)),10)]
for topic in ('/command','/chat','/manual_nav_target'):
    subs.append(n.create_subscription(String,topic,lambda m:state.update(interrupted=True),10))
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose')
planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
def wait(f,limit=12):
    deadline=time.monotonic()+limit
    while not f.done() and time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.02)
    if not f.done(): raise TimeoutError('Request timeout')
    return f.result()
def pause(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
def params(node,names):
    client=n.create_client(GetParameters,'/'+node+'/get_parameters')
    assert client.wait_for_service(timeout_sec=10),node
    request=GetParameters.Request(names=names)
    result=wait(client.call_async(request))
    values={}
    for name,value in zip(names,result.values):
        values[name]=value.double_value if value.type==3 else value.double_array_value.tolist() if value.type==8 else value.bool_value if value.type==1 else value.integer_value if value.type==2 else value.string_array_value if value.type==9 else value.string_value
    return values
report={'label':a.label,'legs':[],'parameters':{}}
handle=None
try:
    assert nav.wait_for_server(timeout_sec=15) and planner.wait_for_server(timeout_sec=15)
    pause(2)
    assert 'pose' in state and state['speed']<.02 and not state['interrupted'], state
    for node,names in [('controller_server',['FollowPath.max_speed_xy','FollowPath.acc_lim_x','FollowPath.acc_lim_y','FollowPath.sim_time']),
                       ('velocity_smoother',['max_velocity','max_accel','max_decel']),
                       ('scan_velocity_guard',['max_linear_speed','predictive_enabled']),
                       ('local_costmap/local_costmap',['width','height'])]:
        report['parameters'][node]=params(node,names)
    if a.reuse_origin:
        route=json.loads((out/'route-origin.json').read_text())
        origin=route['origin']
        assert math.dist(state['pose'][:2],origin[:2])<.3,(state['pose'],origin)
    else:
        origin=list(state['pose'])
        route=None
        for distance in (4.,3.,2.5):
            candidates=[]
            for dx,dy in ((1.,0.),(0.,1.),(-1.,0.),(0.,-1.)):
                check=ComputePathToPose.Goal()
                check.goal.header.frame_id='map'
                check.goal.header.stamp=n.get_clock().now().to_msg()
                check.goal.pose.position.x=origin[0]+distance*dx
                check.goal.pose.position.y=origin[1]+distance*dy
                check.goal.pose.orientation.w=1.
                check.planner_id='GridBased'
                candidate=wait(planner.send_goal_async(check))
                if not candidate.accepted:continue
                result=wait(candidate.get_result_async())
                if result.status!=4:continue
                pts=result.result.path.poses
                length=sum(math.hypot(b.pose.position.x-c.pose.position.x,b.pose.position.y-c.pose.position.y) for c,b in zip(pts,pts[1:]))
                if len(pts)>1 and length<distance*1.10:
                    candidates.append((length,check.goal.pose.position.x,check.goal.pose.position.y))
            if candidates:
                length,x,y=min(candidates)
                route={'origin':origin,'target':[x,y],'distance':distance,'checked_length':length}
                break
        assert route is not None,'No clear straight test route from current position'
        (out/'route-origin.json').write_text(json.dumps(route))
        print('Checked route',json.dumps(route),flush=True)
    targets=[('outbound',*route['target']),('return',origin[0],origin[1])]
    for kind,x,y in targets:
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='map'
        goal.pose.header.stamp=n.get_clock().now().to_msg()
        goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
        goal.pose.pose.orientation.z=math.sin(origin[2]/2)
        goal.pose.pose.orientation.w=math.cos(origin[2]/2)
        check=ComputePathToPose.Goal();check.goal=goal.pose;check.planner_id='GridBased'
        path_handle=wait(planner.send_goal_async(check));assert path_handle.accepted
        path_result=wait(path_handle.get_result_async())
        assert path_result.status==4,'Route unavailable'
        path=path_result.result.path.poses
        length=sum(math.hypot(b.pose.position.x-c.pose.position.x,b.pose.position.y-c.pose.position.y) for c,b in zip(path,path[1:]))
        assert len(path)>1 and length<route['distance']*1.15,('Route is not a clear short segment',length)
        def feedback(message):state['recoveries']=message.feedback.number_of_recoveries
        start=time.monotonic();sim=n.get_clock().now().nanoseconds/1e9
        handle=wait(nav.send_goal_async(goal,feedback_callback=feedback));assert handle.accepted
        future=handle.get_result_async();trace=[]
        while not future.done() and time.monotonic()-start<90 and not state['interrupted']:
            rclpy.spin_once(n,timeout_sec=.02)
            trace.append({'wall':time.monotonic()-start,'sim':n.get_clock().now().nanoseconds/1e9-sim,
                          'pose':state.get('pose'),'speed':state.get('speed'),
                          'command':state.get('command'),'recoveries':state['recoveries']})
        if not future.done() or state['interrupted']:
            wait(handle.cancel_goal_async())
            report['interrupted_leg']={'kind':kind,'reason':'external_task' if state['interrupted'] else 'timeout',
                                      'samples':trace,'final_pose':state.get('pose')}
            raise RuntimeError('Route timeout or external task started')
        result=future.result()
        leg={'kind':kind,'status':result.status,'wall_seconds':time.monotonic()-start,
             'sim_seconds':n.get_clock().now().nanoseconds/1e9-sim,'planned_length':length,
             'actual_peak_speed':max(s['speed'] or 0 for s in trace),
             'command_peak_speed':max(math.hypot(*(s['command'] or [0,0])[:2]) for s in trace),
             'recoveries':state['recoveries'],'final_pose':state['pose'],'samples':trace}
        report['legs'].append(leg)
        print(json.dumps({k:v for k,v in leg.items() if k!='samples'}),flush=True)
        (out/(a.label+'.json')).write_text(json.dumps(report,indent=2))
        assert result.status==4,result.status
        pause(2)
    report['passed']=True
finally:
    (out/(a.label+'.json')).write_text(json.dumps(report,indent=2))
    if handle is not None and not report.get('passed'):
        try:wait(handle.cancel_goal_async(),5)
        except Exception:pass
    n.destroy_node();rclpy.shutdown()
