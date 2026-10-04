"""One navigation-only door crossing in the active simulator, no arm commands."""
import json
import math
import time
import argparse
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PolygonStamped
from std_msgs.msg import String

out=Path('/home/polarbear/aic_doorway_evidence_20261003')
p=argparse.ArgumentParser()
p.add_argument('--target',nargs=2,type=float,default=[2.593086,-5.727858])
p.add_argument('--label',default='navigation')
a=p.parse_args()
rclpy.init();n=rclpy.create_node('doorway_navigation_probe');n.set_parameters([Parameter('use_sim_time',value=True)])
state={'interrupted':False,'recoveries':0};samples=[];handle=None
def odom(m):
    q=m.pose.pose.orientation
    state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y],yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
def scan(m,side):
    sign=1 if side=='left' else -1
    points=[]
    for i,d in enumerate(m.ranges):
        if not math.isfinite(d) or not m.range_min<=d<=m.range_max:continue
        angle=m.angle_min+i*m.angle_increment+sign*math.pi/2
        points.append(math.hypot(d*math.cos(angle),sign*.225+d*math.sin(angle)))
    state['clearance_'+side]=min(points) if points else None
subs=[n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)]
for side in ('left','right'):
    subs.append(n.create_subscription(LaserScan,'/scan_'+side,lambda m,k=side:scan(m,k),qos_profile_sensor_data))
for topic in ('cmd_vel_nav','cmd_vel_unfiltered','cmd_vel_guarded','cmd_vel'):
    subs.append(n.create_subscription(Twist,'/'+topic,lambda m,k=topic:state.update({k:[m.linear.x,m.linear.y,m.angular.z]}),10))
for topic in ('/command','/chat','/manual_nav_target'):
    subs.append(n.create_subscription(String,topic,lambda m:state.update(interrupted=True),10))
subs.append(n.create_subscription(PolygonStamped,'/navigation/collision_footprint',lambda m:state.update(physical_footprint_radius=max(math.hypot(p.x,p.y) for p in m.polygon.points)),10))
subs.append(n.create_subscription(String,'/navigation/predicted_obstacles',lambda m:state.update(prediction_tracks=len(json.loads(m.data)['tracks'])),10))
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose');plan=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
def wait(f,limit=15):
    end=time.monotonic()+limit
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert f.done(),'Request timeout';return f.result()
report={'samples':samples,'target':a.target,'parameters':{}}
try:
    assert nav.wait_for_server(timeout_sec=15) and plan.wait_for_server(timeout_sec=15)
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert state.get('physical_footprint_radius',0)>.324
    assert min(state['clearance_left'],state['clearance_right'])>.5
    assert state['speed']<.02 and not state['interrupted']
    for node,names in [('collision_monitor',['NearStop.points','FootprintApproach.footprint_topic']),
                       ('controller_server',['FollowPath.max_speed_xy','FollowPath.ObstacleFootprint.scale']),
                       ('scan_velocity_guard',['max_linear_speed','collision_footprint_radius']),
                       ('local_costmap/local_costmap',['footprint','footprint_padding'])]:
        client=n.create_client(GetParameters,'/'+node+'/get_parameters');assert client.wait_for_service(timeout_sec=10)
        result=wait(client.call_async(GetParameters.Request(names=names)))
        values={name:v.double_value if v.type==3 else list(v.double_array_value) if v.type==8 else v.string_value for name,v in zip(names,result.values)}
        report['parameters'][node]=values
    points=report['parameters']['collision_monitor']['NearStop.points']
    assert len(points)==48 and abs(max(math.hypot(*points[i:i+2]) for i in range(0,48,2))-.405)<1e-5,'Full stop circle not restored'
    assert report['parameters']['scan_velocity_guard']['max_linear_speed']==1.0
    assert report['parameters']['collision_monitor']['FootprintApproach.footprint_topic']=='/navigation/collision_footprint'
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
    goal.pose.pose.position.x=report['target'][0];goal.pose.pose.position.y=report['target'][1]
    goal.pose.pose.orientation.z=math.sin(state['yaw']/2);goal.pose.pose.orientation.w=math.cos(state['yaw']/2)
    check=ComputePathToPose.Goal();check.goal=goal.pose;check.planner_id='GridBased'
    path_handle=wait(plan.send_goal_async(check));assert path_handle.accepted
    result=wait(path_handle.get_result_async());assert result.status==4
    report['planned_path']=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]
    report['initial']=dict(state)
    def feedback(m):state['recoveries']=m.feedback.number_of_recoveries
    handle=wait(nav.send_goal_async(goal,feedback_callback=feedback));assert handle.accepted
    future=handle.get_result_async();start=time.monotonic();sim=n.get_clock().now().nanoseconds/1e9
    last_print=start
    while not future.done() and time.monotonic()-start<120 and not state['interrupted']:
        rclpy.spin_once(n,timeout_sec=.02)
        samples.append({'wall':time.monotonic()-start,'sim':n.get_clock().now().nanoseconds/1e9-sim,**state})
        if time.monotonic()-last_print>10:
            print(json.dumps({'wall':time.monotonic()-start,**state}),flush=True);last_print=time.monotonic()
    if not future.done() or state['interrupted']:
        wait(handle.cancel_goal_async());raise RuntimeError('Door crossing timed out or external task arrived')
    report.update(status=future.result().status,wall_seconds=time.monotonic()-start,
                  sim_seconds=n.get_clock().now().nanoseconds/1e9-sim,final=state,
                  peak_speed=max(s['speed'] for s in samples),recoveries=state['recoveries'],
                  minimum_lidar_clearance=min(s[k] for s in samples for k in ('clearance_left','clearance_right') if s.get(k) is not None))
    assert report['status']==4,report['status']
    assert report['minimum_lidar_clearance']>.405,report['minimum_lidar_clearance']
    report['passed']=True
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','planned_path','parameters')},indent=2),flush=True)
finally:
    if handle is not None and not report.get('passed'):
        try:wait(handle.cancel_goal_async(),5)
        except Exception:pass
    (out/(a.label+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    n.destroy_node();rclpy.shutdown()
