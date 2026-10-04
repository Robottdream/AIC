"""Capture active path following without interrupting the business goal."""
import json
import os
import math
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Path as NavPath, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dwb_msgs.msg import LocalPlanEvaluation
import numpy as np

out=Path('/home/polarbear/aic_path_following_evidence_20261003');out.mkdir(parents=True,exist_ok=True)
label=os.environ.get('AIC_PATH_CAPTURE_LABEL','before')
rclpy.init();n=rclpy.create_node('path_following_capture');n.set_parameters([Parameter('use_sim_time',value=True)])
state={};samples=[];raw={};subs=[]
def odom(m):
    q=m.pose.pose.orientation
    state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y],yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
subs.append(n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data))
for topic in ('plan','received_global_plan','transformed_global_plan','local_plan'):
    subs.append(n.create_subscription(NavPath,'/'+topic,lambda m,k=topic:state.update({k:[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10))
for topic in ('cmd_vel_nav','cmd_vel_unfiltered','cmd_vel_guarded','cmd_vel'):
    subs.append(n.create_subscription(Twist,'/'+topic,lambda m,k=topic:state.update({k:[m.linear.x,m.linear.y,m.angular.z]}),10))
for topic in ('navigation/predictive_planner','navigation/dynamic_guard'):
    subs.append(n.create_subscription(String,'/'+topic,lambda m,k=topic:state.update({k:json.loads(m.data)}),10))
for side in ('left','right'):
    def scan(m,k=side):
        sign=1 if k=='left' else -1;points=[]
        for i,d in enumerate(m.ranges):
            if not math.isfinite(d) or not m.range_min<=d<=m.range_max:continue
            angle=m.angle_min+i*m.angle_increment+sign*math.pi/2
            points.append([d*math.cos(angle),sign*.225+d*math.sin(angle)])
        state['scan_'+k]={'nearest':min(math.hypot(*p) for p in points),'points':points}
    subs.append(n.create_subscription(LaserScan,'/scan_'+side,scan,qos_profile_sensor_data))
for scope in ('local','global'):
    subs.append(n.create_subscription(OccupancyGrid,'/'+scope+'_costmap/costmap',lambda m,k=scope:raw.update({k:m}),qos_profile_sensor_data))
def evaluation(m):
    state['evaluation']={'best':m.best_index,'twists':[{'v':[t.traj.velocity.x,t.traj.velocity.y,t.traj.velocity.theta],
            'total':t.total,'scores':[{'name':s.name,'raw':s.raw_score,'scale':s.scale} for s in t.scores]} for t in m.twists]}
subs.append(n.create_subscription(LocalPlanEvaluation,'/evaluation',evaluation,10))
start=time.monotonic();last=start
while time.monotonic()-start<5:
    rclpy.spin_once(n,timeout_sec=.02)
    if time.monotonic()-last>.2:samples.append({'wall':time.monotonic()-start,**state});last=time.monotonic()
for scope,msg in raw.items():
    np.save(out/(scope+'-'+label+'.npy'),np.array(msg.data).reshape(msg.info.height,msg.info.width))
    state[scope+'_grid']={'origin':[msg.info.origin.position.x,msg.info.origin.position.y],'resolution':msg.info.resolution,'width':msg.info.width,'height':msg.info.height}
params={}
for node,names in [('controller_server',['FollowPath.critics','FollowPath.PathFollow.scale','FollowPath.PathFollow.lookahead_distance','FollowPath.PathFollow.control_horizon','FollowPath.PathFollow.max_prefix_deviation','FollowPath.trajectory_generator_name','FollowPath.sim_time']),
                   ('velocity_smoother',['feedback','scale_velocities','smoothing_frequency','max_accel','max_decel']),
                   ('scan_velocity_guard',['max_linear_speed','predictive_enabled','collision_footprint_radius']),
                   ('collision_monitor',['NearStop.points','FootprintApproach.footprint_topic'])]:
    c=n.create_client(GetParameters,'/'+node+'/get_parameters');assert c.wait_for_service(timeout_sec=10)
    f=c.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    params[node]={name:v.double_value if v.type==3 else v.string_value if v.type==4 else v.bool_value if v.type==1 else list(v.double_array_value) if v.type==8 else list(v.string_array_value) if v.type==9 else v.integer_value for name,v in zip(names,f.result().values)}
(out/(label+'.json')).write_text(json.dumps({'state':state,'samples':samples,'parameters':params},indent=2)+'\n')
print(json.dumps({k:v for k,v in state.items() if k not in ('plan','received_global_plan','transformed_global_plan','local_plan','evaluation','scan_left','scan_right')},indent=2))
print('laser nearest',{k:state['scan_'+k]['nearest'] for k in ('left','right')})
print(json.dumps(params,indent=2))
n.destroy_node();rclpy.shutdown()
