"""Observe the existing business task; never send goals, commands, or arm actions."""
import json
import math
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

out=Path('/home/polarbear/aic_path_following_evidence_20261003')/(os.environ.get('AIC_PATH_OBSERVER_LABEL','live')+'.json')
rclpy.init();n=rclpy.create_node('path_following_task_observer')
n.set_parameters([Parameter('use_sim_time',value=True)])
state={};trace=[];events=[];subs=[];start=time.monotonic();last=start
def odom(m):
    state['pose']=[m.pose.pose.position.x,m.pose.pose.position.y]
    state['speed']=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
subs.append(n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data))
for topic in ('transformed_global_plan','local_plan'):
    subs.append(n.create_subscription(NavPath,'/'+topic,lambda m,k=topic:state.update({k:[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10))
for side in ('left','right'):
    def scan(m,k=side):
        sign=1 if k=='left' else -1
        points=[(d*math.cos(m.angle_min+i*m.angle_increment+sign*math.pi/2),
                 sign*.225+d*math.sin(m.angle_min+i*m.angle_increment+sign*math.pi/2))
                for i,d in enumerate(m.ranges) if math.isfinite(d) and m.range_min<=d<=m.range_max]
        state['nearest_'+k]=min((math.hypot(*p) for p in points),default=None)
    subs.append(n.create_subscription(LaserScan,'/scan_'+side,scan,qos_profile_sensor_data))
for topic in ('cmd_vel_nav','cmd_vel'):
    subs.append(n.create_subscription(Twist,'/'+topic,lambda m,k=topic:state.update({k:[m.linear.x,m.linear.y,m.angular.z]}),10))
for topic in ('nav_status','arm_status','navigation/recovering'):
    def event(m,k=topic):
        events.append({'wall':time.monotonic()-start,'sim':n.get_clock().now().nanoseconds/1e9,'topic':k,'data':m.data})
        print(json.dumps(events[-1],ensure_ascii=False),flush=True)
    # Subscribe the two public status strings (the recovery topic is Bool).
    if topic!='navigation/recovering':subs.append(n.create_subscription(String,'/'+topic,event,10))
for topic in ('cur','number_pick'):
    subs.append(n.create_subscription(Int32,'/'+topic,lambda m,k=topic:state.update({k:m.data}),10))
def deviation(point,path):
    if len(path)<2:return None
    a=np.asarray(path[:-1]);d=np.asarray(path[1:])-a;p=np.asarray(point)
    t=np.clip(np.sum((p-a)*d,axis=1)/np.maximum(np.sum(d*d,axis=1),1e-12),0,1)
    return float(np.min(np.linalg.norm(p-(a+t[:,None]*d),axis=1)))
try:
    while time.monotonic()-start<float(os.environ.get('AIC_PATH_OBSERVER_SECONDS','300')):
        rclpy.spin_once(n,timeout_sec=.02)
        if time.monotonic()-last<.2:continue
        last=time.monotonic()
        p=state.get('transformed_global_plan',[])
        sample={k:v for k,v in state.items() if k not in ('transformed_global_plan','local_plan')}
        sample.update(wall=last-start,sim=n.get_clock().now().nanoseconds/1e9)
        if 'pose' in state:sample['path_deviation']=deviation(state['pose'],p)
        sample['local_prefix_deviation']=max((deviation(v,p) or 0 for v in state.get('local_plan',[])[:13]),default=0.)
        trace.append(sample)
        if len(trace)%50==0:
            out.write_text(json.dumps({'trace':trace,'events':events},indent=2)+'\n')
            print(json.dumps(sample,ensure_ascii=False),flush=True)
except KeyboardInterrupt:
    pass
finally:
    out.write_text(json.dumps({'trace':trace,'events':events},indent=2)+'\n')
    n.destroy_node();rclpy.try_shutdown()
