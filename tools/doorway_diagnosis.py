"""Read live scans/costmaps and ask for a path; never send motion commands."""
import json
import math
import time
import os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

rclpy.init()
n=rclpy.create_node('doorway_diagnosis')
n.set_parameters([Parameter('use_sim_time',value=True)])
buffer=Buffer(node=n);listener=TransformListener(buffer,n)
state={};raw={}
subs=[]
for side in ('left','right'):
    subs.append(n.create_subscription(LaserScan,'/scan_'+side,lambda m,k=side:raw.update({k:m}),qos_profile_sensor_data))
for scope in ('local','global'):
    subs.append(n.create_subscription(OccupancyGrid,'/'+scope+'_costmap/costmap',lambda m,k=scope:raw.update({k:m}),qos_profile_sensor_data))
subs.append(n.create_subscription(Odometry,'/odom',lambda m:state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y],yaw=math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2))),qos_profile_sensor_data))
for topic in ('cmd_vel_nav','cmd_vel_unfiltered','cmd_vel_guarded','cmd_vel'):
    subs.append(n.create_subscription(Twist,'/'+topic,lambda m,k=topic:state.update({k:[m.linear.x,m.linear.y,m.angular.z]}),10))
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
state['scans']={}
for side in ('left','right'):
    scan=raw[side]
    tf=buffer.lookup_transform('base_footprint',scan.header.frame_id,rclpy.time.Time()).transform
    q=tf.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    points=[]
    for i,d in enumerate(scan.ranges):
        if not math.isfinite(d) or d<scan.range_min or d>scan.range_max:continue
        angle=yaw+scan.angle_min+i*scan.angle_increment
        points.append([tf.translation.x+d*math.cos(angle),tf.translation.y+d*math.sin(angle)])
    distances=[math.hypot(x,y) for x,y in points]
    state['scans'][side]={'nearest':min(distances),'inside_stop':sum(d<.405 for d in distances),'points':points}
out=Path('/home/polarbear/aic_doorway_evidence_20261003');out.mkdir(parents=True,exist_ok=True)
label=os.environ.get('AIC_DOORWAY_SNAPSHOT','before')
for scope in ('local','global'):
    msg=raw[scope]
    array=np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
    np.save(out/((scope+'-costmap.npy') if label=='before' else (label+'-'+scope+'-costmap.npy')),array)
    state[scope]={'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
                  'origin':[msg.info.origin.position.x,msg.info.origin.position.y],'frame':msg.header.frame_id}
client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=15)
def wait(f):
    end=time.monotonic()+15
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert f.done();return f.result()
state['paths']={}
for name,start,goal in [('return_A',[-8.152837,1.000008],[2.593086,-5.727858]),
                         ('current_A',state['pose'],[2.593086,-5.727858]),
                         ('retreat',state['pose'],[state['pose'][0],state['pose'][1]+1.0])]:
    g=ComputePathToPose.Goal();g.planner_id='GridBased';g.use_start=True
    for target,xy in ((g.start,start),(g.goal,goal)):
        target.header.frame_id='map';target.header.stamp=n.get_clock().now().to_msg()
        target.pose.position.x=float(xy[0]);target.pose.position.y=float(xy[1]);target.pose.orientation.w=1.
    handle=wait(client.send_goal_async(g));assert handle.accepted
    result=wait(handle.get_result_async())
    state['paths'][name]={'status':result.status,'points':[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]}
(out/(label+'.json')).write_text(json.dumps(state,indent=2)+'\n')
print(json.dumps({k:v for k,v in state.items() if k not in ('paths','scans')},indent=2))
print(json.dumps({s:{k:v for k,v in d.items() if k!='points'} for s,d in state['scans'].items()},indent=2))
print({s:{'status':d['status'],'poses':len(d['points'])} for s,d in state['paths'].items()})
n.destroy_node();rclpy.shutdown()
