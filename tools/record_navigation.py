#!/usr/bin/env python3
"""Record planning, force telemetry and timestamped TF conversions to odom/world."""
import argparse,json,signal,time
from pathlib import Path
import rclpy
from rclpy.time import Time
from rclpy.parameter import Parameter
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('speed_ladder_plan_recorder')
node.set_parameters([Parameter('use_sim_time',value=True)])
buffer=Buffer();listener=TransformListener(buffer,node)
alive=True;records=[];pending=[];last={};events=[];forces=[];debug=[];prefetch=[]
def stop(*_):
    global alive
    alive=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
def stage(m):events.append(dict(received_wall=time.monotonic(),**json.loads(m.data)))
def path(m,topic):
    wall=time.monotonic()
    if wall-last.get(topic,0)<.2:return
    last[topic]=wall
    pending.append(dict(topic=topic,received_wall=wall,stamp_sec=m.header.stamp.sec,stamp_nanosec=m.header.stamp.nanosec,frame=m.header.frame_id,points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]))
def force(m):forces.append(dict(sim=m.header.stamp.sec+m.header.stamp.nanosec/1e9,frame=m.header.frame_id,fx=m.wrench.force.x,fy=m.wrench.force.y,tz=m.wrench.torque.z))
def convert(record):
    query=Time(seconds=record['stamp_sec'],nanoseconds=record['stamp_nanosec'])
    transform=buffer.lookup_transform('odom',record['frame'],query)
    tr,q=transform.transform.translation,transform.transform.rotation
    points=[]
    for x,y,z in record['points']:
        a,b,c=2*(q.y*z-q.z*y),2*(q.z*x-q.x*z),2*(q.x*y-q.y*x)
        points.append([x+q.w*a+q.y*c-q.z*b+tr.x,y+q.w*b+q.z*a-q.x*c+tr.y,z+q.w*c+q.x*b-q.y*a+tr.z])
    record.update(world_points=points,target_frame='odom',transform_stamp=transform.header.stamp.sec+transform.header.stamp.nanosec/1e9,transform_translation=[tr.x,tr.y,tr.z],transform_quaternion=[q.x,q.y,q.z,q.w])
node.create_subscription(String,'/task_stage',stage,100)
node.create_subscription(String,'/known_predictive_control',lambda m:debug.append(json.loads(m.data)),100)
node.create_subscription(String,'/return_prefetch_status',lambda m:prefetch.append(
    dict(received_wall=time.monotonic(),**json.loads(m.data))),100)
node.create_subscription(WrenchStamped,'/body_drive/wrench',force,100)
for topic in ('/plan','/received_global_plan'):
    node.create_subscription(RosPath,topic,lambda m,topic=topic:path(m,topic),qos_profile_sensor_data)
try:
    while alive:
        rclpy.spin_once(node,timeout_sec=.02)
        for record in pending[:]:
            try:
                convert(record);records.append(record);pending.remove(record)
            except Exception as exc:
                if time.monotonic()-record['received_wall']>3:
                    record['transform_error']=str(exc);records.append(record);pending.remove(record)
finally:
    for record in pending:record['transform_error']='Recorder stopped before transform available'
    records.extend(pending)
    (args.output/'plans.json').write_text(json.dumps(records))
    (args.output/'stages.json').write_text(json.dumps(events,indent=2))
    (args.output/'wrenches.json').write_text(json.dumps(forces))
    (args.output/'predictive-control.json').write_text(json.dumps(debug))
    (args.output/'return-prefetch.json').write_text(json.dumps(prefetch,indent=2))
    (args.output/'coverage.json').write_text(json.dumps(dict(plans=len(records),converted=sum('world_points' in r for r in records),unconverted=sum('world_points' not in r for r in records),force_samples=len(forces),odom_is_physical_world=True),indent=2))
    node.destroy_node();rclpy.shutdown()
