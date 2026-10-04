"""Collect live dual lidar, camera, joint state and yaw actuator evidence."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    rclpy.init(); n=rclpy.create_node('mecanum_live_check'); n.set_parameters([Parameter('use_sim_time',value=True)])
    state={}; counts={}; first={}; last={}; subs=[]; yaw_trace=[]
    def scan(m,t):
        counts[t]=counts.get(t,0)+1
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        first.setdefault(t,stamp); last[t]=stamp
        state[t]={'frame':m.header.frame_id,'samples':len(m.ranges),'finite':sum(math.isfinite(v) for v in m.ranges),'nan':sum(math.isnan(v) for v in m.ranges)}
    for t in ['/scan_left_raw','/scan_right_raw','/scan_left','/scan_right','/scan']:
        subs.append(n.create_subscription(LaserScan,t,lambda m,t=t:scan(m,t),qos_profile_sensor_data))
    subs.append(n.create_subscription(Image,'/camera/image_raw',lambda m:state.update(camera={'width':m.width,'height':m.height}),qos_profile_sensor_data))
    def joints(m):
        state['joints']=dict(zip(m.name,m.position)); yaw_trace.append(state['joints'].get('arm_yaw_joint'))
    subs.append(n.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data))
    subs.append(n.create_subscription(Odometry,'/odom',lambda m:state.update(odom_z=m.pose.pose.position.z),qos_profile_sensor_data))
    deadline=time.monotonic()+8
    while time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.03)
    for t in ['/scan_left','/scan_right','/scan']:
        assert counts.get(t,0)>10, f'Missing scanner {t}'
        state[t]['sim_hz']=(counts[t]-1)/(last[t]-first[t])
    assert 'camera' in state
    for j in ['arm_yaw_joint']+[p+'_wheel_joint' for p in ['front_left','front_right','rear_left','rear_right']]: assert j in state['joints'], f'Missing joint {j}'
    client=ActionClient(n,FollowJointTrajectory,'/arm_yaw_controller/follow_joint_trajectory')
    assert client.wait_for_server(timeout_sec=10)
    def wait(f,limit):
        deadline=time.monotonic()+limit
        while not f.done() and time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.03)
        assert f.done(),'Yaw action timeout'
        return f.result()
    state['yaw_tests']=[]
    for angle in [.35,0.]:
        g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=['arm_yaw_joint']
        point=JointTrajectoryPoint(); point.positions=[angle]; point.time_from_start.sec=1
        g.trajectory.points=[point]
        h=wait(client.send_goal_async(g),5); assert h.accepted
        r=wait(h.get_result_async(),10); assert r.status==4 and r.result.error_code==0
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.03)
        actual=state['joints']['arm_yaw_joint']; assert abs(actual-angle)<.04
        state['yaw_tests'].append({'target':angle,'actual':actual,'status':r.status})
    for t in ['/scan_left','/scan_right','/scan']:
        state[t]['sim_hz']=(counts[t]-1)/(last[t]-first[t])
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(state,indent=2))
    print(json.dumps(state,indent=2)); n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
