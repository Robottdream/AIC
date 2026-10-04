"""Measure a canceled FollowPath's stop through the full protected velocity chain."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters

rclpy.init();n=rclpy.create_node('protected_fast_stop_test');n.set_parameters([Parameter('use_sim_time',value=True)])
state={'interrupted':False};subs=[];handle=None;rows=[];report={}
def odom(m):
    q=m.pose.pose.orientation
    state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y],speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y),
      yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
subs.append(n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data))
for topic in ('/command','/chat','/manual_nav_target'):subs.append(n.create_subscription(String,topic,lambda m:state.update(interrupted=True),qos_profile_sensor_data))
for topic in ('cmd_vel_nav','cmd_vel_unfiltered','cmd_vel'):
    subs.append(n.create_subscription(Twist,'/'+topic,lambda m,k=topic:state.update({k:[m.linear.x,m.linear.y]}),10))
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def wait(f):
    end=time.monotonic()+10
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
    assert f.done(),'Action/service timed out'
    return f.result()
try:
    spin(2)
    assert '所有优化任务执行完成' in Path('/home/polarbear/ws_aic_mecanum_20261002/log/run/09_main.log').read_text().splitlines()[-1]
    assert state['speed']<.03 and not state['interrupted']
    service=n.create_client(GetParameters,'/velocity_smoother/get_parameters');assert service.wait_for_service(timeout_sec=10)
    values=wait(service.call_async(GetParameters.Request(names=['max_velocity','max_accel','max_decel']))).values
    report['smoother']={k:list(v.double_array_value) for k,v in zip(('max_velocity','max_accel','max_decel'),values)}
    assert report['smoother']['max_velocity'][0]==4 and report['smoother']['max_accel'][0]==2 and report['smoother']['max_decel'][0]==-5
    client=ActionClient(n,FollowPath,'/follow_path');assert client.wait_for_server(timeout_sec=10)
    goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
    goal.path.header.frame_id='map';goal.path.header.stamp=n.get_clock().now().to_msg()
    origin=state['pose'][:];assert abs(origin[1]-1.1)<.3 and (origin[0]<-10 or origin[0]>1),'Use a cleared corridor end'
    target=4. if origin[0]<-10 else -11.5
    for i in range(181):
        p=PoseStamped();p.header=goal.path.header;p.pose.position.x=origin[0]+(target-origin[0])*i/180;p.pose.position.y=origin[1]+(1.1-origin[1])*i/180
        p.pose.orientation.z=math.sin(state['yaw']/2);p.pose.orientation.w=math.cos(state['yaw']/2);goal.path.poses.append(p)
    handle=wait(client.send_goal_async(goal));assert handle.accepted
    start=time.monotonic();deadline=start+20
    while time.monotonic()<deadline and state['speed']<3.0 and not state['interrupted']:
        spin(.02);rows.append({**state,'sim':n.get_clock().now().nanoseconds/1e9,'wall':time.monotonic()})
    assert not state['interrupted'],'External task arrived'
    assert state['speed']>=3.0,'Predictive/clearance limits did not permit a high-speed stop test'
    report.update(speed_at_cancel=state['speed'],pose_at_cancel=state['pose'][:],sim_at_cancel=n.get_clock().now().nanoseconds/1e9,wall_at_cancel=time.monotonic())
    wait(handle.cancel_goal_async())
    deadline=time.monotonic()+6
    while time.monotonic()<deadline:
        spin(.02);rows.append({**state,'sim':n.get_clock().now().nanoseconds/1e9,'wall':time.monotonic()})
        if state['speed']<.05:break
    assert state['speed']<.05,'Failed to stop'
    report.update(pose_at_stop=state['pose'],stop_sim_seconds=n.get_clock().now().nanoseconds/1e9-report['sim_at_cancel'],
       stop_wall_seconds=time.monotonic()-report['wall_at_cancel'],stop_distance=math.dist(state['pose'],report['pose_at_cancel']),passed=True)
    print(json.dumps(report,indent=2),flush=True)
finally:
    if handle is not None:
        try:wait(handle.cancel_goal_async())
        except Exception:pass
    report['samples']=rows
    path=Path('/home/polarbear/aic_speed_ladder_20261003/fast-stop-4.json')
    if path.exists() and not path.with_name('fast-stop-4-first-attempt.json').exists():path.with_name('fast-stop-4-first-attempt.json').write_text(path.read_text())
    path.write_text(json.dumps(report,indent=2))
    n.destroy_node();rclpy.try_shutdown()
