"""One low-speed navigation step away from the measured wall; full command chain."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

out=Path('/home/polarbear/aic_doorway_evidence_20261003')
diagnosis=json.loads((out/'before.json').read_text())
points=[p for scan in diagnosis['scans'].values() for p in scan['points']]
# The near wall is behind/left of the robot. Verify every sampled point along
# the proposed translation remains outside the temporary .360m stop polygon.
offset=[.7,.25]
minimum=min(math.hypot(p[0]-offset[0]*i/40,p[1]-offset[1]*i/40) for p in points for i in range(41))
assert minimum>.375,('Insufficient measured clearance for slow escape',minimum)
rclpy.init();n=rclpy.create_node('doorway_safe_escape')
n.set_parameters([Parameter('use_sim_time',value=True)])
state={'interrupted':False};samples=[];handle=None
def odom(m):
    q=m.pose.pose.orientation
    state.update(pose=[m.pose.pose.position.x,m.pose.pose.position.y],yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
subs=[n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data),
      n.create_subscription(Twist,'/cmd_vel',lambda m:state.update(command=[m.linear.x,m.linear.y,m.angular.z]),10)]
for topic in ('/command','/chat','/manual_nav_target'):
    subs.append(n.create_subscription(String,topic,lambda m:state.update(interrupted=True),10))
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose');plan=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
def wait(f,limit=15):
    end=time.monotonic()+limit
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert f.done(),'Action timeout';return f.result()
report={'sweep_min_before':minimum,'temporary_stop_radius':.36,'cap':.12,'samples':samples}
try:
    assert nav.wait_for_server(timeout_sec=15) and plan.wait_for_server(timeout_sec=15)
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert not state['interrupted'] and state['speed']<.02
    assert math.dist(state['pose'],diagnosis['pose'])<.05,'Robot changed since clearance snapshot'
    c,s=math.cos(state['yaw']),math.sin(state['yaw'])
    target=[state['pose'][0]+c*offset[0]-s*offset[1],state['pose'][1]+s*offset[0]+c*offset[1]]
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
    goal.pose.pose.position.x=target[0];goal.pose.pose.position.y=target[1]
    goal.pose.pose.orientation.z=math.sin(state['yaw']/2);goal.pose.pose.orientation.w=math.cos(state['yaw']/2)
    check=ComputePathToPose.Goal();check.goal=goal.pose;check.planner_id='GridBased'
    path_handle=wait(plan.send_goal_async(check));assert path_handle.accepted
    result=wait(path_handle.get_result_async());assert result.status==4
    pts=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]
    assert sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))<1.2,'Escape path detours toward other obstacles'
    handle=wait(nav.send_goal_async(goal));assert handle.accepted
    f=handle.get_result_async();start=time.monotonic()
    while not f.done() and time.monotonic()-start<45 and not state['interrupted']:
        rclpy.spin_once(n,timeout_sec=.02)
        samples.append({'wall':time.monotonic()-start,**state})
        assert state.get('speed',0)<.14,'Escape exceeded slow cap'
    if not f.done() or state['interrupted']:
        wait(handle.cancel_goal_async());raise RuntimeError('Escape did not finish or a new task arrived')
    report.update(status=f.result().status,wall_seconds=time.monotonic()-start,final=state,target=target,
                  peak_speed=max(p['speed'] for p in samples))
    assert report['status']==4
    report['passed']=True
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)
finally:
    if handle is not None and not report.get('passed'):
        try:wait(handle.cancel_goal_async(),5)
        except Exception:pass
    (out/'escape.json').write_text(json.dumps(report,indent=2)+'\n')
    n.destroy_node();rclpy.shutdown()
