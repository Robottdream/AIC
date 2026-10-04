"""Actual NavFn plans around a jamb with old/new clearance on isolated domain 168."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rosgraph_msgs.msg import Clock as ClockMsg
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from scipy.ndimage import distance_transform_edt
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID')=='168'
repo=Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC')
out=Path('/home/polarbear/aic_doorway_evidence_20261003/isolated-plan');out.mkdir(parents=True,exist_ok=True)
configs={'old':out.parent/'before-files/src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml',
         'new':repo/'src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml'}
rclpy.init();n=rclpy.create_node('doorway_plan_probe');n.set_parameters([Parameter('use_sim_time',value=True)])
clock=n.create_publisher(ClockMsg,'/clock',10);tf=TransformBroadcaster(n)
map_pub=n.create_publisher(OccupancyGrid,'/doorway_test_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
resolution=.05;side=200;grid=np.zeros((side,side),dtype=np.int8)
grid[0,:]=100;grid[-1,:]=100;grid[:,0]=100;grid[:,-1]=100
# 1.30m doorway centered on x=0; 10cm horizontal wall separates two rooms.
for row in (99,100):
    grid[row,:87]=100;grid[row,113:]=100
distance=distance_transform_edt(grid!=100)*resolution
msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=resolution
msg.info.width=side;msg.info.height=side;msg.info.origin.position.x=-5.;msg.info.origin.position.y=-5.
msg.info.origin.orientation.w=1.;msg.data=grid.ravel().tolist()
wall_start=time.monotonic()
def tick():
    seconds=400+time.monotonic()-wall_start
    stamp=ClockMsg();stamp.clock.sec=int(seconds);stamp.clock.nanosec=int((seconds-int(seconds))*1e9);clock.publish(stamp)
    tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='base_link'
    tr.header.stamp=n.get_clock().now().to_msg();tr.transform.translation.x=-2.;tr.transform.translation.y=1.
    tr.transform.rotation.w=1.;tf.sendTransform(tr)
    msg.header.stamp=tr.header.stamp;map_pub.publish(msg)
timer=n.create_timer(.1,tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
def wait(f,seconds=15):
    end=time.monotonic()+seconds
    while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    assert f.done(),'Timeout';return f.result()
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
report={}
for mode,path in configs.items():
    data=yaml.safe_load(path.read_text())
    costmap=copy.deepcopy(data['global_costmap']['global_costmap']['ros__parameters'])
    costmap.update(plugins=['static_layer','inflation_layer'],robot_base_frame='base_link',update_frequency=10.,
                   publish_frequency=10.,global_frame='map')
    costmap['static_layer']['map_topic']='/doorway_test_map'
    config={'planner_server':data['planner_server'], 'global_costmap':{'global_costmap':{'ros__parameters':costmap}}}
    config_path=out/(mode+'-params.yaml');config_path.write_text(yaml.safe_dump(config))
    log=(out/(mode+'.log')).open('w')
    process=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file',str(config_path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        lc=n.create_client(ChangeState,'/planner_server/change_state')
        end=time.monotonic()+15
        while not lc.service_is_ready() and time.monotonic()<end:spin(.1)
        assert lc.service_is_ready()
        for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
            req=ChangeState.Request();req.transition.id=transition;assert wait(lc.call_async(req)).success
        spin(1)
        action=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert action.wait_for_server(timeout_sec=5)
        goal=ComputePathToPose.Goal();goal.use_start=True;goal.planner_id='GridBased'
        for pose,xy in ((goal.start,(-2.,1.)),(goal.goal,(0.,-2.))):
            pose.header.frame_id='map';pose.header.stamp=n.get_clock().now().to_msg()
            pose.pose.position.x=xy[0];pose.pose.position.y=xy[1];pose.pose.orientation.w=1.
        handle=wait(action.send_goal_async(goal));assert handle.accepted
        result=wait(handle.get_result_async());assert result.status==4,(mode,result.status)
        pts=np.array([[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses])
        cells=((pts+5.)/resolution).astype(int)
        clearance=distance[cells[:,1],cells[:,0]]
        report[mode]={'path':pts.tolist(),'minimum_wall_distance':float(clearance.min()),
                      'length':float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum())}
        action.destroy();n.destroy_client(lc)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        log.close()
    spin(.5)
assert report['new']['minimum_wall_distance']>.435,report
assert report['new']['minimum_wall_distance']>=report['old']['minimum_wall_distance'],report
report['passed']=True;report['domain']=168;report['robot_motion']=False
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:{a:b for a,b in v.items() if a!='path'} if isinstance(v,dict) else v for k,v in report.items()},indent=2))
n.destroy_node();rclpy.shutdown()
