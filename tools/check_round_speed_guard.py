"""Exercise the real final guard on an isolated ROS domain, including stale data."""
import json
import math
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PolygonStamped, Twist
from sensor_msgs.msg import LaserScan

assert os.environ.get('ROS_DOMAIN_ID')=='166'
sys.path.insert(0,'/home/polarbear/ws_aic_mecanum_20261002/src/yzbot/bot_navigation/scripts')
from scan_velocity_guard import ScanVelocityGuard

limit=float(os.environ.get('AIC_GUARD_CHECK_SPEED','1.0'))
rclpy.init(args=['--ros-args','-p',f'max_linear_speed:={limit}','-p','collision_footprint_radius:=0.325'])
guard=ScanVelocityGuard()
probe=rclpy.create_node('speed_guard_inputs')
executor=SingleThreadedExecutor();executor.add_node(guard);executor.add_node(probe)
scan_pub=probe.create_publisher(LaserScan,'/scan',10)
cmd_pub=probe.create_publisher(Twist,'/cmd_vel_guarded',10)
foot_pubs=[probe.create_publisher(PolygonStamped,'/'+s+'_costmap/published_footprint',10) for s in ('local','global')]
latest=[]
sub=probe.create_subscription(Twist,'/cmd_vel',lambda m:latest.append([m.linear.x,m.linear.y,m.angular.z]),10)
footprints=[]
foot_sub=probe.create_subscription(PolygonStamped,'/navigation/collision_footprint',lambda m:footprints.append(m),10)
def run(seconds,xy=(.6,0),scans=True,commands=True,footprints=True):
    start=len(latest);deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        stamp=probe.get_clock().now().to_msg()
        if scans:
            msg=LaserScan();msg.header.stamp=stamp;msg.header.frame_id='laser_link'
            msg.angle_min=-math.pi;msg.angle_increment=2*math.pi/360
            msg.range_min=.05;msg.range_max=8.;msg.ranges=[float('inf')]*361
            scan_pub.publish(msg)
        if footprints:
            for pub in foot_pubs:
                msg=PolygonStamped();msg.header.stamp=stamp;pub.publish(msg)
        if commands:
            msg=Twist();msg.linear.x=float(xy[0]);msg.linear.y=float(xy[1]);msg.angular.z=.2;cmd_pub.publish(msg)
        until=time.monotonic()+.05
        while time.monotonic()<until:executor.spin_once(timeout_sec=.005)
    return latest[start:]
report={}
try:
    run(.5)
    for label,xy,expected in [('forward',(1.4*limit,0),(limit,0)),('diagonal',(limit,limit),(limit*math.sqrt(.5),limit*math.sqrt(.5))),
                               ('reverse',(-1.4*limit,0),(-limit,0)),('below_limit',(.6,.2),(.6,.2))]:
        msgs=run(.5,xy)
        actual=msgs[-1];assert math.dist(actual[:2],expected)<1e-6,(label,actual)
        assert max(math.hypot(*m[:2]) for m in msgs)<=limit+1e-6
        assert abs(actual[2]-.2)<1e-6
        report[label]=actual
    # Allow five 50 ms zero-command samples after the 0.6 s scan timeout.
    msgs=run(1.,scans=False)
    assert msgs and all(math.hypot(*m[:2])==0 for m in msgs[-5:]),'Scan loss did not stop'
    report['scan_loss_stops']=True
    run(.3)
    msgs=run(.7,commands=False)
    assert msgs and all(math.hypot(*m[:2])==0 for m in msgs[-5:]),'Command loss did not stop'
    report['command_loss_stops']=True
    run(.3)
    msgs=run(1.5,footprints=False)
    assert msgs and all(math.hypot(*m[:2])==0 for m in msgs[-5:]),'Pose loss did not stop'
    report['pose_loss_stops']=True
    assert footprints and footprints[-1].header.frame_id=='base_footprint'
    assert len(footprints[-1].polygon.points)==24
    assert all(abs(math.hypot(p.x,p.y)-.325)<1e-6 for p in footprints[-1].polygon.points)
    report['physical_footprint_radius']=.325
    report.update(passed=True,isolated_domain=166,robot_motion=False)
    out=Path(os.environ.get('AIC_GUARD_CHECK_OUTPUT','/home/polarbear/aic_round_speed_evidence_20261003/guard.json'))
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
finally:
    executor.shutdown();guard.destroy_node();probe.destroy_node();rclpy.shutdown()
