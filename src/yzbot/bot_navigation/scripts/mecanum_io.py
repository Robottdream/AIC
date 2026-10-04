#!/usr/bin/env python3
"""Ideal four-wheel animation and a conservative dual-scan compatibility topic.

Nav2 and Collision Monitor consume the two real scans independently. /scan is
only a compatibility projection for startup, display and the freshness guard:
unknown bins remain NaN, never free-space rays from an invented sensor origin.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray


def wheel_speeds(vx, vy, wz):
    radius=math.hypot(.195,.195)
    angles=(math.pi/4,-math.pi/4,3*math.pi/4,-3*math.pi/4)
    return [(math.sin(a)*vx-math.cos(a)*vy-radius*wz)/.075 for a in angles]



def project_scans(scans, count=1441):
    ranges=[math.nan]*count
    step=2*math.pi/(count-1)
    for side,msg in scans.items():
        sign=1 if side=='left' else -1
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or not msg.range_min <= r <= msg.range_max:
                continue
            a=msg.angle_min+i*msg.angle_increment+sign*math.pi/2
            x=r*math.cos(a); y=sign*.225+r*math.sin(a)
            # Discard self hits as unknown, without asserting empty space beyond.
            if -.34 < x < .38 and abs(y)<.305:
                continue
            distance=math.hypot(x,y)
            index=round((math.atan2(y,x)+math.pi)/step)
            if math.isnan(ranges[index]) or distance<ranges[index]: ranges[index]=distance
    return ranges


class MecanumIO(Node):
    def __init__(self):
        super().__init__('mecanum_io')
        self.scans={}; self.received={}; self.last_pair=None
        self.command=None; self.command_time=0.
        self.scan_pub=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.source_pubs={side:self.create_publisher(LaserScan,'/scan_'+side,qos_profile_sensor_data) for side in ('left','right')}
        self.wheel_pub=self.create_publisher(Float64MultiArray,'/mecanum_wheel_controller/commands',10)
        for side in ('left','right'):
            self.create_subscription(LaserScan,'/scan_'+side+'_raw',lambda m,s=side:self.scan(m,s),qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',self.cmd,10)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def scan(self,msg,side):
        sign=1 if side=='left' else -1
        ranges=list(msg.ranges)
        for i,r in enumerate(ranges):
            if not math.isfinite(r): continue
            a=msg.angle_min+i*msg.angle_increment+sign*math.pi/2
            x=r*math.cos(a); y=sign*.225+r*math.sin(a)
            if -.26<x<.32 and abs(y)<.305: ranges[i]=math.nan
        msg.ranges=ranges
        self.scans[side]=msg; self.received[side]=time.monotonic()
        self.source_pubs[side].publish(msg)

    def cmd(self,msg):
        self.command=msg; self.command_time=time.monotonic()

    def tick(self):
        now=time.monotonic()
        data=[0.]*4
        if self.command is not None and now-self.command_time<.3:
            m=self.command; data=wheel_speeds(m.linear.x,m.linear.y,m.angular.z)
        self.wheel_pub.publish(Float64MultiArray(data=data))
        if any(s not in self.received or now-self.received[s]>.25 for s in ('left','right')): return
        stamps=[self.scans[s].header.stamp.sec+self.scans[s].header.stamp.nanosec/1e9 for s in ('left','right')]
        sim=self.get_clock().now().nanoseconds/1e9
        if max(stamps)-min(stamps)>.15 or not -.1<=sim-min(stamps)<.25: return
        pair=tuple(stamps)
        if pair==self.last_pair: return
        self.last_pair=pair
        oldest=self.scans['left' if stamps[0]<=stamps[1] else 'right']
        m=LaserScan(); m.header.stamp=oldest.header.stamp; m.header.frame_id='laser_link'
        m.angle_min=-math.pi; m.angle_max=math.pi; m.angle_increment=math.tau/1440
        m.scan_time=max(v.scan_time for v in self.scans.values()); m.range_min=.05; m.range_max=30.
        m.ranges=project_scans(self.scans)
        self.scan_pub.publish(m)


def main():
    rclpy.init(); node=MecanumIO()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
