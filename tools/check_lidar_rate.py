"""Measure the real Gazebo scans and compatibility freshness without motion."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
rclpy.init();n=rclpy.create_node('speed_lidar_rate_probe');rows={};sim=[];subs=[]
def scan(m,k):
    rows.setdefault(k,[]).append({'wall':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
      'count':len(m.ranges),'min_angle':m.angle_min,'max_angle':m.angle_max,'scan_time':m.scan_time})
for name in ('scan_left_raw','scan_right_raw','scan_left','scan_right','scan'):
    subs.append(n.create_subscription(LaserScan,'/'+name,lambda m,k=name:scan(m,k),qos_profile_sensor_data))
subs.append(n.create_subscription(Clock,'/clock',lambda m:sim.append((time.monotonic(),m.clock.sec+m.clock.nanosec/1e9)),qos_profile_sensor_data))
end=time.monotonic()+12
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
report={}
for name,v in rows.items():
    assert len(v)>5,name
    report[name]={'points':v[-1]['count'],'simulation_hz':(len(v)-1)/(v[-1]['stamp']-v[0]['stamp']),
      'wall_hz':(len(v)-1)/(v[-1]['wall']-v[0]['wall']),
      'max_wall_interval':max(b['wall']-a['wall'] for a,b in zip(v,v[1:])),
      'max_sim_interval':max(b['stamp']-a['stamp'] for a,b in zip(v,v[1:])),
      'span_degrees':(v[-1]['max_angle']-v[-1]['min_angle'])*180/3.141592653589793}
report['real_time_factor']=(sim[-1][1]-sim[0][1])/(sim[-1][0]-sim[0][0])
report['max_clock_wall_interval']=max(b[0]-a[0] for a,b in zip(sim,sim[1:]))
report['continuous_wall_freshness']=report['scan']['max_wall_interval']<.6
report['passed']=all(report[k]['points']==811 and 7.5<report[k]['simulation_hz']<8.5 for k in ('scan_left_raw','scan_right_raw'))
Path('/home/polarbear/aic_speed_ladder_20261003/lidar-live.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True);n.destroy_node();rclpy.shutdown()
assert report['passed'],'Configured scan rate/count failed; wall stalls remain separately reported'
