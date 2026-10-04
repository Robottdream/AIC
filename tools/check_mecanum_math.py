"""Verify wheel sign conventions and conservative scan projection in ROS Python."""
import importlib.util
import math
from pathlib import Path
from sensor_msgs.msg import LaserScan

path=Path(__file__).resolve().parents[1]/'src/yzbot/bot_navigation/scripts/mecanum_io.py'
spec=importlib.util.spec_from_file_location('mecanum_io',path)
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
assert all(abs(a-b)<1e-10 for a,b in zip(m.wheel_speeds(.075,0,0),[math.sqrt(.5),-math.sqrt(.5),math.sqrt(.5),-math.sqrt(.5)]))
assert all(abs(a-b)<1e-10 for a,b in zip(m.wheel_speeds(0,.075,0),[-math.sqrt(.5),-math.sqrt(.5),math.sqrt(.5),math.sqrt(.5)]))
w=m.wheel_speeds(0,0,1)
assert all(a<0 for a in w)
# Recover arbitrary planar velocities from all four radial wheel speeds.
for vx,vy,wz in ((.4,-.3,.2),(-.2,.6,-.7),(0,0,1)):
    speeds=m.wheel_speeds(vx,vy,wz)
    angles=(math.pi/4,-math.pi/4,3*math.pi/4,-3*math.pi/4)
    assert abs(sum(math.sin(a)*s*.075 for a,s in zip(angles,speeds))/2-vx)<1e-10
    assert abs(sum(-math.cos(a)*s*.075 for a,s in zip(angles,speeds))/2-vy)<1e-10
    assert abs(-sum(speeds)*.075/(4*math.hypot(.195,.195))-wz)<1e-10
s=LaserScan(); s.angle_min=-math.pi/2; s.angle_increment=.01; s.range_min=.05; s.range_max=30.; s.ranges=[1.]
result=m.project_scans({'left':s})
finite=[v for v in result if math.isfinite(v)]
assert len(finite)==1 and abs(finite[0]-math.hypot(1,.225))<1e-6
s.ranges=[math.inf]; assert all(math.isnan(v) for v in m.project_scans({'left':s}))
s.angle_min=math.pi/2; s.ranges=[.225]
assert all(math.isnan(v) for v in m.project_scans({'left':s})), 'Self-hit became obstacle/free-space'
# A missing/stale side must prevent a merged scan from refreshing the final guard.
import time
from types import SimpleNamespace
class Publisher:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(msg)
now=time.monotonic()
fake=SimpleNamespace(command=None,command_time=0.,wheel_pub=Publisher(),scan_pub=Publisher(),received={'left':now},scans={'left':s})
m.MecanumIO.tick(fake)
assert not fake.scan_pub.messages and list(fake.wheel_pub.messages[-1].data)==[0.]*4
fake.received={'left':now,'right':now-1.}
m.MecanumIO.tick(fake)
assert not fake.scan_pub.messages
print('PASS: wheel forward/lateral/yaw, endpoint projection, unknown/self-hit handling, missing/stale side stop')
import sys
sys.path.insert(0,str(path.parent))
from dynamic_obstacle_tracker import Tracker
tracker=Tracker()
tracker.tracks=[SimpleNamespace(stable=True, ident=1, velocity=(.2,0.), points=[(-.1,.8),(0.,.8),(.1,.8)])]
assert tracker.collision((0,0,0),0.,0.) is None
assert tracker.collision((0,0,0),0.,0.,lateral=.5) is not None
assert tracker.collision((0,0,0),0.,.01,lateral=.5) is not None
print('PASS: predictive guard includes straight and turning lateral motion')
