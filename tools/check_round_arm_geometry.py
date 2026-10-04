"""Sample the executed bend/lift path against conservative sensor boxes.

Primitive support points give conservative link AABBs, with analytic cylinder
bounds. This is a sampled static geometry check, not a dynamics certification.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
root=ET.parse(Path(__file__).resolve().parents[1]/'assets/robot_mecanum/arm_reference.urdf').getroot()
def rotation(axis,a):
    v=np.asarray(axis,dtype=float); v/=np.linalg.norm(v)
    x,y,z=v; k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(a)*k+(1-math.cos(a))*(k@k)
def origin(e):
    t=np.eye(4)
    if e is not None:
        t[:3,3]=list(map(float,e.get('xyz','0 0 0').split()))
        r,p,y=map(float,e.get('rpy','0 0 0').split())
        t[:3,:3]=rotation((0,0,1),y)@rotation((0,1,0),p)@rotation((1,0,0),r)
    return t
def frames(fraction):
    angles={'joint2':1.2*fraction,'joint3':1.17*fraction,'joint5':-.3*fraction,'finger_joint1':0.}
    result={'arm_base_link':np.eye(4)}; result['arm_base_link'][2,3]=.28
    pending=list(root.findall('joint'))
    while pending:
        for j in pending[:]:
            parent=j.find('parent').get('link')
            if parent not in result: continue
            t=origin(j.find('origin')); axis=j.find('axis'); a=angles.get(j.get('name'),0.)
            if axis is not None:
                v=list(map(float,axis.get('xyz').split()))
                if j.get('type')=='prismatic': t[:3,3]+=t[:3,:3]@np.array(v)*a
                else: t[:3,:3]=t[:3,:3]@rotation(v,a)
            result[j.find('child').get('link')]=result[parent]@t; pending.remove(j)
    return result
def bounds(link,t):
    all_bounds=[]
    for c in link.findall('collision'):
        m=t@origin(c.find('origin')); g=c.find('geometry'); box=g.find('box'); cylinder=g.find('cylinder')
        if box is not None: extent=np.abs(m[:3,:3])@(np.array(list(map(float,box.get('size').split())))/2)
        elif cylinder is not None:
            radius=float(cylinder.get('radius')); length=float(cylinder.get('length'))
            extent=radius*np.sqrt(m[:3,0]**2+m[:3,1]**2)+length/2*np.abs(m[:3,2])
        else: continue
        all_bounds.append((m[:3,3]-extent,m[:3,3]+extent))
    return all_bounds
sensors=[('left_lidar',np.array([-.044,.14,.164]),np.array([.044,.263,.212])),('right_lidar',np.array([-.044,-.263,.164]),np.array([.044,-.14,.212])),('camera',np.array([.137,-.036,.168]),np.array([.178,.036,.209]))]
overlaps=[]; lowest=1.; lifted_radius=0.; end=None
loaded_overlaps=[]; loaded_radius=0.
# Existing Gazebo attachment preserves the cube's relative transform. Check
# representative pickup distances allowed by the 0.15m arrival tolerance.
bend=frames(1.)['link6']
cube_offsets=[np.linalg.inv(bend)@np.array([distance,0,.75,1.]) for distance in (.40,.55,.70)]
for fraction in np.linspace(0,1,101):
    f=frames(fraction)
    for yaw in np.linspace(-math.pi,math.pi,73):
        rz=np.eye(4); rz[:3,:3]=rotation((0,0,1),yaw)
        for offset in cube_offsets:
            center=(rz@f['link6']@offset)[:3]; extent=np.abs((rz@f['link6'])[:3,:3])@np.array([.015]*3)
            lo,hi=center-extent,center+extent
            if fraction==0: loaded_radius=max(loaded_radius,math.hypot(max(abs(lo[0]),abs(hi[0])),max(abs(lo[1]),abs(hi[1]))))
            for name,slo,shi in sensors:
                if np.all(hi>slo) and np.all(lo<shi): loaded_overlaps.append((name,float(fraction),float(yaw)))
        for link in root.findall('link'):
            for lo,hi in bounds(link,rz@f[link.get('name')]):
                lowest=min(lowest,float(lo[2]))
                if fraction==0: lifted_radius=max(lifted_radius,math.hypot(max(abs(lo[0]),abs(hi[0])),max(abs(lo[1]),abs(hi[1]))))
                for name,slo,shi in sensors:
                    if np.all(hi>slo) and np.all(lo<shi): overlaps.append((link.get('name'),name,float(fraction),float(yaw)))
    if fraction==1: end=f['link6'][:3,3].tolist()
result={'sampled_poses':101*73,'sensor_AABB_overlaps':overlaps[:20],'overlap_count':len(overlaps),'loaded_sensor_overlap_count':len(loaded_overlaps),'minimum_arm_z_m':lowest,'lifted_arm_AABB_radius_m':lifted_radius,'lifted_loaded_AABB_radius_m':loaded_radius,'bend_link6_origin_m':end,'method':'Analytic primitive AABBs; 101 bend fractions x 73 pedestal angles; 3cm cube at pickup distances 0.40/0.55/0.70m, z=0.75, preserved attachment transform'}
print(json.dumps(result,indent=2))
assert not overlaps, 'Possible arm/sensor intersection; review before runtime'
assert not loaded_overlaps, 'Possible carried cube/sensor intersection'
