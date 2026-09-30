"""Apply the accepted visual model and matching primitive physics geometry."""
import json, re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
DESC=ROOT/'src/yzbot/mybot_description'
BACKUP=ROOT/'log/robot-concept-integration/before'
def edit(relative,fn):
    p=ROOT/relative; text=p.read_text(encoding='utf-8')
    backup=BACKUP/relative; backup.parent.mkdir(parents=True,exist_ok=True)
    if not backup.exists(): backup.write_text(text,encoding='utf-8')
    p.write_text(fn(text),encoding='utf-8',newline='\n')

manifest=json.loads((DESC/'meshes/concept/manifest.json').read_text())
xml=['<?xml version="1.0"?>','<robot xmlns:xacro="http://www.ros.org/wiki/xacro">','<xacro:macro name="concept_visual" params="link">']
for link,parts in manifest.items():
    xml.append('<xacro:if value="${link == \''+link+'\'}">')
    for i,p in enumerate(parts):
        xml.append('<visual><geometry><mesh filename="package://mybot_description/meshes/concept/'+p['file']+'"/></geometry><material name="concept_'+link+'_'+str(i)+'"><color rgba="'+' '.join(str(v) for v in p['rgba'])+'"/></material></visual>')
    xml.append('</xacro:if>')
xml+=['</xacro:macro>','</robot>']
(DESC/'urdf/concept_visuals.xacro').write_text('\n'.join(xml)+'\n')

def base(t):
    t=t.replace('value="0.480" />\n    <xacro:property name="base_length"','value="0.400" />\n    <xacro:property name="base_length"')
    t=t.replace('value="${(base_width+wheel_length)/2}"','value="${(base_width+wheel_length)/2 + 0.002}"')
    t=t.replace('value="${base_height/2 - 0.015}"','value="${base_height/2 - 0.005}"')
    t=t.replace('value="${(wheel_joint_z + wheel_radius - base_height/2)/2}"','value="0.025"')
    t=t.replace('value="${base_length/2 - caster_radius}"','value="0.195"')
    t=t.replace('${base_height/2 + caster_radius*2}','${wheel_radius + wheel_joint_z}')
    t=t.replace('${-(base_height/2 + caster_radius)}','${caster_radius - wheel_radius - wheel_joint_z}')
    t=t.replace('xmlns:xacro="http://www.ros.org/wiki/xacro">','xmlns:xacro="http://www.ros.org/wiki/xacro">\n    <xacro:include filename="$(find mybot_description)/urdf/concept_visuals.xacro" />',1)
    for name,link in [('${prefix}_wheel_link','${prefix}_wheel_link'),('${prefix}_caster_link','${prefix}_caster_link'),('base_link','base_link')]:
        pattern='(<link name="'+re.escape(name)+'">)\\s*<visual>.*?</visual>'
        t,n=re.subn(pattern,lambda m:m[1]+'\n            <xacro:concept_visual link="'+link+'" />',t,count=1,flags=re.S)
        assert n==1,(name,n)
    # Hull is body only: wheels have their own cylindrical collisions.
    t=t.replace('${base_width+wheel_length*2}','${base_width}')
    # Mast, mounting deck and plate are simple collision primitives.
    marker='        <xacro:box_inertia m="${base_mass}"'
    pos=t.index(marker)
    t=t[:pos]+'''        <collision name="upper_deck"><origin xyz="0 0 0.054"/><geometry><box size="0.43 0.35 0.008"/></geometry></collision>
        <collision name="arm_plate"><origin xyz="0 0 0.064"/><geometry><box size="0.13 0.13 0.012"/></geometry></collision>
        <collision name="sensor_mast"><origin xyz="-0.19 0 0.293"/><geometry><box size="0.018 0.025 0.47"/></geometry></collision>
        <collision name="high_lidar"><origin xyz="-0.19 0 0.55"/><geometry><cylinder radius="0.0355" length="0.034"/></geometry></collision>
'''+t[pos:]
    return t
edit('src/yzbot/mybot_description/urdf/large_base.xacro',base)
def drive(t):
    t=t.replace('value="${base_height/2+0.01}"','value="0.09"')
    t=t.replace('<material>Gazebo/Blue</material>\n        <kp>','<kp>')
    t=t.replace('<material>Gazebo/Gray</material>','')
    t=t.replace('<material>Gazebo/Black</material>\n        <mu1>','<mu1>')
    return t
edit('src/yzbot/mybot_description/urdf/originbot_with_arm_gazebo.xacro',drive)
def sensors(t):
    t=t.replace('value="${base_length/2+0.02}"','value="0.155"').replace('value="-0.01"','value="0.08"')
    t=t.replace('<xacro:mipi_camera prefix="camera" />','<xacro:mipi_camera prefix="camera" show_visual="false" />')
    t=t.replace('<xacro:lidar prefix="laser" gz_visualize="false" range_min="${robot_radius + safety_margin}" />','''<xacro:lidar prefix="laser" gz_visualize="false" range_min="${robot_radius + safety_margin}" show_visual="false" />
    <!-- High scanner is available for inspection; navigation retains low /scan. -->
    <joint name="high_lidar_joint" type="fixed"><origin xyz="-0.19 0 0.55"/><parent link="base_link"/><child link="laser_high_link"/></joint>
    <xacro:lidar prefix="laser_high" topic="scan_high" range_min="0.55" show_visual="false" />''',1)
    return t
edit('src/yzbot/mybot_description/urdf/originbot_with_rgbd_gazebo_arm.xacro',sensors)
def lidar(t):
    t=t.replace('range_max:=30.0"','range_max:=30.0 topic:=scan show_visual:=true"')
    t=t.replace('<visual>','<xacro:if value="${show_visual}"><visual>',1).replace('</visual>','</visual></xacro:if>',1)
    t=t.replace('<sensor type="ray" name="lidar">','<sensor type="ray" name="${prefix}_lidar">').replace('name="gazebo_lidar"','name="${prefix}_gazebo_lidar"').replace('~/out:=scan','~/out:=${topic}')
    return t
edit('src/yzbot/mybot_description/urdf/lidar_gazebo.xacro',lidar)
def camera(t):
    t=t.replace('params="prefix:=camera"','params="prefix:=camera show_visual:=true"')
    t=t.replace('<visual>','<xacro:if value="${show_visual}"><visual>',1).replace('</visual>','</visual></xacro:if>',1)
    return t
edit('src/yzbot/mybot_description/urdf/camera_gazebo.xacro',camera)
def setup(t):
    return t.replace('        # world files','        (os.path.join("share", package_name, "meshes", "concept"), glob(os.path.join("meshes", "concept", "*"))),\n        # world files')
edit('src/yzbot/mybot_description/setup.py',setup)
for name in ['originbot_nav2.yaml','originbot_nav2_odom.yaml']:
    edit('src/yzbot/bot_navigation/param/'+name,lambda t:t.replace('[[0.26, 0.31], [0.26, -0.31], [-0.26, -0.31], [-0.26, 0.31]]','[[0.32, 0.276], [0.32, -0.276], [-0.26, -0.276], [-0.26, 0.276]]'))
print('Integrated accepted concept. Backups:',BACKUP)
