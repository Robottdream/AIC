"""Export the accepted Blender model in metre-scale URDF link coordinates."""
import bpy
import json
import struct
from pathlib import Path
from mathutils import Vector, Matrix
import math

OUT=Path(__file__).resolve().parents[2]/'src/yzbot/mybot_description/meshes/mecanum'
OUT.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=str(Path(__file__).with_name('robot_mecanum.blend')))
groups={}
for obj in bpy.data.collections['ROBOT | metre scale'].objects:
    if obj.type != 'MESH': continue
    link='base_link'; offset=(0,0,.11)
    for fore,x in [('front',.195),('rear',-.195)]:
        for side,y in [('left',.195),('right',-.195)]:
            prefix=fore+'_'+side
            if obj.name.startswith(prefix+'_') and not obj.name.endswith('_motor'):
                link=prefix+'_wheel_link'; offset=(x,y,.075)
    for side,y in [('left',.225),('right',-.225)]:
        if obj.name.startswith(side+'_lidar_'):
            link='laser_'+side+'_link'; offset=(0,y,.194)
    if obj.name in ['arm_yaw_rotating_flange','arm_yaw_output_shaft'] or obj.name.startswith('yaw_flange_bolt_'):
        link='arm_yaw_link'; offset=(0,0,.26)
    mat=obj.data.materials[0]
    groups.setdefault((link,mat.name),[]).append((obj,offset))
manifest={}
deps=bpy.context.evaluated_depsgraph_get()
for (link,matname),items in groups.items():
    faces=[]
    for obj,offset in items:
        evaluated=obj.evaluated_get(deps); mesh=evaluated.to_mesh(); mesh.calc_loop_triangles()
        for tri in mesh.loop_triangles:
            inverse_yaw=Matrix.Rotation(-math.pi/2 if link=='laser_left_link' else math.pi/2,3,'Z') if link.startswith('laser_') else Matrix.Identity(3)
            if link.endswith('_wheel_link'): inverse_yaw=Matrix.Rotation(-(math.atan2(offset[1],offset[0])-(1 if offset[1]>0 else -1)*math.pi/2),3,'Z')
            vs=[inverse_yaw @ (obj.matrix_world @ mesh.vertices[i].co-Vector(offset)) for i in tri.vertices]
            n=(vs[1]-vs[0]).cross(vs[2]-vs[0]).normalized()
            faces.append(struct.pack('<12fH',*n,*vs[0],*vs[1],*vs[2],0))
        evaluated.to_mesh_clear()
    filename=link+'_'+matname.lower().replace(' ','_')+'.stl'
    (OUT/filename).write_bytes(b'AIC mecanum metre-scale visual'.ljust(80,b'\0')+struct.pack('<I',len(faces))+b''.join(faces))
    manifest.setdefault(link,[]).append({'file':filename,'rgba':list(bpy.data.materials[matname].diffuse_color),'triangles':len(faces)})
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
lines=['<?xml version="1.0"?>','<robot xmlns:xacro="http://www.ros.org/wiki/xacro">','<xacro:macro name="mecanum_visual" params="link">']
for link,items in manifest.items():
    lines.append('<xacro:if value="${link == \''+link+'\'}">')
    for i,item in enumerate(items):
        color=' '.join(map(str,item['rgba']))
        lines.append(f'<visual><geometry><mesh filename="package://mybot_description/meshes/mecanum/{item["file"]}"/></geometry><material name="mecanum_{link}_{i}"><color rgba="{color}"/></material></visual>')
    lines.append('</xacro:if>')
lines += ['</xacro:macro>','</robot>']
(OUT.parents[1]/'urdf/mecanum_visuals.xacro').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('EXPORTED',len(manifest),'links',sum(len(x) for x in manifest.values()),'meshes')
