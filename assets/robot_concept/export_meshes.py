"""Export evaluated Blender visuals as metre-scale, link-local binary STL."""
import bpy, json, struct
from pathlib import Path
from mathutils import Vector
OUT=Path(__file__).resolve().parents[2]/'src/yzbot/mybot_description/meshes/concept'
OUT.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=str(Path(__file__).with_name('robot_concept.blend')))
groups={}
for obj in bpy.data.collections['ROBOT | metre scale'].objects:
    if obj.name.startswith(('left_','right_')):
        side=obj.name.split('_')[0]; link=side+'_wheel_link'; offset=(0,.227 if side=='left' else -.227,.065)
    elif obj.name.endswith('_support_ball'):
        front=obj.name.startswith('front'); link='backl_caster_link' if front else 'backr_caster_link'; offset=(.195 if front else -.195,0,.025)
    else: link='base_link'; offset=(0,0,.11)
    mat=obj.data.materials[0]
    key=(link,mat.name); groups.setdefault(key,[]).append((obj,offset))
manifest={}
deps=bpy.context.evaluated_depsgraph_get()
for (link,matname),items in groups.items():
    faces=[]
    for obj,offset in items:
        evaluated=obj.evaluated_get(deps); mesh=evaluated.to_mesh(); mesh.calc_loop_triangles()
        for tri in mesh.loop_triangles:
            vs=[obj.matrix_world @ mesh.vertices[i].co-Vector(offset) for i in tri.vertices]
            n=(vs[1]-vs[0]).cross(vs[2]-vs[0]).normalized()
            faces.append(struct.pack('<12fH',*n,*vs[0],*vs[1],*vs[2],0))
        evaluated.to_mesh_clear()
    filename=link+'_'+matname.lower().replace(' ','_')+'.stl'
    (OUT/filename).write_bytes(b'AIC concept metre-scale visual'.ljust(80,b'\0')+struct.pack('<I',len(faces))+b''.join(faces))
    rgba=list(bpy.data.materials[matname].diffuse_color)
    manifest.setdefault(link,[]).append({'file':filename,'rgba':rgba,'triangles':len(faces)})
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
print('EXPORTED',sum(len(x) for x in manifest.values()),'visual meshes')
