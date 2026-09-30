"""Run with Blender --background --python build_model.py. Units: metres."""
import bpy, math, json, xml.etree.ElementTree as ET
from pathlib import Path
from mathutils import Vector, Matrix, Euler

OUT = Path(__file__).resolve().parent
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.length_unit = 'METERS'

def material(name, color, metallic=0, roughness=.4):
    m=bpy.data.materials.new(name); m.diffuse_color=(*color,1); m.use_nodes=True
    p=m.node_tree.nodes.get('Principled BSDF'); p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Metallic'].default_value=metallic; p.inputs['Roughness'].default_value=roughness
    return m
blue=material('Blue gray chassis',(.075,.18,.27),.55)
silver=material('Light alloy',(.65,.72,.76),.65)
rubber=material('Dark rubber',(.025,.03,.035),0,.8)
dark=material('Sensor housing',(.055,.07,.085),.3)
glass=material('Optical window',(.06,.25,.32),.6,.18)
orange=material('Front identification',(.95,.33,.065),.3)

robot=bpy.data.collections.new('ROBOT | metre scale'); scene.collection.children.link(robot)
arm=bpy.data.collections.new('ARM | URDF reference, illustrative transport pose'); scene.collection.children.link(arm)
studio=bpy.data.collections.new('STUDIO | excluded from robot dimensions'); scene.collection.children.link(studio)
def finish(o,name,mat,col=robot):
    o.name=name; o.data.materials.append(mat)
    for c in list(o.users_collection): c.objects.unlink(o)
    col.objects.link(o)
    return o
def box(name,loc,size,mat,bevel=0,col=robot):
    bpy.ops.mesh.primitive_cube_add(size=1,location=loc); o=finish(bpy.context.object,name,mat,col)
    o.dimensions=size; bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    if bevel:
        m=o.modifiers.new('Manufactured edge radius','BEVEL'); m.width=bevel; m.segments=5
        o.modifiers.new('Weighted normals','WEIGHTED_NORMAL')
    return o
def cylinder(name,loc,radius,depth,mat,axis='Z',col=robot):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=radius,depth=depth,location=loc)
    o=finish(bpy.context.object,name,mat,col)
    if axis=='Y': o.rotation_euler.x=math.pi/2
    for p in o.data.polygons: p.use_smooth=True
    m=o.modifiers.new('Edge finish','BEVEL'); m.width=.001; m.segments=3
    o.modifiers.new('Weighted normals','WEIGHTED_NORMAL')
    return o

# Z=0 ground; X forward; Y left. Wheel inner edges at +/- .202:
# 2 mm clearance to 400 mm body, 504 mm total track envelope.
box('chassis_480x400x100', (0,0,.11),(.48,.40,.10),blue,.025)
box('upper_deck',(0,0,.164),(.43,.35,.008),silver,.018)
box('arm_mount_plate',(0,0,.174),(.13,.13,.012),silver,.008)
for sign,label in [(1,'left'),(-1,'right')]:
    y=sign*.227
    cylinder(label+'_drive_wheel',(0,y,.065),.065,.05,rubber,'Y')
    cylinder(label+'_wheel_hub',(0,sign*.253,.065),.032,.003,silver,'Y')
    cylinder(label+'_hub_cap',(0,sign*.255,.065),.014,.002,blue,'Y')
    # Tread grooves remain inside nominal tyre diameter.
    for i in range(28):
        a=2*math.pi*i/28
        o=box(label+'_tread_%02d'%i,(.063*math.sin(a),y,.065+.063*math.cos(a)),(.004,.046,.003),dark,.0008)
        o.rotation_euler.y=a
for x,label in [(.195,'front'),(-.195,'rear')]:
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32,ring_count=16,radius=.025,location=(x,0,.025))
    finish(bpy.context.object,label+'_support_ball',rubber)
    cylinder(label+'_support_socket',(x,0,.048),.029,.018,silver)
    cylinder(label+'_support_mount',(x,0,.059),.018,.014,dark)
for x in [-.048,.048]:
    for y in [-.048,.048]: cylinder('mount_fastener',(x,y,.182),.004,.004,dark)
box('front_marker',(.2405,0,.113),(.002,.12,.017),orange,.001)
box('rear_sensor_mast',(-.19,0,.403),(.018,.025,.47),silver,.003)
cylinder('lidar_mount',(-.19,0,.64),.037,.006,silver)
cylinder('lidar_housing',(-.19,0,.66),.035,.034,dark)
cylinder('lidar_scan_band',(-.19,0,.66),.0355,.009,glass)
box('camera_housing',(.155,0,.19),(.033,.07,.035),dark,.004)
box('camera_mount',(.155,0,.1705),(.025,.05,.005),silver,.001)
for y in [-.02,.02]:
    o=cylinder('camera_lens',(.174,y,.19),.01,.006,glass); o.rotation_euler.y=math.pi/2

# Preserve original link geometry and joint frames, rather than guessing arm lengths.
root=ET.parse(OUT/'arm_reference.urdf').getroot()
def origin(el):
    if el is None: return Matrix.Identity(4)
    xyz=[float(v) for v in el.get('xyz','0 0 0').split()]
    rpy=[float(v) for v in el.get('rpy','0 0 0').split()]
    return Matrix.Translation(xyz) @ Euler(rpy,'XYZ').to_matrix().to_4x4()
frames={'arm_base_link':Matrix.Translation((0,0,.20))}
angles={'joint2':-.65,'joint3':1.65,'joint4':0,'joint5':-.8,'finger_joint1':.2}
pending=list(root.findall('joint'))
while pending:
    changed=False
    for j in pending[:]:
        parent=j.find('parent').get('link'); child=j.find('child').get('link')
        if parent not in frames: continue
        axis=Vector([float(v) for v in j.find('axis').get('xyz').split()]) if j.find('axis') is not None else Vector((0,0,1))
        rot=Matrix.Rotation(angles.get(j.get('name'),0),4,axis)
        frames[child]=frames[parent] @ origin(j.find('origin')) @ rot
        pending.remove(j); changed=True
    if not changed: raise RuntimeError('Disconnected arm reference tree')
for link in root.findall('link'):
    for i,v in enumerate(link.findall('visual')):
        geo=v.find('geometry'); name='reference_'+link.get('name')+'_'+str(i)
        mat=silver if link.get('name') in ['arm_base_link','link2','link4'] else blue
        if geo.find('box') is not None:
            size=tuple(float(a) for a in geo.find('box').get('size').split()); o=box(name,(0,0,0),size,mat,.002,arm)
        elif geo.find('cylinder') is not None:
            g=geo.find('cylinder'); o=cylinder(name,(0,0,0),float(g.get('radius')),float(g.get('length')),mat,col=arm)
        else: continue
        o.matrix_world=frames[link.get('name')] @ origin(v.find('origin'))
        o['reference_only']=True

box('studio_floor',(0,0,-.015),(200,200,.03),material('Studio',(.18,.21,.25),0,.85),col=studio)
scene.render.engine='CYCLES'; scene.cycles.samples=32
scene.render.resolution_x=1400; scene.render.resolution_y=1100; scene.render.resolution_percentage=100
scene.world=bpy.data.worlds.new('Studio world'); scene.world.color=(.25,.25,.25)
for name,loc,power,size in [('key',(1,-1,2),150,1.5),('fill',(-1,-.5,1),90,1),('rim',(-.6,1,1.5),180,1)]:
    data=bpy.data.lights.new(name,'AREA'); data.energy=power; data.shape='DISK'; data.size=size
    o=bpy.data.objects.new(name,data); studio.objects.link(o); o.location=loc
    o.rotation_euler=(Vector((0,0,.2))-o.location).to_track_quat('-Z','Y').to_euler()
def camera(name,loc,target,ortho=None):
    data=bpy.data.cameras.new(name); o=bpy.data.objects.new(name,data); studio.objects.link(o)
    o.location=loc; o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler()
    if ortho: data.type='ORTHO'; data.ortho_scale=ortho
    else: data.lens=52
    return o
cameras=[camera('front_perspective',(1.4,-1.45,1.05),(0,0,.32)),camera('rear_perspective',(-1.4,1.45,1.05),(0,0,.32)),camera('top',(0,0,1.8),(0,0,0),.9),camera('side',(0,-1.8,.34),(0,0,.34),1.1)]
scene.camera=cameras[0]
scene['notes']='Concept only. No dynamics, collision or manipulator workspace validation. X forward / Y left / Z up.'
scene['dimensions']='Body 0.480 x 0.400 x 0.100 m; tyre envelope width 0.504 m; hubs width 0.512 m.'
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D':
            area.spaces.active.region_3d.view_distance=1.2
            area.spaces.active.region_3d.view_location=(0,0,.22)
            area.spaces.active.region_3d.view_rotation=cameras[0].rotation_euler.to_quaternion()
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'robot_concept.blend'))
for cam in cameras:
    scene.camera=cam; scene.render.filepath=str(OUT/(cam.name+'.png')); bpy.ops.render.render(write_still=True)
def bounds(collection):
    vs=[o.matrix_world @ Vector(v) for o in collection.objects if o.type=='MESH' for v in o.bound_box]
    return {'min':[min(v[i] for v in vs) for i in range(3)],'max':[max(v[i] for v in vs) for i in range(3)]}
checks={'body_m':[.48,.40,.10],'wheel_radius_m':.065,'wheel_width_m':.05,'wheel_to_body_gap_m':.002,'wheel_ground_z_m':0,'support_ground_z_m':0,'robot_bounds':bounds(robot),'arm_reference_bounds':bounds(arm),'validation':'Geometry concept; physics and full workspace unverified'}
checks['lidar_scan_plane_m']=.66
checks['lidar_above_reference_arm_m']=.66-checks['arm_reference_bounds']['max'][2]
assert checks['lidar_above_reference_arm_m']>.02, 'Arm obstructs nominal lidar scan plane'
assert abs(checks['robot_bounds']['min'][2])<1e-6, 'Ground alignment incorrect'
assert checks['robot_bounds']['max'][1]-checks['robot_bounds']['min'][1]<.513
(OUT/'geometry_checks.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
print('CONCEPT_COMPLETE',json.dumps(checks))
