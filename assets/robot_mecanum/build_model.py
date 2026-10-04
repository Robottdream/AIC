"""Run with Blender --background --python build_model.py. Units: metres."""
import bpy, math, json, sys, xml.etree.ElementTree as ET
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

# Z=0 ground; X forward; Y left.
cylinder('round_chassis_d480', (0,0,.11),.24,.10,blue)
cylinder('round_upper_deck',(0,0,.164),.225,.008,silver)
box('arm_mount_plate',(0,0,.174),(.13,.13,.012),silver,.008)
# Four omni wheels: radial axles and passive rollers tangent to each wheel.
for x, fore in [(.195,'front'),(-.195,'rear')]:
    for sign, side in [(1,'left'),(-1,'right')]:
        y=sign*.245; label=fore+'_'+side
        before=set(robot.objects)
        cylinder(label+'_motor',(x,sign*.1955,.075),.035,.055,dark,'Y')
        cylinder(label+'_rim',(x,y,.075),.051,.052,silver,'Y')
        cylinder(label+'_hub',(x,sign*.276,.075),.025,.01,blue,'Y')
        handed=sign*(1 if x>0 else -1)
        for i in range(10):
            a=2*math.pi*i/10
            radial=Vector((math.sin(a),0,math.cos(a)))
            tangent=Vector((math.cos(a),0,-math.sin(a)))
            axis=tangent.normalized()
            loc=Vector((x,y,.075))+.06*radial
            bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,radius=1,location=loc)
            o=finish(bpy.context.object,label+'_roller_%02d'%i,rubber)
            o.scale=(.015,.015,.043)
            o.rotation_euler=axis.to_track_quat('Z','Y').to_euler()
            o['roller_angle_degrees']=90
            o['wheel_position']=label
        for a in range(6):
            ang=a*math.tau/6
            cylinder(label+'_hub_bolt_%d'%a,(x+.018*math.sin(ang),sign*.283,.075+.018*math.cos(ang)),.0025,.003,dark,'Y')
        # Transform the reference Y-axis wheel into a radial axle.
        bpy.context.view_layer.update()
        theta=math.atan2(sign*.195,x); turn=Matrix.Rotation(theta-sign*math.pi/2,4,'Z')
        pivot=Vector((x,y,.075)); center=Vector((x,sign*.195,.075))
        transform=Matrix.Translation(center) @ turn @ Matrix.Translation(-pivot)
        for part in set(robot.objects)-before: part.matrix_world=transform @ part.matrix_world
# Coaxial electric rotary actuator and bearing below the original arm.
cylinder('arm_yaw_motor_housing',(0,0,.207),.059,.054,dark)
cylinder('arm_yaw_motor_band',(0,0,.21),.060,.012,orange)
cylinder('arm_yaw_bearing',(0,0,.241),.073,.014,silver)
cylinder('arm_yaw_rotating_flange',(0,0,.254),.065,.012,blue)
cylinder('arm_yaw_output_shaft',(0,0,.257),.024,.02,silver)
for i in range(8):
    a=math.tau*i/8
    cylinder('yaw_flange_bolt_%d'%i,(.053*math.cos(a),.053*math.sin(a),.262),.003,.004,dark)
rotor=bpy.data.objects.new('ARM_YAW | motor driven Z rotation',None)
arm.objects.link(rotor); rotor.location=(0,0,.26)
rotor['axis']='Z'; rotor['actuator']='Coaxial geared electric motor'; rotor['range']='Concept: +/-180 degrees; cable routing to be engineered'
for x in [-.048,.048]:
    for y in [-.048,.048]: cylinder('mount_fastener',(x,y,.182),.004,.004,dark)
box('front_marker',(.2405,0,.113),(.002,.12,.017),orange,.001)
# Low left/right sensors sit between the wheels and behind the camera.
# Brackets overlap the deck edge; outward scan sectors must be validated
# with the real sensor FoV and self-filtering before ROS integration.
lidar_parts=[]
for sign,label in [(1,'left'),(-1,'right')]:
    y=sign*.225
    lidar_parts.append(box(label+'_lidar_bracket',(0,sign*.200,.169),(.086,.120,.008),silver,.004))
    lidar_parts.append(cylinder(label+'_lidar_mount',(0,y,.176),.037,.006,silver))
    lidar_parts.append(cylinder(label+'_lidar_housing',(0,y,.194),.035,.034,dark))
    lidar_parts.append(cylinder(label+'_lidar_scan_band',(0,y,.194),.0355,.009,glass))
    for x in [-.030,.030]:
        lidar_parts.append(cylinder(label+'_lidar_fastener',(x,sign*.175,.175),.003,.004,dark))
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
frames={'arm_base_link':Matrix.Translation((0,0,.28))}
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
        world=o.matrix_world.copy(); o.parent=rotor; o.matrix_world=world

box('studio_floor',(0,0,-.015),(200,200,.03),material('Studio',(.18,.21,.25),0,.85),col=studio)
scene.render.engine='CYCLES'; scene.cycles.samples=24
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
scene['notes']='Round omni revision. Separate sampled arm/sensor geometry and Gazebo tests in docs/round-omni-20261003.md. X forward / Y left / Z up.'
scene['dimensions']='Round body diameter 0.480 m, height 0.100 m; four radial-axis omni wheels; motorized arm yaw pedestal; low left/right lidar.'
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D':
            area.spaces.active.region_3d.view_distance=1.2
            area.spaces.active.region_3d.view_location=(0,0,.22)
            area.spaces.active.region_3d.view_rotation=cameras[0].rotation_euler.to_quaternion()
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'robot_round_omni.blend'))
# Existing mesh export/integration paths remain a compatibility alias.
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'robot_mecanum.blend'))
for cam in ([] if '--check-only' in sys.argv else cameras):
    scene.camera=cam; scene.render.filepath=str(OUT/(cam.name+'.png')); bpy.ops.render.render(write_still=True)
def bounds(collection):
    vs=[o.matrix_world @ Vector(v) for o in collection.objects if o.type=='MESH' for v in (vert.co for vert in o.data.vertices)]
    return {'min':[min(v[i] for v in vs) for i in range(3)],'max':[max(v[i] for v in vs) for i in range(3)]}
checks={'body_m':[.48,.48,.10],'wheel_count':4,'rollers_per_wheel':10,'roller_angle_deg':90,'drive':'four radial-axis omni wheels','body_diameter_m':.48,'wheel_center_z_m':.075,'wheel_center_track_m':.39,'arm_yaw_motor':True,'robot_bounds':bounds(robot),'arm_reference_bounds':bounds(arm),'validation':'Geometry concept; physics and full workspace unverified'}
checks['lidar_count']=2
checks['lidar_centers_m']=[[0,.225,.194],[0,-.225,.194]]
checks['lidar_scan_plane_m']=.194
checks['lidar_assembly_top_m']=max((o.matrix_world @ v.co).z for o in lidar_parts for v in o.data.vertices)
checks['reference_pose_yaw_sweep_vertical_gap_m']=checks['arm_reference_bounds']['min'][2]-checks['lidar_assembly_top_m']
checks['sweep_validation']='Vertical separation holds for any Z yaw of this fixed arm pose only; articulated and loaded sweeps unverified.'
lidar_vertices=[o.matrix_world @ v.co for o in lidar_parts for v in o.data.vertices]
checks['camera_forward_axis']='Local +X; conceptual optics, actual FoV unverified'
checks['camera_lens_front_x_m']=.177
checks['lidar_assembly_front_x_m']=max(v.x for v in lidar_vertices)
checks['lidar_behind_camera_plane_gap_m']=.177-checks['lidar_assembly_front_x_m']
wheel_vertices=[o.matrix_world @ v.co for o in robot.objects if o.type=='MESH' and any(o.name.startswith(p) for p in ['front_left_','front_right_','rear_left_','rear_right_']) for v in o.data.vertices]
checks['lidar_wheel_longitudinal_gap_m']=min(abs(v.x) for v in wheel_vertices)-max(abs(v.x) for v in lidar_vertices)
assert checks['lidar_behind_camera_plane_gap_m']>.10, 'Lidar assembly projects in front of camera'
# Sensors are above the wheels; radial wheel layout needs 3D clearance checks.
assert checks['reference_pose_yaw_sweep_vertical_gap_m']>.04, 'Lidar too high for reference pose yaw sweep'
assert not any('mast' in o.name for o in robot.objects), 'Tall sensor mast remains'
assert checks['robot_bounds']['min'][2]>=-0.002, 'Wheel below floor'
assert checks['robot_bounds']['max'][1]-checks['robot_bounds']['min'][1]<.60
(OUT/'geometry_checks.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
print('CONCEPT_COMPLETE',json.dumps(checks))

checks['max_fixed_body_radius_m']=max(math.hypot((o.matrix_world@v.co).x,(o.matrix_world@v.co).y) for o in robot.objects if o.type=='MESH' for v in o.data.vertices)
assert checks['max_fixed_body_radius_m']<.325*math.cos(math.pi/24)
(OUT/'geometry_checks.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
