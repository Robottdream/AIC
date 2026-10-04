"""Generate new-model files and add opt-in launch selection. Run in WSL."""
from pathlib import Path
import yaml

root=Path(__file__).resolve().parents[1]
if "round_chassis_d480" in (root/'assets/robot_mecanum/build_model.py').read_text():
    raise SystemExit('Historical mecanum bootstrap: round-omni files are now maintained directly. Use build_mecanum_test.sh.')
desc=root/'src/yzbot/mybot_description'
nav=root/'src/yzbot/bot_navigation'
mybot=root/'src/yzbot/mybot'

def save(path,text): path.write_text(text,encoding='utf-8',newline='\n')
def edit(path,old,new):
    text=path.read_text()
    if new in text: return
    if old not in text: raise RuntimeError(f'Patch anchor absent: {path}: {old[:60]}')
    save(path,text.replace(old,new))

# Keep the old robot available unchanged for comparison.
text=(desc/'urdf/originbot_with_arm_gazebo.xacro').read_text()
text=text.replace('/urdf/large_base.xacro','/urdf/mecanum_base.xacro')
text=text.replace('name="arm_offset_z" value="0.09"','name="arm_offset_z" value="0.02"')
text=text.replace('<parent link="base_link" />\n        <child link="arm_base_link" />','<parent link="arm_yaw_link" />\n        <child link="arm_base_link" />')
text=text.replace('<xacro:ros2_control_joint joint_name="joint1" />','<joint name="arm_yaw_joint"><command_interface name="position"/><state_interface name="position"><param name="initial_value">0.0</param></state_interface><state_interface name="velocity"/></joint>\n        <xacro:ros2_control_joint joint_name="joint1" />')
wheel_control=''
for prefix in ['front_left','front_right','rear_left','rear_right']:
    wheel_control+=f'<joint name="{prefix}_wheel_joint"><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/></joint>\n'
text=text.replace('</ros2_control>',wheel_control+'</ros2_control>')
text=text.replace('/config/ros2_controllers.yaml','/config/ros2_controllers_mecanum.yaml')
text=text[:text.index('    <gazebo reference="backl_caster_link">')]+'''
    <gazebo><plugin name="ideal_mecanum_planar_drive" filename="libgazebo_ros_planar_move.so">
      <update_rate>100</update_rate><publish_rate>30</publish_rate>
      <publish_odom>true</publish_odom><publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame><robot_base_frame>base_footprint</robot_base_frame>
    </plugin></gazebo>
</robot>
'''
save(desc/'urdf/mecanum_with_arm_gazebo.xacro',text)
text=(desc/'urdf/originbot_with_rgbd_gazebo_arm.xacro').read_text()
text=text.replace('/urdf/originbot_with_arm_gazebo.xacro','/urdf/mecanum_with_arm_gazebo.xacro')
start=text.index('    <!-- Lidar -->'); end=text.index('    <!-- IMU -->',start)
text=text[:start]+'''
    <!-- Virtual origin for compatibility /scan only. Costmaps use real frames. -->
    <joint name="lidar_joint" type="fixed"><origin xyz="0 0 0.084"/><parent link="base_link"/><child link="laser_link"/></joint>
    <link name="laser_link"/>
    <joint name="left_lidar_joint" type="fixed"><origin xyz="0 0.225 0.084" rpy="0 0 ${M_PI/2}"/><parent link="base_link"/><child link="laser_left_link"/></joint>
    <xacro:lidar prefix="laser_left" topic="scan_left_raw" range_min="0.05" show_visual="false" samples="1081" min_angle="${-3*M_PI/4}" max_angle="${3*M_PI/4}"/>
    <joint name="right_lidar_joint" type="fixed"><origin xyz="0 -0.225 0.084" rpy="0 0 ${-M_PI/2}"/><parent link="base_link"/><child link="laser_right_link"/></joint>
    <xacro:lidar prefix="laser_right" topic="scan_right_raw" range_min="0.05" show_visual="false" samples="1081" min_angle="${-3*M_PI/4}" max_angle="${3*M_PI/4}"/>
    <gazebo reference="laser_left_link"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
    <gazebo reference="laser_right_link"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
''' +text[end:]
save(desc/'urdf/originbot_mecanum_gazebo.xacro',text)
edit(desc/'urdf/lidar_gazebo.xacro','topic:=scan show_visual:=true"','topic:=scan show_visual:=true samples:=1441 min_angle:=-3.14159 max_angle:=3.14159"')
edit(desc/'urdf/lidar_gazebo.xacro','<samples>1441</samples>','<samples>${samples}</samples>')
edit(desc/'urdf/lidar_gazebo.xacro','<min_angle>-3.14159</min_angle>','<min_angle>${min_angle}</min_angle>')
edit(desc/'urdf/lidar_gazebo.xacro','<max_angle>3.14159</max_angle>','<max_angle>${max_angle}</max_angle>')
# Link-local Blender scanner meshes need the inverse of the sensor's outward yaw.
edit(desc/'urdf/lidar_gazebo.xacro','<xacro:if value="${show_visual}"><visual>', '''<xacro:if value="${prefix == 'laser_left' or prefix == 'laser_right'}">
            <xacro:mecanum_visual link="${prefix}_link"/>
            <collision name="mecanum_scanner_housing"><geometry><cylinder radius="0.0355" length="0.034"/></geometry></collision>
            <collision name="mecanum_scanner_bracket"><origin xyz="-0.025 0 -0.025"/><geometry><box size="0.12 0.086 0.008"/></geometry></collision>
            </xacro:if>
            <xacro:if value="${show_visual}"><visual>''')
# Scanner visuals are rotationally symmetric except the bracket: exporter below
# writes them in the outward-yaw link frame.
edit(desc/'setup.py', '# world files', '''(os.path.join("share", package_name, "meshes", "mecanum"), glob(os.path.join("meshes", "mecanum", "*"))),
        # world files''')

controllers=yaml.safe_load((mybot/'config/ros2_controllers.yaml').read_text())
cm=controllers['controller_manager']['ros__parameters']
cm['arm_yaw_controller']={'type':'joint_trajectory_controller/JointTrajectoryController'}
cm['mecanum_wheel_controller']={'type':'velocity_controllers/JointGroupVelocityController'}
controllers['arm_yaw_controller']={'ros__parameters':{'joints':['arm_yaw_joint'],'command_interfaces':['position'],'state_interfaces':['position','velocity']}}
controllers['mecanum_wheel_controller']={'ros__parameters':{'joints':[p+'_wheel_joint' for p in ['front_left','front_right','rear_left','rear_right']]}}
save(mybot/'config/ros2_controllers_mecanum.yaml',yaml.safe_dump(controllers,sort_keys=False))

launch=mybot/'launch/gazebo_world.launch.py'
edit(launch,'urdf_name = "originbot_with_rgbd_gazebo_arm.xacro"','mecanum = os.environ.get("AIC_ROBOT_MODEL", "legacy") == "mecanum"\n    urdf_name = "originbot_mecanum_gazebo.xacro" if mecanum else "originbot_with_rgbd_gazebo_arm.xacro"')
edit(launch,'    ld.add_action(spawn_entity_cmd)','''    ld.add_action(spawn_entity_cmd)
    if mecanum:
        ld.add_action(Node(package='bot_navigation', executable='mecanum_io.py', parameters=[{'use_sim_time': True}], output='screen'))
        ld.add_action(RegisterEventHandler(OnProcessExit(target_action=load_joint_state_controller_gripper,
            on_exit=[Node(package='controller_manager', executable='spawner', arguments=['arm_yaw_controller','mecanum_wheel_controller'], output='screen')])) )''')

# MoveIt must see the same model as Gazebo, rather than the old six-link base.
moveit=mybot/'launch/my_moveit_rviz.launch.py'
edit(moveit,'from moveit_configs_utils import MoveItConfigsBuilder','import os\nfrom moveit_configs_utils import MoveItConfigsBuilder')
edit(moveit,'    moveit_config = MoveItConfigsBuilder("six_arm", package_name="mybot").to_moveit_configs()', '''    builder = MoveItConfigsBuilder("six_arm", package_name="mybot")
    if os.environ.get("AIC_ROBOT_MODEL", "legacy") == "mecanum":
        from ament_index_python.packages import get_package_share_directory
        builder.robot_description(file_path=os.path.join(get_package_share_directory('mybot_description'), 'urdf', 'originbot_mecanum_gazebo.xacro'))
        builder.robot_description_semantic(file_path='config/six_arm_mecanum.srdf')
        builder.trajectory_execution(file_path='config/moveit_controllers_mecanum.yaml')
    moveit_config = builder.to_moveit_configs()''')
import xml.etree.ElementTree as ET
srdf=ET.parse(mybot/'config/six_arm.srdf')
for joint in srdf.getroot().findall('virtual_joint'): srdf.getroot().remove(joint)
group=ET.SubElement(srdf.getroot(),'group',{'name':'arm_yaw'})
ET.SubElement(group,'joint',{'name':'arm_yaw_joint'})
state=ET.SubElement(srdf.getroot(),'group_state',{'name':'forward','group':'arm_yaw'})
ET.SubElement(state,'joint',{'name':'arm_yaw_joint','value':'0'})
ET.SubElement(srdf.getroot(),'disable_collisions',{'link1':'base_link','link2':'arm_yaw_link','reason':'Adjacent'})
ET.SubElement(srdf.getroot(),'disable_collisions',{'link1':'arm_yaw_link','link2':'arm_base_link','reason':'Adjacent'})
srdf.write(mybot/'config/six_arm_mecanum.srdf',encoding='utf-8',xml_declaration=True)
mc=yaml.safe_load((mybot/'config/moveit_controllers.yaml').read_text())
manager=mc['moveit_simple_controller_manager']
manager['controller_names'].append('arm_yaw_controller')
manager['arm_yaw_controller']={'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':['arm_yaw_joint']}
save(mybot/'config/moveit_controllers_mecanum.yaml',yaml.safe_dump(mc,sort_keys=False))

for suffix in ['', '_odom']:
    config=yaml.safe_load((nav/f'param/originbot_nav2{suffix}.yaml').read_text())
    controller=config['controller_server']['ros__parameters']
    controller['FollowPath']={
        'plugin':'dwb_core::DWBLocalPlanner','debug_trajectory_details':False,
        'min_vel_x':-.7,'max_vel_x':.7,'min_vel_y':-.7,'max_vel_y':.7,'max_vel_theta':1.5,
        'min_speed_xy':0.,'max_speed_xy':.7,'min_speed_theta':0.,
        'acc_lim_x':.35,'acc_lim_y':.35,'acc_lim_theta':2.4,
        'decel_lim_x':-2.5,'decel_lim_y':-2.5,'decel_lim_theta':-3.2,
        'vx_samples':11,'vy_samples':11,'vtheta_samples':9,'sim_time':1.5,
        'linear_granularity':.05,'angular_granularity':.025,'transform_tolerance':.5,
        'xy_goal_tolerance':.10,'trans_stopped_velocity':.03,
        'short_circuit_trajectory_evaluation':True,'stateful':True,
        'critics':['RotateToGoal','Oscillation','Twirling','ObstacleFootprint','GoalDist','PathDist'],
        'Twirling.scale':10.,
        'ObstacleFootprint.scale':.02,'PathDist.scale':24.,'GoalDist.scale':24.,
        'RotateToGoal.scale':32.,'RotateToGoal.slowing_factor':5.,'RotateToGoal.lookahead_time':-1.
    }
    for scope in ['local_costmap','global_costmap']:
        params=config[scope][scope]['ros__parameters']
        params['footprint']='[[0.34, 0.305], [0.34, -0.305], [-0.27, -0.305], [-0.27, 0.305]]'
        layer=params['obstacle_layer']; scan=layer.pop('scan'); layer['observation_sources']='left right'
        for side in ['left','right']: layer[side]=dict(scan,topic='/scan_'+side)
    smoother=config['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.7,.7,1.5],min_velocity=[-.7,-.7,-1.5],max_accel=[.35,.35,2.4],max_decel=[-2.5,-2.5,-3.2])
    config['amcl']['ros__parameters']['robot_model_type']='nav2_amcl::OmniMotionModel'
    save(nav/f'param/originbot_nav2{suffix}_mecanum.yaml',yaml.safe_dump(config,sort_keys=False))
config=yaml.safe_load((nav/'param/collision_monitor.yaml').read_text())
params=config['collision_monitor']['ros__parameters']; scan=params.pop('scan'); params['observation_sources']=['left','right']
for side in ['left','right']: params[side]=dict(scan,topic='/scan_'+side)
save(nav/'param/collision_monitor_mecanum.yaml',yaml.safe_dump(config,sort_keys=False))
navlaunch=nav/'launch/nav_bringup_gazebo.launch.py'
edit(navlaunch,"    nav2_param_path = LaunchConfiguration('params_file'", "    if os.environ.get('AIC_ROBOT_MODEL', 'legacy') == 'mecanum':\n        default_params = default_params.replace('.yaml', '_mecanum.yaml')\n    nav2_param_path = LaunchConfiguration('params_file'")
edit(navlaunch,"'param', 'collision_monitor.yaml'", "'param', 'collision_monitor_mecanum.yaml' if os.environ.get('AIC_ROBOT_MODEL', 'legacy') == 'mecanum' else 'collision_monitor.yaml'")
edit(nav/'CMakeLists.txt','PROGRAMS scripts/bright_costmap.py scripts/scan_velocity_guard.py','PROGRAMS scripts/bright_costmap.py scripts/scan_velocity_guard.py scripts/mecanum_io.py')
# Clamp vector magnitude so diagonals do not get a free sqrt(2) speed increase.
guard=nav/'scripts/scan_velocity_guard.py'
edit(guard,'        self.command_received = time.monotonic()\n        self.pub.publish', '''        self.command_received = time.monotonic()
        speed = math.hypot(msg.linear.x, msg.linear.y)
        if speed > 0.7:
            msg.linear.x *= 0.7/speed
            msg.linear.y *= 0.7/speed
        self.pub.publish''')
print('MECANUM_CONFIG_GENERATED')
