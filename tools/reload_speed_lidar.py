"""Reload only the idle robot, retaining the world, delivered cubes and main state."""
import json
import re
import subprocess
import time
from pathlib import Path
import xacro
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from rclpy.qos import qos_profile_sensor_data

root=Path('/home/polarbear/aic_speed_ladder_20261003')
rclpy.init();n=rclpy.create_node('idle_lidar_model_reload');state={'interrupted':False}
subs=[n.create_subscription(String,t,lambda m:state.update(interrupted=True),qos_profile_sensor_data)
      for t in ('/command','/chat','/manual_nav_target')]
subs.append(n.create_subscription(Int32,'/cur',lambda m:state.update(stage=m.data),10))
subs.append(n.create_subscription(Odometry,'/odom',lambda m:state.update(speed=(m.twist.twist.linear.x**2+m.twist.twist.linear.y**2)**.5),qos_profile_sensor_data))
def call(kind,name,req,timeout=40):
    c=n.create_client(kind,name);assert c.wait_for_service(timeout_sec=timeout),name
    f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
    assert f.done() and f.result(),name
    return f.result()
try:
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
    main=Path('/home/polarbear/ws_aic_mecanum_20261002/log/run/09_main.log').read_text()
    assert '所有优化任务执行完成' in main.splitlines()[-1] and state.get('stage',0)==0 and state['speed']<.03 and not state['interrupted']
    poses={}
    for name in ['six_arm']+[f'{c}_cube_{i}' for c in ('red','blue') for i in range(1,6)]:
        poses[name]=[float(v) for v in subprocess.check_output(['gz','model','-m',name,'-p'],text=True,timeout=8).split()]
    if not (root/'world-before-lidar.json').exists():
        (root/'world-before-lidar.json').write_text(json.dumps(poses,indent=2))
    old=call(GetParameters,'/robot_state_publisher/get_parameters',GetParameters.Request(names=['robot_description'])).values[0].string_value
    if not (root/'robot-before-lidar.urdf').exists():(root/'robot-before-lidar.urdf').write_text(old)
    model=re.sub(r'<!--.*?-->', '', xacro.process_file(str(Path(get_package_share_directory('mybot_description'))/'urdf/originbot_mecanum_gazebo.xacro')).toxml(), flags=re.DOTALL)
    (root/'robot-low-lidar.urdf').write_text(model)
    assert model.count('<samples>811</samples>')==2 and model.count('<update_rate>8</update_rate>')==2
    rclpy.spin_once(n,timeout_sec=.1);assert not state['interrupted'],'Business command arrived'
    deleted=call(DeleteEntity,'/delete_entity',DeleteEntity.Request(name='six_arm'))
    assert deleted.success,deleted.status_message
    response=call(SetParameters,'/robot_state_publisher/set_parameters',SetParameters.Request(parameters=[Parameter(name='robot_description',value=ParameterValue(type=4,string_value=model))]))
    assert all(v.successful for v in response.results)
    req=SpawnEntity.Request(name='six_arm',xml=model,robot_namespace='',reference_frame='world')
    x,y,z,roll,pitch,yaw=poses['six_arm'];req.initial_pose.position.x=x;req.initial_pose.position.y=y;req.initial_pose.position.z=z
    from tf_transformations import quaternion_from_euler
    q=quaternion_from_euler(roll,pitch,yaw)
    req.initial_pose.orientation.x,req.initial_pose.orientation.y,req.initial_pose.orientation.z,req.initial_pose.orientation.w=q
    result=call(SpawnEntity,'/spawn_entity',req);assert result.success,result.status_message
    for controller in ('joint_state_broadcaster','arm_controller','gripper_controller','arm_yaw_controller','mecanum_wheel_controller'):
        subprocess.run(['ros2','control','load_controller','--set-state','active',controller],check=True,timeout=40)
    after={name:[float(v) for v in subprocess.check_output(['gz','model','-m',name,'-p'],text=True,timeout=8).split()] for name in poses if name!='six_arm'}
    (root/'world-after-lidar.json').write_text(json.dumps(after,indent=2))
    assert all(sum((a-b)**2 for a,b in zip(after[name][:2],poses[name][:2]))<.01 for name in after),'Delivered cubes moved'
    print('PASS robot reloaded, 811 rays per 270-degree sensor at 8 Hz; delivered cubes retained',flush=True)
finally:n.destroy_node();rclpy.shutdown()
