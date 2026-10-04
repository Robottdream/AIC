import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
from launch.event_handlers import OnProcessExit
 
import xacro
 
import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)
 
def generate_launch_description():
    robot_name_in_model = 'six_arm'
    package_name = 'mybot_description'
    mecanum = os.environ.get("AIC_ROBOT_MODEL", "legacy") == "mecanum"
    urdf_name = "originbot_mecanum_gazebo.xacro" if mecanum else "originbot_with_rgbd_gazebo_arm.xacro"
    world_file_path = 'room.world'
    #world_file_path = 'cloister.world'
 
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    world_file_path = os.path.join(pkg_share, f'worlds/{world_file_path}')
 
    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gzserver', '--verbose',
             world_file_path,
             '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')
 
 
    # A GUI failure must not terminate the simulation server.
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'], output='screen',
        condition=IfCondition(LaunchConfiguration('gui')))

    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'robot_description': remove_comments(doc.toxml())} # type: ignore
 
    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题.
    # 这些节点、话题的名称可不可以自定义？
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )
 
    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description', '-timeout', '180'], output='screen')
 
 
    # # Launch the robot, 这个是通过传递文件路径来在gazebo里生成模型.此时要求urdf文件里面没有xacro的语句
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')
    
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     arguments=[urdf_model_path],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
 
    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点
    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
 
    # 路径执行控制器，也就是那个action
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )
    load_joint_state_controller_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    # 监听 load_joint_trajectory_controller，当其退出（完全启动）时，启动load_joint_state_controller_gripper
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_joint_state_controller_gripper],
            )
    )
    
    ld = LaunchDescription()
    # WSL can resolve the host name to its DNS proxy address after a restart.
    # Use loopback for this local simulation, while allowing an explicit override.
    ld.add_action(SetEnvironmentVariable('GAZEBO_IP', os.environ.get('GAZEBO_IP', '127.0.0.1')))
    # Gazebo converts package:// mesh URIs to model://; include the ROS share
    # parent so both server and GUI can resolve the description package.
    ld.add_action(SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', os.path.dirname(pkg_share) + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo client'))
 
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(close_evt3)
 
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_client)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)
    if mecanum:
        ld.add_action(Node(package='bot_navigation', executable='mecanum_io.py', parameters=[{'use_sim_time': True}], output='screen'))
        ld.add_action(RegisterEventHandler(OnProcessExit(target_action=load_joint_state_controller_gripper,
            on_exit=[Node(package='controller_manager', executable='spawner', arguments=['arm_yaw_controller','mecanum_wheel_controller'], output='screen')])) )
 
    return ld
