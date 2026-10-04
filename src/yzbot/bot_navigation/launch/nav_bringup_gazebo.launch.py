#!/usr/bin/python3

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition



def generate_launch_description():
    navigation2_dir = get_package_share_directory('bot_navigation')
    # Scope the Humble asynchronous TF repair to navigation processes only.
    # The package pins the dependency ABI; /opt/ros is never modified.
    tf_fix = os.path.join(get_package_prefix('aic_tf2_fix'), 'lib', 'libaic_tf2_fix.so')
    tf_version = ET.parse(os.path.join(get_package_share_directory('tf2_ros'), 'package.xml')).findtext('version')
    if tf_version != '0.25.23':
        raise RuntimeError('aic_tf2_fix targets tf2_ros 0.25.23; review/rebuild it after ROS upgrades')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_file = "map.yaml"

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(navigation2_dir,f'maps/{map_yaml_file}'))
    odom_map_default = os.environ.get('AIC_ODOM_MAP', 'false').lower()
    odom_map_enabled = odom_map_default in ('1', 'true', 'yes')
    default_params = 'originbot_nav2_odom.yaml' if odom_map_enabled else 'originbot_nav2.yaml'
    mecanum = os.environ.get('AIC_ROBOT_MODEL', 'legacy') == 'mecanum'
    if mecanum:
        default_params = default_params.replace('.yaml', '_mecanum.yaml')
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(navigation2_dir, 'param', default_params))
    # Use separate executors and DDS participants for Nav2 on this WSL host.
    # Keep composition opt-in for controlled comparisons on other ROS/DDS setups.
    use_composition = LaunchConfiguration('use_composition')
    rviz = LaunchConfiguration('rviz')
    odom_map = LaunchConfiguration('odom_map')
    slam = LaunchConfiguration('slam', default='False')  
    rviz_config_dir = os.path.join(navigation2_dir, 'rviz', 'nav2_default_view.rviz')
    # slam = LaunchConfiguration('slam', default='True')

    return LaunchDescription([
        DeclareLaunchArgument('use_composition', default_value='False',
                              description='Run Nav2 in a shared component container'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch the navigation RViz view'),
        DeclareLaunchArgument('odom_map', default_value='true' if odom_map_enabled else 'false',
                              description='Use static map-to-odom alignment for Gazebo odometry tests'),
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to param file to load'),
        DeclareLaunchArgument("map", default_value=map_yaml_path, description='Full path to map file to load'),

        Node(condition=IfCondition(odom_map), package='nav_simple',
             executable='navigation_tf_relay', name='navigation_tf_relay',
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
        GroupAction(actions=[
            SetEnvironmentVariable('LD_PRELOAD',
                ' '.join(filter(None, [tf_fix, os.environ.get('LD_PRELOAD', '')]))),
            SetRemap(src='/tf', dst='/navigation/tf', condition=IfCondition(odom_map)),
            SetRemap(src='/tf_static', dst='/navigation/tf_static', condition=IfCondition(odom_map)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([navigation2_dir,'/launch','/safe_bringup.launch.py']),
                launch_arguments={
                    'map': map_yaml_path,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_param_path,
                    'use_composition': use_composition,
                    'slam': slam,}.items(),
            ),
            Node(package='bot_navigation', executable='scan_velocity_guard.py',
                 name='scan_velocity_guard', output='screen',
                 parameters=[nav2_param_path, {'use_sim_time': use_sim_time,
                              'predictive_enabled': os.environ.get('AIC_DYNAMIC_GUARD', '1' if mecanum else '0') == '1',
                              'publish_lidar_forecasts': not mecanum}]),
            Node(condition=IfCondition('true' if mecanum else 'false'),
                 package='bot_navigation', executable='known_obstacle_forecaster.py',
                 name='known_obstacle_forecaster', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen',
                 parameters=[os.path.join(navigation2_dir, 'param', 'collision_monitor_mecanum.yaml' if os.environ.get('AIC_ROBOT_MODEL', 'legacy') == 'mecanum' else 'collision_monitor.yaml'),
                             {'use_sim_time': use_sim_time}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_collision', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'node_names': ['collision_monitor']}]),
        ]),
        Node(
            condition=IfCondition(odom_map),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_static',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='bot_navigation',
            executable='bright_costmap.py',
            name='bright_costmap',
            output='screen'),
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # # sleep(5) # 等待一段时间，等待地图加载完成
        # ExecuteProcess(
        #     cmd=['sleep', '5'],
        #     output='screen'
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        #     output='screen'
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        #     output='screen'
        # ),
    ])
