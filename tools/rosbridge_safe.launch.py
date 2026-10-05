"""Keep websocket publishing responsive when ROS graph services fail."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', str(Path(__file__).with_name('rosbridge_with_assets.py')),
                            '--port', '9090', '--max_message_size', '16000000',
                            '--write_queue_size', '16', '--call_services_in_new_thread', 'true',
                            '--default_call_service_timeout', '5.0',
                            '--send_action_goals_in_new_thread', 'true'],
                       output='screen', respawn=True, respawn_delay=2.0),
        ExecuteProcess(cmd=['python3', str(Path(__file__).with_name('rosapi_safe.py'))],
                       output='screen', respawn=True, respawn_delay=2.0),
    ])
