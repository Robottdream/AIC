"""Keep websocket publishing responsive when ROS graph services fail."""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    port = int(os.environ.get('AIC_ROSBRIDGE_PORT', '9090'))
    if not 1 <= port <= 65535:
        raise ValueError('AIC_ROSBRIDGE_PORT must be between 1 and 65535')
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', str(Path(__file__).with_name('rosbridge_with_assets.py')),
                            '--port', str(port), '--max_message_size', '16000000',
                            '--write_queue_size', '16', '--call_services_in_new_thread', 'true',
                            '--default_call_service_timeout', '5.0',
                            '--send_action_goals_in_new_thread', 'true'],
                       output='screen', respawn=True, respawn_delay=2.0),
        ExecuteProcess(cmd=['python3', str(Path(__file__).with_name('rosapi_safe.py'))],
                       output='screen', respawn=True, respawn_delay=2.0),
    ])
