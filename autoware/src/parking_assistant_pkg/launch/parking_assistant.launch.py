# File: launch/parking_assistant.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parking_assistant_pkg',
            executable='parking_assistant_node',  # same name as your node script (defined in setup.py)
            output='screen'
        )
    ])
