# In ~/ros2_ws/src/parking_system/launch/parking_simulation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. DEFINE PATHS ---
    
    # *** CORRECTED: Hardcoded absolute path to your map directory ***
    # Note: Using an absolute path is okay for personal projects but for shared projects,
    # it's better to place the map inside a ROS 2 package.
    map_path = '$HOME/autoware_auto/autoware/autoware_map/sample-map-planning'

    # Path to the main Autoware launch file
    autoware_launch_file = os.path.join(
        get_package_share_directory('autoware_launch'),
        'launch/autoware.launch.py'
    )

    # --- 2. CONFIGURE AND LAUNCH AUTOWARE ---
    autoware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autoware_launch_file),
        launch_arguments={
            'map_path': map_path,
            'vehicle_model': 'sample_vehicle',
            'sensor_model': 'sample_sensor_kit'
        }.items()
    )

    # --- 3. LAUNCH YOUR PARKING MANAGEMENT NODE ---
    parking_management_node = Node(
        package='parking_system',
        executable='parking_management_node', 
        name='parking_management_node',
        output='screen'
    )

    return LaunchDescription([
        autoware_launch,
        parking_management_node
    ])
