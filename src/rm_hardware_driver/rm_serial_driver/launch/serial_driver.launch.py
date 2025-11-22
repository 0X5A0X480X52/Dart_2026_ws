#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_rm_serial_driver = get_package_share_directory('rm_serial_driver')
    
    # Get the parameters file path
    params_file = os.path.join(
        pkg_rm_serial_driver,
        'config',
        'serial_driver_params.yaml'
    )
    
    # Check if parameters file exists
    if not os.path.exists(params_file):
        print(f"Warning: Parameters file not found at {params_file}")
        params_file = None
    
    # Create the serial driver node
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='serial_driver_node',
        name='serial_driver_node',
        output='screen',
        parameters=[params_file] if params_file else []
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(serial_driver_node)
    
    return ld