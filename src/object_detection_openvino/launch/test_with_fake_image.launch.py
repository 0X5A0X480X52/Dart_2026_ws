#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('object_detection_openvino')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Test image publisher
        Node(
            package='object_detection_openvino',
            executable='test_image_publisher.py',
            name='test_image_publisher',
            output='screen'
        ),
        
        # Object detection node
        Node(
            package='object_detection_openvino',
            executable='object_detection_openvino_node',
            name='object_detection_openvino_node',
            parameters=[params_file],
            remappings=[
                ('image_raw', 'image_raw'),
            ],
            output='screen'
        ),
    ])
