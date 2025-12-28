#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, EmitEvent
from launch.events import Shutdown
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
        
        # Object detection node (default params)
        Node(
            package='object_detection_openvino',
            executable='object_detection_openvino_node',
            name='object_detection_openvino_node',
            parameters=[
                params_file,
                {
                    'image_topic': '/image_raw'
                }
            ],
            remappings=[
                ('image_raw', 'image_raw'),
            ],
            output='screen'
        ),

        # Object detection node (custom center test)
        Node(
            package='object_detection_openvino',
            executable='object_detection_openvino_node',
            name='object_detection_openvino_node_custom',
            parameters=[
                params_file,
                {
                    'center_x': 100,
                    'center_y': 150,
                    'detection_topic': '/detector/custom',
                    'debug_image_topic': '/detector/custom_debug',
                    'image_topic': '/image_raw'
                }
            ],
            remappings=[
                ('image_raw', 'image_raw'),
            ],
            output='screen'
        ),

        # ROI test nodes
        Node(
            package='object_detection_openvino',
            executable='test_roi_functionality.py',
            name='roi_test_default',
            parameters=[
                {
                    'roi_mode': 'center',
                    'roi_width': 320,
                    'roi_height': 240,
                    'center_x': -1,
                    'center_y': -1,
                    'input_width': 640,
                    'input_height': 384
                }
            ],
            output='screen'
        ),
        Node(
            package='object_detection_openvino',
            executable='test_roi_functionality.py',
            name='roi_test_custom',
            parameters=[
                {
                    'roi_mode': 'center',
                    'roi_width': 320,
                    'roi_height': 240,
                    'center_x': 100,
                    'center_y': 150,
                    'input_width': 640,
                    'input_height': 384
                }
            ],
            output='screen'
        ),
        # Auto-shutdown after a short period to let tests complete
        TimerAction(
            period=5.0,
            actions=[
                EmitEvent(event=Shutdown())
            ]
        ),
    ])
