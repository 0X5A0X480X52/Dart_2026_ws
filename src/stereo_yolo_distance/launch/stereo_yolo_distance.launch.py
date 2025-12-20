#!/usr/bin/env python3
"""
Stereo YOLO Distance Estimator Launch File
启动基于双目YOLO检测的立体测距节点
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for stereo YOLO distance estimator"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('stereo_yolo_distance')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_yolo_distance'),
            'config',
            'stereo_yolo_distance.yaml'
        ]),
        description='Path to configuration file'
    )
    
    left_target_topic_arg = DeclareLaunchArgument(
        'left_target_topic',
        default_value='/detector/left/target2d_array',
        description='Left camera detection topic'
    )
    
    right_target_topic_arg = DeclareLaunchArgument(
        'right_target_topic',
        default_value='/detector/right/target2d_array',
        description='Right camera detection topic'
    )
    
    target3d_topic_arg = DeclareLaunchArgument(
        'target3d_topic',
        default_value='/stereo/target3d_array',
        description='Output 3D target topic'
    )

    max_time_diff_arg = DeclareLaunchArgument(
        'max_time_diff',
        default_value='0.2',
        description='Max allowed time difference between left and right detections (seconds)'
    )
    
    # Get launch configuration
    config_file = LaunchConfiguration('config_file')
    left_target_topic = LaunchConfiguration('left_target_topic')
    right_target_topic = LaunchConfiguration('right_target_topic')
    target3d_topic = LaunchConfiguration('target3d_topic')
    max_time_diff = LaunchConfiguration('max_time_diff')
    
    # Create stereo YOLO distance node
    stereo_yolo_distance_node = Node(
        package='stereo_yolo_distance',
        executable='stereo_yolo_distance_node',
        name='stereo_yolo_distance',
        output='screen',
        parameters=[
            config_file,
            {
                'left_target_topic': left_target_topic,
                'right_target_topic': right_target_topic,
                'target3d_topic': target3d_topic,
                'max_time_diff': max_time_diff,
            }
        ],
    )
    
    return LaunchDescription([
        config_file_arg,
        left_target_topic_arg,
        right_target_topic_arg,
        target3d_topic_arg,
        max_time_diff_arg,
        stereo_yolo_distance_node,
    ])
