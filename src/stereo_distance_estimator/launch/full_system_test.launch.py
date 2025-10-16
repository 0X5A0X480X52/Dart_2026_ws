#!/usr/bin/env python3
"""
完整系统测试启动文件
启动顺序：
1. 双目相机 (mindvision_camera)
2. 立体视觉处理 (stereo_image_proc_wrapper) 
3. 目标检测 (object_detection_openvino)
4. 立体距离估计 (stereo_distance_estimator)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """生成完整系统测试的启动描述"""
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    stereo_config_arg = DeclareLaunchArgument(
        'stereo_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_distance_estimator'),
            'config',
            'stereo_distance_estimator.yaml'
        ]),
        description='Path to stereo distance estimator config file'
    )
    
    # 1. 启动双目相机
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mindvision_camera'),
                'launch',
                'dual_camera_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # 2. 启动立体视觉处理 (延迟2秒，等待相机初始化)
    stereo_proc_launch = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="Launching stereo image processing..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('stereo_image_proc_wrapper'),
                        'launch',
                        'stereo_image_proc.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )
    
    # 3. 启动目标检测 (延迟4秒，等待立体视觉初始化)
    detection_launch = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="Launching object detection..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('object_detection_openvino'),
                        'launch',
                        'object_detection_openvino.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )
    
    # 4. 启动立体距离估计 (延迟6秒，等待所有前置节点就绪)
    distance_estimator_node = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="Launching stereo distance estimator..."),
            Node(
                package='stereo_distance_estimator',
                executable='stereo_distance_estimator_node',
                name='stereo_distance_estimator',
                output='screen',
                parameters=[LaunchConfiguration('stereo_config')],
                arguments=['--ros-args', '--log-level', 'info']
            )
        ]
    )
    
    return LaunchDescription([
        # 声明参数
        use_sim_time_arg,
        stereo_config_arg,
        
        # 启动信息
        LogInfo(msg="=" * 60),
        LogInfo(msg="Starting Full System Test"),
        LogInfo(msg="=" * 60),
        LogInfo(msg="Stage 1: Launching dual cameras..."),
        
        # 按顺序启动各个组件
        camera_launch,
        stereo_proc_launch,
        detection_launch,
        distance_estimator_node,
        
        # 完成信息
        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg="=" * 60),
                LogInfo(msg="All nodes launched successfully!"),
                LogInfo(msg="=" * 60),
            ]
        )
    ])
