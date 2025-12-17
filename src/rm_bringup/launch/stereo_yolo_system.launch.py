#!/usr/bin/env python3
"""
双目YOLO测距系统启动文件
按照指定顺序启动：
1. 双目相机驱动 (Stage 1)
2. 左右双路YOLO检测 (Stage 2)
3. 立体YOLO测距和串口驱动 (Stage 3)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成双目YOLO测距系统启动描述"""
    
    # 声明参数
    camera_params_arg = DeclareLaunchArgument(
        'camera_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('mindvision_camera'),
            'config',
            'dual_camera_params.yaml'
        ]),
        description='双目相机参数文件'
    )
    
    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_xml',
        default_value='',
        description='YOLO模型XML文件路径'
    )
    
    yolo_model_bin_arg = DeclareLaunchArgument(
        'yolo_model_bin',
        default_value='',
        description='YOLO模型BIN文件路径'
    )
    
    stereo_config_arg = DeclareLaunchArgument(
        'stereo_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_yolo_distance'),
            'config',
            'stereo_yolo_distance.yaml'
        ]),
        description='立体测距配置文件'
    )
    
    # ========== Stage 1: 双目相机驱动 ==========
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mindvision_camera'),
                'launch',
                'dual_camera_launch.py'
            ])
        ])
    )
    
    stage1 = TimerAction(
        period=0.0,
        actions=[
            LogInfo(msg="=== Stage 1: 启动双目相机驱动 ==="),
            camera_launch
        ]
    )
    
    # ========== Stage 2: 左右双路YOLO检测 ==========
    
    # 获取YOLO配置目录
    yolo_pkg_dir = get_package_share_directory('object_detection_openvino')
    yolo_config = os.path.join(yolo_pkg_dir, 'config', 'params.yaml')
    
    # 左相机YOLO检测节点
    left_yolo_node = Node(
        package='object_detection_openvino',
        executable='object_detection_openvino_node',
        name='object_detection_left',
        namespace='detector_left',
        output='screen',
        parameters=[
            yolo_config,
            {
                'image_topic': '/camera_left/image_raw',
                'detection_topic': '/detector/left/target2d_array',
                'debug_image_topic': '/detector/left/debug_image',
                'roi_mode': 'center',
                'roi_width': 320,
                'roi_height': 240,
            }
        ],
        remappings=[
            ('image_raw', '/camera_left/image_raw'),
            ('/detector/target2d_array', '/detector/left/target2d_array'),
            ('/detector/debug_image', '/detector/left/debug_image'),
        ]
    )
    
    # 右相机YOLO检测节点
    right_yolo_node = Node(
        package='object_detection_openvino',
        executable='object_detection_openvino_node',
        name='object_detection_right',
        namespace='detector_right',
        output='screen',
        parameters=[
            yolo_config,
            {
                'image_topic': '/camera_right/image_raw',
                'detection_topic': '/detector/right/target2d_array',
                'debug_image_topic': '/detector/right/debug_image',
                'roi_mode': 'center',
                'roi_width': 320,
                'roi_height': 240,
            }
        ],
        remappings=[
            ('image_raw', '/camera_right/image_raw'),
            ('/detector/target2d_array', '/detector/right/target2d_array'),
            ('/detector/debug_image', '/detector/right/debug_image'),
        ]
    )
    
    stage2 = TimerAction(
        period=5.0,  # 等待相机启动
        actions=[
            LogInfo(msg="=== Stage 2: 启动左右双路YOLO检测 ==="),
            left_yolo_node,
            right_yolo_node
        ]
    )
    
    # ========== Stage 3: 立体YOLO测距和串口驱动 ==========
    
    # 立体YOLO测距节点
    stereo_yolo_distance_node = Node(
        package='stereo_yolo_distance',
        executable='stereo_yolo_distance_node',
        name='stereo_yolo_distance',
        output='screen',
        parameters=[LaunchConfiguration('stereo_config')],
    )
    
    # 串口驱动节点
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='serial_driver_node',
        name='serial_driver',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rm_serial_driver'),
                'config',
                'serial_driver_params.yaml'
            ])
        ],
    )
    
    stage3 = TimerAction(
        period=10.0,  # 等待YOLO检测启动
        actions=[
            LogInfo(msg="=== Stage 3: 启动立体测距和串口驱动 ==="),
            stereo_yolo_distance_node,
            serial_driver_node
        ]
    )
    
    return LaunchDescription([
        camera_params_arg,
        yolo_model_path_arg,
        yolo_model_bin_arg,
        stereo_config_arg,
        LogInfo(msg="=== 双目YOLO测距系统启动开始 ==="),
        stage1,
        stage2,
        stage3,
        LogInfo(msg="=== 系统启动配置完成 ==="),
    ])
