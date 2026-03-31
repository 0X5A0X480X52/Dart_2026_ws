#!/usr/bin/env python3
"""
双目YOLO测距系统启动文件
按照指定顺序启动：
1. 双目相机驱动 (Stage 1)
2. 左右双路YOLO检测 (Stage 2)
3. 立体YOLO测距和串口驱动 (Stage 3)
"""

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
from launch_ros.parameter_descriptions import ParameterValue


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
    
    yolo_params_file_arg = DeclareLaunchArgument(
        'yolo_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('object_detection_openvino'),
            'config',
            'params_int8.yaml'
        ]),
        description='YOLO参数文件路径（建议使用 params_int8.yaml）'
    )

    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_xml',
        default_value='./src/object_detection_openvino/config/best_int8_openvino_model/best_int8.xml',
        description='YOLO模型XML文件路径'
    )
    
    yolo_model_bin_arg = DeclareLaunchArgument(
        'yolo_model_bin',
        default_value='./src/object_detection_openvino/config/best_int8_openvino_model/best_int8.bin',
        description='YOLO模型BIN文件路径'
    )

    yolo_model_output_format_arg = DeclareLaunchArgument(
        'yolo_model_output_format',
        default_value='xywh_conf_5xn',
        description='YOLO输出格式（legacy27 或 xywh_conf_5xn）'
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
    
    # 左相机YOLO检测节点
    left_yolo_node = Node(
        package='object_detection_openvino',
        executable='object_detection_openvino_node',
        name='object_detection_left',
        namespace='detector_left',
        output='screen',
        parameters=[
            LaunchConfiguration('yolo_params_file'),
            {
                # 显式覆盖关键模型参数，避免因参数文件节点名不匹配而回退默认值
                'model_output_format': LaunchConfiguration('yolo_model_output_format'),
                'xml_path': LaunchConfiguration('yolo_model_xml'),
                'bin_path': LaunchConfiguration('yolo_model_bin'),
                'input_width': ParameterValue(640, value_type=int),
                'input_height': ParameterValue(640, value_type=int),
                'score_threshold': ParameterValue(0.5, value_type=float),
                'nms_threshold': ParameterValue(0.4, value_type=float),
                'image_topic': '/camera_left/image_raw',
                'detection_topic': '/detector/left/target2d_array',
                'debug_image_topic': '/detector/left/debug_image',
                'roi_mode': 'center',
                # 'roi_width': 320,
                # 'roi_height': 240,
                # 'center_x': 636.0,
                # 'center_y': -1.0,
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
            LaunchConfiguration('yolo_params_file'),
            {
                # 显式覆盖关键模型参数，避免因参数文件节点名不匹配而回退默认值
                'model_output_format': LaunchConfiguration('yolo_model_output_format'),
                'xml_path': LaunchConfiguration('yolo_model_xml'),
                'bin_path': LaunchConfiguration('yolo_model_bin'),
                'input_width': ParameterValue(640, value_type=int),
                'input_height': ParameterValue(640, value_type=int),
                'score_threshold': ParameterValue(0.5, value_type=float),
                'nms_threshold': ParameterValue(0.4, value_type=float),
                'image_topic': '/camera_right/image_raw',
                'detection_topic': '/detector/right/target2d_array',
                'debug_image_topic': '/detector/right/debug_image',
                'roi_mode': 'center',
                # 'roi_width': 320,
                # 'roi_height': 240,
                # 'center_x': 523.0,
                # 'center_y': -1.0,
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
            # serial_driver_node
        ]
    )
    
    return LaunchDescription([
        camera_params_arg,
        yolo_params_file_arg,
        yolo_model_path_arg,
        yolo_model_bin_arg,
        yolo_model_output_format_arg,
        stereo_config_arg,
        LogInfo(msg="=== 双目YOLO测距系统启动开始 ==="),
        stage1,
        stage2,
        stage3,
        LogInfo(msg="=== 系统启动配置完成 ==="),
    ])
