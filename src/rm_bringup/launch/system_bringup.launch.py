#!/usr/bin/env python3
"""
系统启动文件
按照指定顺序启动各个子系统：
1. 相机驱动 (Stage 1)
2. 目标检测和立体图像处理 (Stage 2, 并行启动)
3. 距离估计 (Stage 3)

支持通过配置文件传递launch参数
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_launch_config(config_file):
    """加载配置文件"""
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    return None


def create_include_launch(package, launch_file, launch_arguments=None):
    """
    创建IncludeLaunchDescription动作
    
    Args:
        package: 包名
        launch_file: launch文件名
        launch_arguments: launch参数字典（可选）
    
    Returns:
        IncludeLaunchDescription对象
    """
    launch_args = []
    if launch_arguments and isinstance(launch_arguments, dict):
        # 将字典转换为元组列表
        launch_args = [(key, str(value)) for key, value in launch_arguments.items()]
        print(f"[INFO] 为 {package}/{launch_file} 传递参数: {launch_args}")
    
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package),
                'launch',
                launch_file
            ])
        ]),
        launch_arguments=launch_args
    )


def generate_launch_description():
    """生成启动描述"""
    
    # 声明参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rm_bringup'),
            'config',
            'launch_config.yaml'
        ]),
        description='配置文件路径'
    )
    
    # 启动延迟参数（秒）
    stage1_delay_arg = DeclareLaunchArgument(
        'stage1_delay',
        default_value='0.0',
        description='Stage 1启动延迟（秒）'
    )
    
    stage2_delay_arg = DeclareLaunchArgument(
        'stage2_delay',
        default_value='5.0',
        description='Stage 2启动延迟，等待Stage 1启动完成（秒）'
    )
    
    stage3_delay_arg = DeclareLaunchArgument(
        'stage3_delay',
        default_value='10.0',
        description='Stage 3启动延迟，等待Stage 2启动完成（秒）'
    )
    
    # 获取配置文件路径
    try:
        rm_bringup_share = get_package_share_directory('rm_bringup')
        default_config_path = os.path.join(rm_bringup_share, 'config', 'launch_config.yaml')
    except:
        # 如果包还未安装，使用相对路径
        default_config_path = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'launch_config.yaml'
        )
    
    # 加载配置
    config = load_launch_config(default_config_path)
    if config is None:
        # 如果配置文件不存在，使用默认配置
        config = {
            'stage1': {
                'camera': {
                    'package': 'ros2_mindvision_camera',
                    'launch_file': 'dual_camera_launch.py',
                    'enabled': True
                }
            },
            'stage2': {
                'object_detection': {
                    'package': 'object_detection_openvino',
                    'launch_file': 'object_detection_openvino.launch.py',
                    'enabled': True
                },
                'stereo_image_proc': {
                    'package': 'stereo_image_proc_wrapper',
                    'launch_file': 'stereo_image_proc.launch.py',
                    'enabled': True
                }
            },
            'stage3': {
                'distance_estimator': {
                    'package': 'stereo_distance_estimator',
                    'launch_file': 'stereo_distance_estimator_config.launch.py',
                    'enabled': True
                }
            }
        }
    
    # 创建launch描述列表
    launch_actions = []
    
    # 添加日志
    launch_actions.append(
        LogInfo(msg="=== 系统启动序列开始 ===")
    )
    
    # ========== Stage 1: 相机驱动 ==========
    if config.get('stage1', {}).get('camera', {}).get('enabled', True):
        camera_config = config['stage1']['camera']
        camera_package = camera_config['package']
        camera_launch = camera_config['launch_file']
        camera_args = camera_config.get('launch_arguments', None)
        
        stage1_camera = create_include_launch(
            camera_package, 
            camera_launch,
            camera_args
        )
        
        launch_actions.append(
            TimerAction(
                period=0.0,
                actions=[
                    LogInfo(msg="=== Stage 1: 启动相机驱动 ==="),
                    stage1_camera
                ]
            )
        )
    
    # ========== Stage 2: 目标检测和立体图像处理（并行启动）==========
    stage2_actions = []
    
    # 目标检测
    if config.get('stage2', {}).get('object_detection', {}).get('enabled', True):
        obj_det_config = config['stage2']['object_detection']
        obj_det_package = obj_det_config['package']
        obj_det_launch = obj_det_config['launch_file']
        obj_det_args = obj_det_config.get('launch_arguments', None)
        
        stage2_object_detection = create_include_launch(
            obj_det_package,
            obj_det_launch,
            obj_det_args
        )
        stage2_actions.append(stage2_object_detection)
    
    # 立体图像处理
    if config.get('stage2', {}).get('stereo_image_proc', {}).get('enabled', True):
        stereo_config = config['stage2']['stereo_image_proc']
        stereo_package = stereo_config['package']
        stereo_launch = stereo_config['launch_file']
        stereo_args = stereo_config.get('launch_arguments', None)
        
        stage2_stereo = create_include_launch(
            stereo_package,
            stereo_launch,
            stereo_args
        )
        stage2_actions.append(stage2_stereo)
    
    # 使用定时器在Stage 1启动后延迟启动Stage 2
    if stage2_actions:
        launch_actions.append(
            TimerAction(
                period=5.0,  # 默认延迟5秒
                actions=[
                    LogInfo(msg="=== Stage 2: 启动目标检测和立体图像处理（并行）==="),
                ] + stage2_actions
            )
        )
    
    # ========== Stage 3: 距离估计 ==========
    if config.get('stage3', {}).get('distance_estimator', {}).get('enabled', True):
        dist_config = config['stage3']['distance_estimator']
        dist_package = dist_config['package']
        dist_launch = dist_config['launch_file']
        dist_args = dist_config.get('launch_arguments', None)
        
        stage3_distance = create_include_launch(
            dist_package,
            dist_launch,
            dist_args
        )
        
        # 使用定时器在Stage 2启动后延迟启动Stage 3
        launch_actions.append(
            TimerAction(
                period=10.0,  # 默认延迟10秒
                actions=[
                    LogInfo(msg="=== Stage 3: 启动距离估计 ==="),
                    stage3_distance
                ]
            )
        )
    
    launch_actions.append(
        LogInfo(msg="=== 系统启动配置完成 ===")
    )
    
    return LaunchDescription([
        config_file_arg,
        stage1_delay_arg,
        stage2_delay_arg,
        stage3_delay_arg,
        *launch_actions
    ])
