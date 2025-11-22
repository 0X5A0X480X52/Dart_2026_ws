#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包目录
    pkg_rm_visualization = get_package_share_directory('rm_visualization')
    
    # 获取参数文件路径
    params_file = os.path.join(
        pkg_rm_visualization,
        'config',
        'visualization_params.yaml'
    )
    
    # 检查参数文件是否存在
    if not os.path.exists(params_file):
        print(f"Warning: Parameters file not found at {params_file}")
        params_file = None
    
    # 创建可视化节点
    visualization_node = Node(
        package='rm_visualization',
        executable='dart_ui',
        name='visualization_node',
        output='screen',
        parameters=[
            params_file if params_file and os.path.exists(params_file) else {},
            {'serial_driver_node_name': 'virtual_serial_node'}
        ]
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    ld.add_action(visualization_node)
    
    return ld