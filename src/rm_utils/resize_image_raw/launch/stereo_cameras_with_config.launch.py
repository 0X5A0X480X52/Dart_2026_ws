import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """使用配置文件启动双目相机的图像缩放节点"""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('resize_image_raw')
    
    # Config files
    left_config = os.path.join(pkg_dir, 'config', 'camera_left.yaml')
    right_config = os.path.join(pkg_dir, 'config', 'camera_right.yaml')
    
    # 左相机节点
    left_camera_node = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node_left',
        parameters=[left_config],
        output='screen'
    )
    
    # 右相机节点
    right_camera_node = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node_right',
        parameters=[right_config],
        output='screen'
    )

    return LaunchDescription([
        left_camera_node,
        right_camera_node
    ])
