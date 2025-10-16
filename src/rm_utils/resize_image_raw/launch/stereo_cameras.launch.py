import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """启动双目相机的图像缩放节点（左右各一个节点）"""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('resize_image_raw')
    
    # Declare launch arguments
    scale_width_arg = DeclareLaunchArgument(
        'scale_width',
        default_value='0.5',
        description='Scale factor for image width'
    )
    
    scale_height_arg = DeclareLaunchArgument(
        'scale_height',
        default_value='0.5',
        description='Scale factor for image height'
    )
    
    interpolation_arg = DeclareLaunchArgument(
        'interpolation',
        default_value='3',
        description='Interpolation method: 0=NEAREST, 1=LINEAR, 2=CUBIC, 3=AREA'
    )

    # 左相机节点
    left_camera_node = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node_left',
        namespace='camera_left',
        parameters=[{
            'scale_width': LaunchConfiguration('scale_width'),
            'scale_height': LaunchConfiguration('scale_height'),
            'interpolation': LaunchConfiguration('interpolation'),
            'input_image_topic': '/camera_left/image_raw',
            'input_camera_info_topic': '/camera_left/camera_info',
            'output_image_topic': '/camera_left/resized/image_raw',
            'output_camera_info_topic': '/camera_left/resized/camera_info',
        }],
        output='screen'
    )
    
    # 右相机节点
    right_camera_node = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node_right',
        namespace='camera_right',
        parameters=[{
            'scale_width': LaunchConfiguration('scale_width'),
            'scale_height': LaunchConfiguration('scale_height'),
            'interpolation': LaunchConfiguration('interpolation'),
            'input_image_topic': '/camera_right/image_raw',
            'input_camera_info_topic': '/camera_right/camera_info',
            'output_image_topic': '/camera_right/resized/image_raw',
            'output_camera_info_topic': '/camera_right/resized/camera_info',
        }],
        output='screen'
    )

    return LaunchDescription([
        scale_width_arg,
        scale_height_arg,
        interpolation_arg,
        left_camera_node,
        right_camera_node
    ])
