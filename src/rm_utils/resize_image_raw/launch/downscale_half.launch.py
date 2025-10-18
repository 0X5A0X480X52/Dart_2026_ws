import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """使用 downscale_half.yaml 配置文件启动节点"""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('resize_image_raw')
    config_file = os.path.join(pkg_dir, 'config', 'downscale_half.yaml')
    
    # Declare launch arguments for topic remapping
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera_left/image_raw',
        description='Input image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_left/camera_info',
        description='Input camera info topic'
    )
    
    resized_image_topic_arg = DeclareLaunchArgument(
        'resized_image_topic',
        default_value='/resize/image_raw',
        description='Output resized image topic'
    )
    
    resized_camera_info_topic_arg = DeclareLaunchArgument(
        'resized_camera_info_topic',
        default_value='/resize/camera_info',
        description='Output resized camera info topic'
    )

    # Create resize node
    resize_node = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node',
        parameters=[config_file],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('resized/image', LaunchConfiguration('resized_image_topic')),
            ('resized/camera_info', LaunchConfiguration('resized_camera_info_topic')),
        ],
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        resized_image_topic_arg,
        resized_camera_info_topic_arg,
        resize_node
    ])
