import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('resize_image_raw')
    
    # Default config file path
    default_config_file = os.path.join(pkg_dir, 'config', 'resize_params.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the YAML configuration file'
    )
    
    use_config_file_arg = DeclareLaunchArgument(
        'use_config_file',
        default_value='true',
        description='Whether to use config file (true) or command line parameters (false)'
    )
    
    # Command line parameter arguments (used when use_config_file=false)
    scale_width_arg = DeclareLaunchArgument(
        'scale_width',
        default_value='0.5',
        description='Scale factor for image width (only used if use_config_file=false)'
    )
    
    scale_height_arg = DeclareLaunchArgument(
        'scale_height',
        default_value='0.5',
        description='Scale factor for image height (only used if use_config_file=false)'
    )
    
    interpolation_arg = DeclareLaunchArgument(
        'interpolation',
        default_value='1',
        description='Interpolation method: 0=NEAREST, 1=LINEAR, 2=CUBIC, 3=AREA (only used if use_config_file=false)'
    )
    
    # Topic remapping arguments
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

    # Node with config file
    resize_node_with_config = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('resized/image', LaunchConfiguration('resized_image_topic')),
            ('resized/camera_info', LaunchConfiguration('resized_camera_info_topic')),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_config_file'))
    )

    # Node with command line parameters
    resize_node_with_params = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node',
        parameters=[{
            'scale_width': LaunchConfiguration('scale_width'),
            'scale_height': LaunchConfiguration('scale_height'),
            'interpolation': LaunchConfiguration('interpolation'),
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('resized/image', LaunchConfiguration('resized_image_topic')),
            ('resized/camera_info', LaunchConfiguration('resized_camera_info_topic')),
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_config_file'))
    )

    return LaunchDescription([
        config_file_arg,
        use_config_file_arg,
        scale_width_arg,
        scale_height_arg,
        interpolation_arg,
        image_topic_arg,
        camera_info_topic_arg,
        resized_image_topic_arg,
        resized_camera_info_topic_arg,
        resize_node_with_config,
        resize_node_with_params
    ])
