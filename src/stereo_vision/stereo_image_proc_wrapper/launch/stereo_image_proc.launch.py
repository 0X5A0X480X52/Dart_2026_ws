from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch stereo_image_proc pipeline for stereo vision processing.
    
    This launch file sets up the complete stereo vision pipeline using ROS2's
    stereo_image_proc package, which includes:
    - Image rectification for left and right cameras
    - Disparity map computation
    - Point cloud generation
    """
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_image_proc_wrapper'),
            'config',
            'stereo_params.yaml'
        ]),
        description='Path to stereo processing configuration file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    use_intra_process_arg = DeclareLaunchArgument(
        'use_intra_process',
        default_value='true',
        description='Use intra-process communication for better performance'
    )
    
    # Get launch configuration
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    use_intra_process = LaunchConfiguration('use_intra_process')
    
    # Create composable node container for better performance
    stereo_container = ComposableNodeContainer(
        name='stereo_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Disparity node - computes disparity from rectified stereo pair
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                parameters=[config_file],
                remappings=[
                    ('left/image_rect', '/camera/left/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/image_rect', '/camera/right/image_rect'),
                    ('right/camera_info', '/camera/right/camera_info'),
                    ('disparity', '/stereo/disparity'),
                ],
            ),
            # Point cloud node - generates 3D point cloud from disparity
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                parameters=[config_file],
                remappings=[
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/camera_info', '/camera/right/camera_info'),
                    ('left/image_rect_color', '/camera/left/image_rect'),
                    ('disparity', '/stereo/disparity'),
                    ('points2', '/stereo/points2'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        use_intra_process_arg,
        stereo_container,
    ])
