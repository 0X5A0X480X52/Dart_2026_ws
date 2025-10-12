from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    """
    Launch stereo_image_proc pipeline for stereo vision processing.
    
    This launch file sets up the complete stereo vision pipeline using ROS2's
    stereo_image_proc package, which includes:
    - Image rectification for left and right cameras
    - Disparity map computation
    - Point cloud generation
    
    Topic names (subscribed and published) can be specified in the YAML
    configuration file under the `ros__parameters` mapping. Those values are
    loaded here as defaults and exposed as launch arguments so they can be
    overridden at launch time.
    """

    # Compute default config path and attempt to read topic defaults from it
    default_config_path = os.path.join(
        get_package_share_directory('stereo_image_proc_wrapper'),
        'config',
        'stereo_params.yaml'
    )

    ros_params = {}
    try:
        with open(default_config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            if isinstance(cfg, dict):
                ros_params = cfg.get('/**', {}).get('ros__parameters', {}) or {}
    except Exception:
        # If the file can't be read, fall back to hardcoded defaults below
        ros_params = {}

    # topic defaults (can be overridden in YAML or via launch arguments)
    left_image_default = ros_params.get('left_image_topic', '/camera/left/image_rect')
    right_image_default = ros_params.get('right_image_topic', '/camera/right/image_rect')
    left_info_default = ros_params.get('left_camera_info_topic', '/camera/left/camera_info')
    right_info_default = ros_params.get('right_camera_info_topic', '/camera/right/camera_info')
    disparity_default = ros_params.get('disparity_topic', '/stereo/disparity')
    points_default = ros_params.get('points_topic', '/stereo/points2')
    
    print("Default config path:", default_config_path)
    print("- Left image topic default:", left_image_default)
    print("- Right image topic default:", right_image_default)
    print("- Left camera info topic default:", left_info_default)
    print("- Right camera info topic default:", right_info_default)
    print("- Disparity topic default:", disparity_default)
    print("- Points topic default:", points_default)

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
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

    # Topic launch arguments (defaults loaded from YAML)
    left_image_arg = DeclareLaunchArgument(
        'left_image_topic', default_value=left_image_default,
        description='Topic name for left rectified image'
    )

    right_image_arg = DeclareLaunchArgument(
        'right_image_topic', default_value=right_image_default,
        description='Topic name for right rectified image'
    )

    left_info_arg = DeclareLaunchArgument(
        'left_camera_info_topic', default_value=left_info_default,
        description='Topic name for left camera_info'
    )

    right_info_arg = DeclareLaunchArgument(
        'right_camera_info_topic', default_value=right_info_default,
        description='Topic name for right camera_info'
    )

    disparity_arg = DeclareLaunchArgument(
        'disparity_topic', default_value=disparity_default,
        description='Topic name for output disparity'
    )

    points_arg = DeclareLaunchArgument(
        'points_topic', default_value=points_default,
        description='Topic name for output point cloud (PointCloud2)'
    )

    # Get launch configuration substitutions
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    use_intra_process = LaunchConfiguration('use_intra_process')

    left_image = LaunchConfiguration('left_image_topic')
    right_image = LaunchConfiguration('right_image_topic')
    left_info = LaunchConfiguration('left_camera_info_topic')
    right_info = LaunchConfiguration('right_camera_info_topic')
    disparity = LaunchConfiguration('disparity_topic')
    points = LaunchConfiguration('points_topic')

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
                    ('left/image_rect', left_image),
                    ('left/camera_info', left_info),
                    ('right/image_rect', right_image),
                    ('right/camera_info', right_info),
                    ('disparity', disparity),
                ],
            ),
            # Point cloud node - generates 3D point cloud from disparity
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                parameters=[config_file],
                remappings=[
                    ('left/camera_info', left_info),
                    ('right/camera_info', right_info),
                    ('left/image_rect_color', left_image),
                    ('disparity', disparity),
                    ('points2', points),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        use_intra_process_arg,
        left_image_arg,
        right_image_arg,
        left_info_arg,
        right_info_arg,
        disparity_arg,
        points_arg,
        stereo_container,
    ])
