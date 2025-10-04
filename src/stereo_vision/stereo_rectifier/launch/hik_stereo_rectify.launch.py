from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Complete stereo camera pipeline with HIK cameras and image rectification.
    
    This launch file includes:
    - Left and right HIK camera drivers
    - Image rectification for both cameras using image_proc
    """
    
    # Declare launch arguments
    left_camera_sn_arg = DeclareLaunchArgument(
        'left_camera_sn',
        default_value='',
        description='Left camera serial number (empty for auto-detect)'
    )
    
    right_camera_sn_arg = DeclareLaunchArgument(
        'right_camera_sn',
        default_value='',
        description='Right camera serial number (empty for auto-detect)'
    )
    
    left_camera_info_url_arg = DeclareLaunchArgument(
        'left_camera_info_url',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_rectifier'),
            'config',
            'left_camera_info.yaml'
        ]),
        description='Left camera calibration file URL'
    )
    
    right_camera_info_url_arg = DeclareLaunchArgument(
        'right_camera_info_url',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_rectifier'),
            'config',
            'right_camera_info.yaml'
        ]),
        description='Right camera calibration file URL'
    )
    
    # Get launch configuration
    left_camera_sn = LaunchConfiguration('left_camera_sn')
    right_camera_sn = LaunchConfiguration('right_camera_sn')
    left_camera_info_url = LaunchConfiguration('left_camera_info_url')
    right_camera_info_url = LaunchConfiguration('right_camera_info_url')
    
    # Create composable node container
    stereo_container = ComposableNodeContainer(
        name='stereo_hik_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Left HIK camera
            ComposableNode(
                package='ros2_hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='left_camera_node',
                parameters=[{
                    'camera_name': 'camera_left',
                    'camera_info_url': left_camera_info_url,
                    'frame_id': 'camera_left_optical_frame',
                    'serial_number': left_camera_sn,
                }],
                remappings=[
                    ('image_raw', '/camera/left/image_raw'),
                    ('camera_info', '/camera/left/camera_info'),
                ],
            ),
            # Right HIK camera
            ComposableNode(
                package='ros2_hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='right_camera_node',
                parameters=[{
                    'camera_name': 'camera_right',
                    'camera_info_url': right_camera_info_url,
                    'frame_id': 'camera_right_optical_frame',
                    'serial_number': right_camera_sn,
                }],
                remappings=[
                    ('image_raw', '/camera/right/image_raw'),
                    ('camera_info', '/camera/right/camera_info'),
                ],
            ),
            # Left image rectification
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify_node',
                remappings=[
                    ('image', '/camera/left/image_raw'),
                    ('camera_info', '/camera/left/camera_info'),
                    ('image_rect', '/camera/left/image_rect'),
                    ('image_rect_color', '/camera/left/image_rect_color'),
                ],
            ),
            # Right image rectification
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify_node',
                remappings=[
                    ('image', '/camera/right/image_raw'),
                    ('camera_info', '/camera/right/camera_info'),
                    ('image_rect', '/camera/right/image_rect'),
                    ('image_rect_color', '/camera/right/image_rect_color'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        left_camera_sn_arg,
        right_camera_sn_arg,
        left_camera_info_url_arg,
        right_camera_info_url_arg,
        stereo_container,
    ])
