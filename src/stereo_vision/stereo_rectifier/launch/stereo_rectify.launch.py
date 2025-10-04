from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch stereo image rectification using image_proc.
    
    This launch file sets up image rectification for left and right cameras.
    It uses ROS2's image_proc package to rectify raw images based on camera calibration.
    """
    
    # Declare launch arguments
    left_camera_name_arg = DeclareLaunchArgument(
        'left_camera_name',
        default_value='camera_left',
        description='Left camera name'
    )
    
    right_camera_name_arg = DeclareLaunchArgument(
        'right_camera_name',
        default_value='camera_right',
        description='Right camera name'
    )
    
    # Get launch configuration
    left_camera_name = LaunchConfiguration('left_camera_name')
    right_camera_name = LaunchConfiguration('right_camera_name')
    
    # Create composable node container for better performance
    rectify_container = ComposableNodeContainer(
        name='stereo_rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Left camera rectification node
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
            # Right camera rectification node
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
        left_camera_name_arg,
        right_camera_name_arg,
        rectify_container,
    ])
