from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Complete stereo vision pipeline: HIK cameras → Image rectification → Stereo processing
    
    This is an all-in-one launch file that combines:
    1. stereo_rectifier: Camera drivers + image rectification
    2. stereo_image_proc_wrapper: Disparity computation + point cloud generation
    
    Output topics:
    - /camera/left/image_raw: Raw left image
    - /camera/right/image_raw: Raw right image
    - /camera/left/image_rect: Rectified left image
    - /camera/right/image_rect: Rectified right image
    - /stereo/disparity: Disparity image
    - /stereo/points2: 3D point cloud
    """
    
    # Declare launch arguments
    left_camera_info_url_arg = DeclareLaunchArgument(
        'left_camera_info_url',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_rectifier'),
            'config',
            'left_camera_info.yaml'
        ]),
        description='Left camera calibration file'
    )
    
    right_camera_info_url_arg = DeclareLaunchArgument(
        'right_camera_info_url',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_rectifier'),
            'config',
            'right_camera_info.yaml'
        ]),
        description='Right camera calibration file'
    )
    
    stereo_config_arg = DeclareLaunchArgument(
        'stereo_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_image_proc_wrapper'),
            'config',
            'stereo_params.yaml'
        ]),
        description='Stereo processing configuration file'
    )
    
    # Get launch configuration
    left_camera_info_url = LaunchConfiguration('left_camera_info_url')
    right_camera_info_url = LaunchConfiguration('right_camera_info_url')
    stereo_config = LaunchConfiguration('stereo_config')
    
    # Launch stereo rectifier (cameras + rectification)
    stereo_rectifier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_rectifier'),
                'launch',
                'hik_stereo_rectify.launch.py'
            ])
        ]),
        launch_arguments={
            'left_camera_info_url': left_camera_info_url,
            'right_camera_info_url': right_camera_info_url,
        }.items()
    )
    
    # Launch stereo image processing (disparity + point cloud)
    stereo_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_image_proc_wrapper'),
                'launch',
                'stereo_image_proc.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': stereo_config,
        }.items()
    )
    
    return LaunchDescription([
        left_camera_info_url_arg,
        right_camera_info_url_arg,
        stereo_config_arg,
        stereo_rectifier,
        stereo_processor,
    ])
