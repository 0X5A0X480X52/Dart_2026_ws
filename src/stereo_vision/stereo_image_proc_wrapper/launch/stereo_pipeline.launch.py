from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Complete stereo vision pipeline with camera drivers.
    
    This launch file includes:
    - Camera drivers (to be configured based on your camera setup)
    - Stereo image processing pipeline
    """
    
    dual_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mindvision_camera'),
                'launch',
                'dual_camera_launch.py'
            ])
        ]),
    )
    
    # Launch stereo_image_proc
    stereo_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_image_proc_wrapper'),
                'launch',
                'stereo_image_proc.launch.py'
            ])
        ]),
    )
    
    return LaunchDescription([
        dual_camera_launch,
        stereo_proc_launch,
    ])
