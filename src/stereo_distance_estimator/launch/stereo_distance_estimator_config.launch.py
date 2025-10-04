from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch stereo_distance_estimator node with config file."""
    
    # Get package share directory
    pkg_share = FindPackageShare('stereo_distance_estimator')
    
    # Declare config file argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'stereo_distance_estimator.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    # Stereo distance estimator node
    stereo_distance_estimator_node = Node(
        package='stereo_distance_estimator',
        executable='stereo_distance_estimator_node',
        name='stereo_distance_estimator',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        stereo_distance_estimator_node,
    ])
