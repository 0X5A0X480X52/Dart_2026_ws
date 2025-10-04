from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch stereo_distance_estimator node."""
    
    # Declare launch arguments
    target2d_topic_arg = DeclareLaunchArgument(
        'target2d_topic',
        default_value='/filter/target2d_array',
        description='Input 2D target array topic'
    )
    
    disparity_topic_arg = DeclareLaunchArgument(
        'disparity_topic',
        default_value='/stereo/disparity',
        description='Input disparity image topic'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/stereo/points2',
        description='Input point cloud topic'
    )
    
    target3d_topic_arg = DeclareLaunchArgument(
        'target3d_topic',
        default_value='/stereo/target3d_array_raw',
        description='Output 3D target array topic'
    )
    
    use_pointcloud_arg = DeclareLaunchArgument(
        'use_pointcloud',
        default_value='true',
        description='Use point cloud (true) or disparity map (false) for depth estimation'
    )
    
    max_distance_arg = DeclareLaunchArgument(
        'max_distance',
        default_value='10.0',
        description='Maximum valid distance in meters'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.1',
        description='Minimum valid distance in meters'
    )
    
    # Camera intrinsics (for disparity-based depth estimation)
    fx_arg = DeclareLaunchArgument(
        'fx',
        default_value='600.0',
        description='Focal length in x direction (pixels)'
    )
    
    fy_arg = DeclareLaunchArgument(
        'fy',
        default_value='600.0',
        description='Focal length in y direction (pixels)'
    )
    
    cx_arg = DeclareLaunchArgument(
        'cx',
        default_value='320.0',
        description='Principal point x coordinate (pixels)'
    )
    
    cy_arg = DeclareLaunchArgument(
        'cy',
        default_value='240.0',
        description='Principal point y coordinate (pixels)'
    )
    
    baseline_arg = DeclareLaunchArgument(
        'baseline',
        default_value='0.12',
        description='Stereo baseline distance (meters)'
    )
    
    # Stereo distance estimator node
    stereo_distance_estimator_node = Node(
        package='stereo_distance_estimator',
        executable='stereo_distance_estimator_node',
        name='stereo_distance_estimator',
        output='screen',
        parameters=[{
            'target2d_topic': LaunchConfiguration('target2d_topic'),
            'disparity_topic': LaunchConfiguration('disparity_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'target3d_topic': LaunchConfiguration('target3d_topic'),
            'use_pointcloud': LaunchConfiguration('use_pointcloud'),
            'max_distance': LaunchConfiguration('max_distance'),
            'min_distance': LaunchConfiguration('min_distance'),
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
            'cx': LaunchConfiguration('cx'),
            'cy': LaunchConfiguration('cy'),
            'baseline': LaunchConfiguration('baseline'),
            'queue_size': 10,
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        target2d_topic_arg,
        disparity_topic_arg,
        pointcloud_topic_arg,
        target3d_topic_arg,
        use_pointcloud_arg,
        max_distance_arg,
        min_distance_arg,
        fx_arg,
        fy_arg,
        cx_arg,
        cy_arg,
        baseline_arg,
        stereo_distance_estimator_node,
    ])
