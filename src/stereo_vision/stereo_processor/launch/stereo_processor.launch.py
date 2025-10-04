from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('stereo_processor'),
            'config',
            'stereo_processor.yaml'
        ]),
        description='Path to the stereo processor configuration file'
    )

    # Stereo Processor 节点
    stereo_processor_node = Node(
        package='stereo_processor',
        executable='stereo_processor_node',
        name='stereo_processor',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # 可以根据实际情况调整 topic 映射
            # ('/camera/left/image_raw', '/left_camera/image_raw'),
            # ('/camera/right/image_raw', '/right_camera/image_raw'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        stereo_processor_node
    ])
