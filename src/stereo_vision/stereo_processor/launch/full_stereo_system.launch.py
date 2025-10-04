from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    完整的双目测距系统 Launch 文件
    包含：stereo_processor, object_detection, coordinate_filter, 
          stereo_distance_estimator, distance_filter, serial_driver
    """
    
    # Stereo Processor 配置文件
    stereo_config = PathJoinSubstitution([
        FindPackageShare('stereo_processor'),
        'config',
        'stereo_processor.yaml'
    ])

    # Stereo Processor 节点
    stereo_processor_node = Node(
        package='stereo_processor',
        executable='stereo_processor_node',
        name='stereo_processor',
        output='screen',
        parameters=[stereo_config],
        remappings=[
            # 根据您的相机驱动调整这些映射
            # ('/camera/left/image_raw', '/left_camera/image_raw'),
            # ('/camera/right/image_raw', '/right_camera/image_raw'),
        ]
    )

    # 目标检测节点 (使用校正后的左图)
    # 注意: 这里假设 object_detection_openvino 已经配置好
    # object_detection_node = Node(
    #     package='object_detection_openvino',
    #     executable='object_detection_node',
    #     name='object_detection',
    #     output='screen',
    #     remappings=[
    #         ('/image_raw', '/camera/left/image_rect'),
    #     ]
    # )

    # 坐标过滤节点
    # coordinate_filter_node = Node(
    #     package='coordinate_filter',
    #     executable='coordinate_filter_node',
    #     name='coordinate_filter',
    #     output='screen',
    # )

    # 立体距离估计节点
    # stereo_distance_node = Node(
    #     package='stereo_distance_estimator',
    #     executable='stereo_distance_estimator_node',
    #     name='stereo_distance_estimator',
    #     output='screen',
    # )

    # 距离过滤节点
    # distance_filter_node = Node(
    #     package='distance_filter',
    #     executable='distance_filter_node',
    #     name='distance_filter',
    #     output='screen',
    # )

    # 串口驱动节点
    # serial_driver_node = Node(
    #     package='rm_serial_driver',
    #     executable='rm_serial_driver_node',
    #     name='serial_driver',
    #     output='screen',
    # )

    return LaunchDescription([
        stereo_processor_node,
        # 取消注释以启用其他节点
        # object_detection_node,
        # coordinate_filter_node,
        # stereo_distance_node,
        # distance_filter_node,
        # serial_driver_node,
    ])
