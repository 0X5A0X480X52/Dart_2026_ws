from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    scale_width_arg = DeclareLaunchArgument(
        'scale_width',
        default_value='0.5',
        description='Scale factor for image width'
    )
    
    scale_height_arg = DeclareLaunchArgument(
        'scale_height',
        default_value='0.5',
        description='Scale factor for image height'
    )
    
    interpolation_arg = DeclareLaunchArgument(
        'interpolation',
        default_value='1',
        description='Interpolation method: 0=NEAREST, 1=LINEAR, 2=CUBIC, 3=AREA'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera_left/image_raw',
        description='Input image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_left/camera_info',
        description='Input camera info topic'
    )
    
    resized_image_topic_arg = DeclareLaunchArgument(
        'resized_image_topic',
        default_value='/resize/image_raw',
        description='Output resized image topic'
    )
    
    resized_camera_info_topic_arg = DeclareLaunchArgument(
        'resized_camera_info_topic',
        default_value='/resize/camera_info',
        description='Output resized camera info topic'
    )

    # Create resize node
    resize_node = Node(
        package='resize_image_raw',
        executable='resize_node_exe',
        name='resize_node',
        parameters=[{
            'scale_width': LaunchConfiguration('scale_width'),
            'scale_height': LaunchConfiguration('scale_height'),
            'interpolation': LaunchConfiguration('interpolation'),
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('resized/image', LaunchConfiguration('resized_image_topic')),
            ('resized/camera_info', LaunchConfiguration('resized_camera_info_topic')),
        ],
        output='screen'
    )

    return LaunchDescription([
        scale_width_arg,
        scale_height_arg,
        interpolation_arg,
        image_topic_arg,
        camera_info_topic_arg,
        resized_image_topic_arg,
        resized_camera_info_topic_arg,
        resize_node
    ])
