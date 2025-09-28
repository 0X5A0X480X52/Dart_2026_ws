from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='armor',
            description='Detection mode'
        ),
        DeclareLaunchArgument(
            'input_width',
            default_value='640',
            description='Input image width for inference'
        ),
        DeclareLaunchArgument(
            'input_height',
            default_value='640',
            description='Input image height for inference'
        ),
        DeclareLaunchArgument(
            'score_threshold',
            default_value='0.5',
            description='Confidence score threshold for detections'
        ),
        DeclareLaunchArgument(
            'nms_threshold',
            default_value='0.4',
            description='NMS threshold for post-processing'
        ),
        DeclareLaunchArgument(
            'xml_path',
            default_value='/path/to/model.xml',
            description='Path to OpenVINO model XML file'
        ),
        DeclareLaunchArgument(
            'bin_path',
            default_value='/path/to/model.bin',
            description='Path to OpenVINO model BIN file'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='CPU',
            description='OpenVINO inference device (CPU/GPU/etc.)'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='image_raw',
            description='Input image topic name'
        ),
        
        Node(
            package='object_detection_openvino',
            executable='object_detection_openvino_node',
            name='object_detection_openvino_node',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'input_width': LaunchConfiguration('input_width'),
                'input_height': LaunchConfiguration('input_height'),
                'score_threshold': LaunchConfiguration('score_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'xml_path': LaunchConfiguration('xml_path'),
                'bin_path': LaunchConfiguration('bin_path'),
                'device': LaunchConfiguration('device'),
            }],
            remappings=[
                ('image_raw', LaunchConfiguration('image_topic')),
            ],
            output='screen'
        )
    ])