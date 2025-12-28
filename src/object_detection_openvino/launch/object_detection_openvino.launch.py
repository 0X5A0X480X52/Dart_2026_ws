import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory('object_detection_openvino')
    
    # Default config files
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    print(f"Using params file: {params_file}")
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=params_file,
            description='Path to the ROS2 parameters file'
        ),
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
            default_value='./src/object_detection_openvino/config/openvino/Katrin.xml',
            description='Path to OpenVINO model XML file'
        ),
        DeclareLaunchArgument(
            'bin_path',
            default_value='./src/object_detection_openvino/config/openvino/Katrin.bin',
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
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Whether to publish debug images'
        ),
        DeclareLaunchArgument(
            'roi_mode',
            default_value='center',
            description='ROI mode: full or center'
        ),
        DeclareLaunchArgument(
            'roi_width',
            default_value='320',
            description='ROI width when roi_mode is center'
        ),
        DeclareLaunchArgument(
            'roi_height',
            default_value='240',
            description='ROI height when roi_mode is center'
        ),
        DeclareLaunchArgument(
            'center_x',
            default_value='-1',
            description='Center X pixel for ROI when roi_mode is center (-1 = image center)'
        ),
        DeclareLaunchArgument(
            'center_y',
            default_value='-1',
            description='Center Y pixel for ROI when roi_mode is center (-1 = image center)'
        ),
        

        Node(
            package='object_detection_openvino',
            executable='object_detection_openvino_node',
            name='object_detection_openvino_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'roi_mode': LaunchConfiguration('roi_mode'),
                    'roi_width': ParameterValue(
                        LaunchConfiguration('roi_width'),
                        value_type=int
                    ),
                    'roi_height': ParameterValue(
                        LaunchConfiguration('roi_height'),
                        value_type=int
                    ),
                    'center_x': ParameterValue(
                        LaunchConfiguration('center_x'),
                        value_type=int
                    ),
                    'center_y': ParameterValue(
                        LaunchConfiguration('center_y'),
                        value_type=int
                    ),
                    'publish_debug_image': ParameterValue(
                        LaunchConfiguration('publish_debug_image'),
                        value_type=bool
                    ),
                }
            ],
            remappings=[
                ('image_raw', LaunchConfiguration('image_topic')),
            ],
            output='screen'
        )
    ])