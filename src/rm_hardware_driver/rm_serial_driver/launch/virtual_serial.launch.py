# virtual_serial.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rm_serial_driver = get_package_share_directory('rm_serial_driver')
    
    params_file = os.path.join(
        pkg_rm_serial_driver,
        'config',
        'serial_driver_params.yaml'
    )
    
    virtual_serial_node = Node(
        package='rm_serial_driver',
        executable='virtual_serial_node',
        name='virtual_serial_node',
        output='screen',
        parameters=[params_file] if os.path.exists(params_file) else []
    )
    
    return LaunchDescription([
        virtual_serial_node
    ])