from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('p-puma'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='p-puma',
            executable='control_node',
            name='puma_node',
            parameters=[param_file]
        )
    ])