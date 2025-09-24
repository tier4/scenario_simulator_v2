from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('trajectory_status_demo')

    # Path to config file
    config_file = os.path.join(package_dir, 'config', 'demo_config.yaml')

    return LaunchDescription([
        Node(
            package='trajectory_status_demo',
            executable='trajectory_status_demo_node',
            name='trajectory_status_demo_node',
            parameters=[config_file],
            output='screen',
            emulate_tty=True,
        )
    ])