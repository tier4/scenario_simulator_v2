import launch
import launch_ros.actions
import ament_index_python
import os
from pathlib import Path

package_path = ament_index_python.get_package_share_directory('lexus_rx_450h_description')
urdf_path = package_path / Path('urdf/lexus_rx_450h.urdf')
rviz_cfg_path = package_path / Path('config/lexus_rx_450h.rviz')


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[str(urdf_path)]),
        launch_ros.actions.Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', str(rviz_cfg_path)])
    ])
