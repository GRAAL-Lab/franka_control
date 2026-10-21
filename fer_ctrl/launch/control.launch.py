from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_directory = Path(get_package_share_directory('fer_ctrl'))
    controller_parameters = str(package_directory / 'conf/controller.yaml')
    kinematic_configuration = str(package_directory / 'conf/kcl_fer.conf')
    dynamic_configuration = str(package_directory / 'conf/dcl_fer.conf')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fer'),
        DeclareLaunchArgument('controller_config', default_value=controller_parameters),
        DeclareLaunchArgument('kcl_config', default_value=kinematic_configuration),
        DeclareLaunchArgument('dcl_config', default_value=dynamic_configuration),
        Node(
            package='fer_ctrl',
            executable='kinematic_control_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('controller_config'),
                {'configuration_file': LaunchConfiguration('kcl_config')},
            ],
        ),
        Node(
            package='fer_ctrl',
            executable='dynamic_control_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('controller_config'),
                {'configuration_file': LaunchConfiguration('dcl_config')},
            ],
        ),
    ])
