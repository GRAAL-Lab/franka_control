from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the hardware arm and optional gripper with system time."""
    driver_configuration = str(Path(get_package_share_directory('fer_driver')) / 'conf/driver.yaml')
    dynamic_configuration = str(Path(get_package_share_directory('fer_ctrl')) / 'conf/dcl_fer.conf')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fer'),
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2'),
        DeclareLaunchArgument('with_gripper', default_value='true'),
        DeclareLaunchArgument('driver_config', default_value=driver_configuration),
        DeclareLaunchArgument('dcl_config', default_value=dynamic_configuration),
        Node(
            package='fer_driver',
            executable='driver_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('driver_config'),
                {'robot_ip': LaunchConfiguration('robot_ip'),
                 'use_sim_time': False,
                 'dynamic_configuration_file': LaunchConfiguration('dcl_config')},
            ],
        ),
        Node(
            package='fer_driver',
            executable='gripper_node',
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('with_gripper')),
            output='screen',
            parameters=[{'robot_ip': LaunchConfiguration('robot_ip'), 'use_sim_time': False}],
        ),
    ])
