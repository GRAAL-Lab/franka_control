from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the numerical driver with the configured global simulation speed."""
    simulator_configuration = str(Path(get_package_share_directory('fer_sim')) / 'conf/simulator.yaml')
    dynamic_configuration = str(Path(get_package_share_directory('fer_ctrl')) / 'conf/dcl_fer.conf')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fer'),
        DeclareLaunchArgument('isSim', default_value='true'),
        DeclareLaunchArgument('simTimeScale', default_value='1.0'),
        DeclareLaunchArgument('simulator_config', default_value=simulator_configuration),
        DeclareLaunchArgument('dcl_config', default_value=dynamic_configuration),
        Node(
            package='fer_sim',
            executable='simulator_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('simulator_config'),
                {
                    'dynamic_configuration_file': LaunchConfiguration('dcl_config'),
                    'isSim': ParameterValue(LaunchConfiguration('isSim'), value_type=bool),
                    'use_sim_time': ParameterValue(LaunchConfiguration('isSim'), value_type=bool),
                    'simTimeScale': ParameterValue(LaunchConfiguration('simTimeScale'), value_type=float),
                },
            ],
        ),
    ])
