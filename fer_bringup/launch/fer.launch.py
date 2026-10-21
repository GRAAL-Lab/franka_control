from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_file(package_name, relative_path):
    return str(Path(get_package_share_directory(package_name)) / relative_path)


def include_launch(package_name, filename, arguments, condition=None):
    launch_file = package_file(package_name, 'launch/' + filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    dynamic_configuration = LaunchConfiguration('dcl_config')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fer'),
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2'),
        DeclareLaunchArgument('with_gripper', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('with_tcp', default_value='false'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='9999'),
        DeclareLaunchArgument('controller_config', default_value=package_file('fer_ctrl', 'conf/controller.yaml')),
        DeclareLaunchArgument('kcl_config', default_value=package_file('fer_ctrl', 'conf/kcl_fer.conf')),
        DeclareLaunchArgument('dcl_config', default_value=package_file('fer_ctrl', 'conf/dcl_fer.conf')),
        DeclareLaunchArgument('driver_config', default_value=package_file('fer_driver', 'conf/driver.yaml')),
        DeclareLaunchArgument('simulator_config', default_value=package_file('fer_sim', 'conf/simulator.yaml')),

        include_launch('fer_ctrl', 'control.launch.py', {
            'namespace': namespace,
            'controller_config': LaunchConfiguration('controller_config'),
            'kcl_config': LaunchConfiguration('kcl_config'),
            'dcl_config': dynamic_configuration,
        }),
        include_launch('fer_sim', 'sim.launch.py', {
            'namespace': namespace,
            'simulator_config': LaunchConfiguration('simulator_config'),
            'dcl_config': dynamic_configuration,
        }, IfCondition(LaunchConfiguration('use_sim'))),
        include_launch('fer_driver', 'driver.launch.py', {
            'namespace': namespace,
            'robot_ip': LaunchConfiguration('robot_ip'),
            'with_gripper': LaunchConfiguration('with_gripper'),
            'driver_config': LaunchConfiguration('driver_config'),
            'dcl_config': dynamic_configuration,
        }, UnlessCondition(LaunchConfiguration('use_sim'))),

        Node(
            package='fer_tcp_bridge',
            executable='tcp_bridge_node',
            namespace=namespace,
            output='screen',
            condition=IfCondition(LaunchConfiguration('with_tcp')),
            parameters=[{
                'host': LaunchConfiguration('tcp_host'),
                'port': ParameterValue(LaunchConfiguration('tcp_port'), value_type=int),
            }],
        ),
        Node(
            package='fer_sim',
            executable='visualizer_node',
            namespace=namespace,
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', package_file('fer_bringup', 'conf/fer.rviz')],
            remappings=[('/fer/visualization/arm', 'visualization/arm')],
        ),
    ])
