from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    def include(package, filename, arguments, condition=None):
        source = Path(get_package_share_directory(package)) / 'launch' / filename
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                        launch_arguments=arguments.items(), condition=condition)

    rviz = str(Path(get_package_share_directory('fer_bringup')) / 'conf/fer.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fer'),
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2'),
        DeclareLaunchArgument('with_gripper', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('with_tcp', default_value='false'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='9999'),
        DeclareLaunchArgument('controller_config', default_value=str(
            Path(get_package_share_directory('fer_ctrl')) / 'conf/controller.yaml')),
        DeclareLaunchArgument('driver_config', default_value=str(
            Path(get_package_share_directory('fer_driver')) / 'conf/driver.yaml')),
        DeclareLaunchArgument('simulator_config', default_value=str(
            Path(get_package_share_directory('fer_sim')) / 'conf/simulator.yaml')),
        include('fer_ctrl', 'control.launch.py',
                {'namespace': namespace, 'controller_config': LaunchConfiguration('controller_config')}),
        include('fer_sim', 'sim.launch.py',
                {'namespace': namespace, 'simulator_config': LaunchConfiguration('simulator_config')},
                IfCondition(LaunchConfiguration('use_sim'))),
        include('fer_driver', 'driver.launch.py',
                {'namespace': namespace, 'robot_ip': LaunchConfiguration('robot_ip'),
                 'with_gripper': LaunchConfiguration('with_gripper'),
                 'driver_config': LaunchConfiguration('driver_config')},
                UnlessCondition(LaunchConfiguration('use_sim'))),
        Node(package='fer_tcp_bridge', executable='tcp_bridge_node', namespace=namespace,
             output='screen', condition=IfCondition(LaunchConfiguration('with_tcp')),
             parameters=[{'host': LaunchConfiguration('tcp_host'),
                          'port': ParameterValue(LaunchConfiguration('tcp_port'), value_type=int)}]),
        Node(package='fer_sim', executable='visualizer_node', namespace=namespace,
             output='screen', condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2', namespace=namespace,
             condition=IfCondition(LaunchConfiguration('rviz')), arguments=['-d', rviz],
             remappings=[('/fer/visualization/arm', 'visualization/arm')]),
    ])
