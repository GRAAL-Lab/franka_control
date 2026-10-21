from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('fer_driver')) / 'conf/driver.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fer'),
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2'),
        DeclareLaunchArgument('with_gripper', default_value='true'),
        DeclareLaunchArgument('driver_config', default_value=config),
        Node(package='fer_driver', executable='driver_node',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[LaunchConfiguration('driver_config'),
                         {'robot_ip': LaunchConfiguration('robot_ip')}]),
        Node(package='fer_driver', executable='gripper_node',
             namespace=LaunchConfiguration('namespace'), output='screen',
             condition=IfCondition(LaunchConfiguration('with_gripper')),
             parameters=[{'robot_ip': LaunchConfiguration('robot_ip')}]),
    ])
