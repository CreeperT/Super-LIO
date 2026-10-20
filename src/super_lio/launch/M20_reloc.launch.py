"""M20 relocation launch — byte-for-byte the structure of launch/M20.launch.py,
with executable switched to relocation_node (the only executable that reads the
lio.relocation block of M20.yaml).  Params file, rviz config and the static TF
are exactly M20.launch.py's; nothing else is changed."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'M20.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'relocation.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Whether to start RVIZ2')
    rviz_flag = LaunchConfiguration('rviz')

    relocation_node = Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[config_yaml, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='relocation_rviz',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz_flag)
    )

    base_link_to_rslidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_rslidar',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rslidar'],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(relocation_node)
    ld.add_action(base_link_to_rslidar)
    ld.add_action(rviz2_node)
    return ld
