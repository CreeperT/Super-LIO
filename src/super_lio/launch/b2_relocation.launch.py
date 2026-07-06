import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'b2.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'relocation.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    super_lio_node = Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': False}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        parameters=[
            {'use_sim_time': False} 
        ],
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=['0', '0', '0', '0', '0', '0', 'imu', 'base_link']
    )

    init_pose_marker = Node(
        package='super_lio',
        executable='initial_pose_3d_marker',
        name='initial_pose_3d_marker',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'init_x': 0.0},
            {'init_y': 0.0},
            {'init_z': 0.2},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(super_lio_node)
    ld.add_action(init_pose_marker)
    ld.add_action(rviz2_node)
    ld.add_action(static_tf)

    return ld