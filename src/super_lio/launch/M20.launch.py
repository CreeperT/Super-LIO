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
    config_yaml = os.path.join(pkg_super_lio, 'config', 'M20.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
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
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz_flag)
    )


    # --- base_link -> rslidar static TF -------------------------------------
    # The merged cloud (/rslidar_points) is published by m20_lidar_merge with
    # output_in_base_link:=true, i.e. its points are ALREADY expressed in
    # base_link.  The frame_id is kept as 'rslidar' for continuity, so the
    # transform that makes it resolvable is the identity.
    base_link_to_rslidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_rslidar',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rslidar'],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(super_lio_node)
    ld.add_action(base_link_to_rslidar)
    ld.add_action(rviz2_node)

    return ld
