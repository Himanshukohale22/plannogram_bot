#!/usr/bin/env python3
#
# Authors: Himanshu kohale

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = get_package_share_directory('plan_bot_gazebo')
    launch_file_dir = os.path.join(get_package_share_directory('plan_bot_gazebo'), 'launch')
    rviz_congif_file = os.path.join(pkg_name, 'rviz','fynd_bot.rviz')
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
    )

    static_transform_publisher_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='link1_broadcaster',
    arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_congif_file]
    )

    return LaunchDescription([
        robot_state_publisher_cmd,
        static_transform_publisher_node,
        rviz_node
    ])