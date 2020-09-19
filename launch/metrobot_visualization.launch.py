# Copyright 2020 Metrobot Research

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # Get the launch directory
    description_dir = get_package_share_directory('metrobot_description')

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(description_dir, 'launch'),
                                       '/metrobot_description.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_launch_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(description_dir, 'rviz', 'visualize.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(rviz_launch_cmd)

    return ld
