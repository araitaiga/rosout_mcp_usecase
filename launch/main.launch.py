#!/usr/bin/env python3

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosout_mcp_usecase',
            executable='trigger_processor',
            name='trigger_processor',
            output='screen'
        ),
        Node(
            package='rosout_mcp_usecase',
            executable='sub_processor',
            name='sub_processor',
            output='screen'
        )
    ])
