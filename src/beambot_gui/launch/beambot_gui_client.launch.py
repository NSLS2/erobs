#!/usr/bin/env python3
"""Launch the GUI; set BEAMBOT_BEAMLINE_CONFIG to the beamline YAML first."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beambot_gui',
            executable='beambot_gui_client',
            name='beambot_gui_client',
            output='screen',
        ),
    ])
