"""Launch the point-cloud relay, Octomap server and MoveIt scene bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_octomap_path = DeclareLaunchArgument(
        'octomap_path',
        default_value='',
        description='Path to saved .bt octomap file. Empty = start with empty map.'
    )

    pointcloud_relay = Node(
        package='beambot',
        executable='pointcloud_relay.py',
        name='pointcloud_relay',
        parameters=[{
            'input_topic': '/points/xyz',
            'output_topic': '/points/xyz_relayed',
            'enable_downsampling': True,
            'voxel_size': 0.01,  # Match the Octomap resolution (meters).
        }],
        output='screen',
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': 0.01,
            'frame_id': 'base_link',
            'sensor_model.max_range': 2.0,  # Meters.
            'sensor_model.hit': 0.7,
            'sensor_model.miss': 0.4,
            'filter_ground_plane': False,  # Retain the ground and table as obstacles.
            'base_frame_id': 'base_link',
            'latch': False,
            'octomap_path': LaunchConfiguration('octomap_path'),
            'use_height_map': True,
            'publish_free_space': False,
        }],
        remappings=[
            ('cloud_in', '/points/xyz_relayed'),
        ],
        output='screen',
    )

    octomap_to_planning_scene = Node(
        package='beambot',
        executable='octomap_to_planning_scene.py',
        name='octomap_to_planning_scene',
        parameters=[{
            'octomap_topic': '/octomap_binary',
            'planning_scene_topic': '/planning_scene',
            'min_update_interval': 0.5,  # Seconds.
            'log_updates': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_octomap_path,
        pointcloud_relay,
        octomap_server,
        octomap_to_planning_scene,
    ])
