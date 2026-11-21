from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/tmp/ttyUR'),
        DeclareLaunchArgument('slave_id', default_value='65'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),

        Node(
            package='onrobot_2fg7_driver',
            executable='onrobot_2fg7_driver_node',
            name='onrobot_2fg7_driver',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'slave_id': LaunchConfiguration('slave_id'),
                'baudrate': LaunchConfiguration('baudrate'),
                'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            }],
            output='screen',
        ),
    ])
