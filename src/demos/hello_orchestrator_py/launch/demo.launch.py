"""Launch file for hello_orchestrator_py demo."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the demo.
    Launches:
    1. Print action server
    2. Move action server (with MTC/OMPL params)
    3. Orchestrator action server
    """

    # MTC parameters (needed by move_server)
    mtc_params = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization',
        },
        'robot_description_kinematics': {
            'ur_arm': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.001,
                'kinematics_solver_timeout': 1.0,
                'kinematics_solver_attempts': 10,
            }
        }
    }

    return LaunchDescription([
        # Print action server 
        Node(
            package='hello_orchestrator_py',
            executable='print_server.py',
            name='print_server_py',
            output='screen',
        ),

        # Move action server
        Node(
            package='hello_orchestrator_py',
            executable='move_server.py',
            name='move_server_py',
            output='screen',
            parameters=[mtc_params],
        ),

        # Orchestrator action server 
        Node(
            package='hello_orchestrator_py',
            executable='orchestrator_server.py',
            name='orchestrator_server_py',
            output='screen',
        ),
    ])
