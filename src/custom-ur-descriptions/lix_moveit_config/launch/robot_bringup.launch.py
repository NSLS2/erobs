"""MoveIt launch file for UR5e + Hand-E at LiX beamline.

Usage:
    ros2 launch lix_moveit_config robot_bringup.launch.py
    ros2 launch lix_moveit_config robot_bringup.launch.py use_mock_hardware:=true

Single gripper configuration — no tool exchange, no camera.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    pkg_share = get_package_share_path("lix_moveit_config")
    desc_share = get_package_share_path("lix_robot_description")

    urdf_file = "ur_with_hande.xacro"

    xacro_args = {
        "name": ur_type,
        "ur_type": ur_type,
        "tf_prefix": tf_prefix,
        "robot_ip": robot_ip,
        "socat_ip_address": robot_ip,
    }

    # ── UR driver ───────────────────────────────────────────────────────
    ur_launch_args = {
        "ur_type": ur_type,
        "robot_ip": robot_ip,
        "tf_prefix": tf_prefix,
        "use_mock_hardware": use_mock_hardware,
        "launch_rviz": "false",
        "description_package": "ur_description",
        "description_file": str(desc_share / "urdf" / urdf_file),
        # Gripper-agnostic base; gripper controller added by the spawner overlay. #86
        "controllers_file": str(pkg_share / "config" / "ur_base_controllers.yaml"),
        "kinematics_params_file": str(desc_share / "config" / "ur5e_calibration.yaml"),
        "use_tool_communication": "false",
        "tool_voltage": "24",
        "controller_spawner_timeout": "30",
    }

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("ur_robot_driver") / "launch" / "ur_control.launch.py")
        ),
        launch_arguments=ur_launch_args.items(),
    )

    # ── MoveIt config ───────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit", package_name="lix_moveit_config")
        .robot_description(
            file_path=str(desc_share / "urdf" / urdf_file),
            mappings=xacro_args,
        )
        .robot_description_semantic(
            file_path=str(pkg_share / "srdf" / "ur.srdf.xacro"),
        )
        .joint_limits(
            file_path=str(pkg_share / "config" / "hande" / "joint_limits.yaml"),
        )
        .trajectory_execution(
            file_path=str(pkg_share / "config" / "hande" / "moveit_controllers.yaml"),
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.to_dict(),
            move_group_capabilities,
            {"planning_scene_monitor_options.wait_for_initial_state_timeout": 30.0},
        ],
    )

    # ── RViz ────────────────────────────────────────────────────────────
    rviz_config = str(pkg_share / "rviz" / "view_robot_mtc.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ── Payload (Hand-E mass on flange) ─────────────────────────────────
    payload_mass = 0.925  # Hand-E alone (no tool block or camera)
    payload_cog = {"x": 0.0, "y": 0.0, "z": 0.058}

    set_payload = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/io_and_status_controller/set_payload",
                    "ur_msgs/srv/SetPayload",
                    f"{{mass: {payload_mass}, "
                    f"center_of_gravity: {{x: {payload_cog['x']}, y: {payload_cog['y']}, z: {payload_cog['z']}}}}}",
                ],
                output="screen",
            )
        ],
    )

    # ── Gripper controller spawner ──────────────────────────────────────
    # Overlay carries the gripper type+params (spawner won't expand $(var ...),
    # so its joint name is literal; LiX runs tf_prefix=""). #86
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "-c", "/controller_manager",
                   "--param-file", str(pkg_share / "config" / "hande_controllers.yaml")],
    )

    return [
        ur_control_launch,
        run_move_group_node,
        rviz_node,
        set_payload,
        gripper_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.101",
            description="IP address of the UR5e at LiX",
        ),
        DeclareLaunchArgument(
            "ur_type", default_value="ur5e",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
        ),
        DeclareLaunchArgument(
            "tf_prefix", default_value="",
            description="Joint/link name prefix; must match controllers' $(var tf_prefix).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
