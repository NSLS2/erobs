"""Launch beambot servers, the orchestrator, rosbridge and optional Zivid capture.

BEAMBOT_BEAMLINE_CONFIG selects the beamline. Vision and pipettor are opt-in;
mock hardware applies only to the robot.
"""

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from beambot.config_loader import build_pipeline_param_args, moveit_config_package


def generate_launch_description():
    """Generate launch description for beambot servers."""

    # Share MoveIt pipeline settings, including Pilz collision-validation adapters.
    # ROS arguments pass the overrides to MTC's in-process C++ node.
    pipeline_args = ["--ros-args", *build_pipeline_param_args()]

    declare_enable_vision = DeclareLaunchArgument(
        "enable_vision",
        default_value="false",
        description="Launch vision/sample servers and the Zivid camera",
    )

    declare_enable_pipettor = DeclareLaunchArgument(
        "enable_pipettor", default_value="false", description="Enable pipettor server"
    )

    declare_use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock robot hardware; camera and pipettor are controlled separately",
    )

    declare_enable_joystick = DeclareLaunchArgument(
        "enable_joystick",
        default_value="false",
        description="Enable gamepad control when MoveIt starts",
    )

    declare_enable_batching = DeclareLaunchArgument(
        "enable_batching",
        default_value="true",
        description="Enable MTC stage batching (false = each task via action server)",
    )

    declare_orchestrator_log_level = DeclareLaunchArgument(
        "orchestrator_log_level",
        default_value="info",
        description="Orchestrator log level (debug|info|warn|error|fatal)",
    )

    declare_enable_rosbridge = DeclareLaunchArgument(
        "enable_rosbridge",
        default_value="true",
        description="Start rosbridge (port 9090) for MCP clients",
    )

    enable_vision = LaunchConfiguration("enable_vision")
    enable_pipettor = LaunchConfiguration("enable_pipettor")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_joystick = LaunchConfiguration("enable_joystick")
    enable_batching = LaunchConfiguration("enable_batching")
    orchestrator_log_level = LaunchConfiguration("orchestrator_log_level")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")

    # Share MoveIt's IK solver settings under RobotModelLoader's parameter key.
    _kinematics_path = (
        get_package_share_path(moveit_config_package()) / "config" / "kinematics.yaml"
    )
    with _kinematics_path.open() as _f:
        _robot_description_kinematics = yaml.safe_load(_f)

    action_server_parameters = [
        {"use_sim_time": False},
        {"robot_description_kinematics": _robot_description_kinematics},
    ]

    move_to_server = Node(
        package="beambot",
        executable="move_to_server.py",
        name="beambot_moveto_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
    )

    end_effector_server = Node(
        package="beambot",
        executable="end_effector_server.py",
        name="beambot_endeffector_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
    )

    tool_exchange_server = Node(
        package="beambot",
        executable="tool_exchange_server.py",
        name="beambot_toolexchange_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
    )

    # Multi-position marker scanning.
    vision_server = Node(
        package="beambot",
        executable="vision_scan_server.py",
        name="beambot_vision_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
        condition=IfCondition(enable_vision),
    )

    vision_task_server = Node(
        package="beambot",
        executable="vision_task_server.py",
        name="beambot_vision_task_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
        condition=IfCondition(enable_vision),
    )

    sample_server = Node(
        package="beambot",
        executable="sample_pick_place_server.py",
        name="beambot_sample_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
        condition=IfCondition(enable_vision),
    )

    zivid_camera = Node(
        package="zivid_camera",
        executable="zivid_camera",
        name="zivid_camera",
        output="screen",
        parameters=[
            {
                "settings_2d_file_path": PathJoinSubstitution(
                    [FindPackageShare("beambot"), "config", "zivid", "zivid_settings.yml"]
                ),
                "settings_file_path": PathJoinSubstitution(
                    [FindPackageShare("beambot"), "config", "zivid", "scene_capture_noproj.yml"]
                ),
                "frame_id": "zivid_optical_frame",
            }
        ],
        condition=IfCondition(enable_vision),
    )

    pipettor_server = Node(
        package="beambot",
        executable="pipettor_server.py",
        name="beambot_pipettor_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=pipeline_args,
        condition=IfCondition(enable_pipettor),
    )

    # Batching plans MTC stages inside the orchestrator process.
    orchestrator = Node(
        package="beambot",
        executable="orchestrator.py",
        # Keep the node's own name; avoid a process-wide __node remap.
        output="screen",
        parameters=action_server_parameters
        + [
            {"use_mock_hardware": use_mock_hardware},
            {"enable_joystick": enable_joystick},
            {"enable_batching": enable_batching},
        ],
        arguments=pipeline_args
        + [
            "--ros-args",
            "--log-level",
            ["beambot_orchestrator:=", orchestrator_log_level],
        ],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"]
            )
        ),
        condition=IfCondition(enable_rosbridge),
    )

    return LaunchDescription(
        [
            # Launch arguments
            declare_enable_vision,
            declare_enable_pipettor,
            declare_use_mock_hardware,
            declare_enable_joystick,
            declare_enable_batching,
            declare_orchestrator_log_level,
            declare_enable_rosbridge,
            move_to_server,
            end_effector_server,
            tool_exchange_server,
            vision_server,
            vision_task_server,
            sample_server,
            zivid_camera,
            pipettor_server,
            orchestrator,
            rosbridge,
        ]
    )
