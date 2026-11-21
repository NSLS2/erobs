"""Bring up the UR5e driver, MoveIt, and the selected gripper.

Usage:
    ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=epick
    ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=hande use_mock_hardware:=true

Supported grippers: none, epick, hande, 2fg7, pipettor

robot_state_publisher is not started here: ur_control.launch.py already runs
one, and a second would publish conflicting /tf.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from ament_index_python.packages import get_package_share_path

# Per-gripper launch values (URDF, tool I/O, MoveIt controllers) come from the
# active beamline YAML.
from beambot.config_loader import gripper_launch_config

# Shared by all grippers; gripper controllers are added by their spawners.
_BASE_CONTROLLERS = "ur_base_controllers.yaml"

SUPPORTED_GRIPPERS = ["none", "epick", "hande", "2fg7", "pipettor"]


def launch_setup(context, *args, **kwargs):
    """Build launch actions from resolved arguments.

    Runs in an OpaqueFunction because MoveItConfigsBuilder needs plain strings,
    not substitutions.
    """
    gripper = LaunchConfiguration("gripper").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    config = gripper_launch_config(gripper)
    pkg_share = get_package_share_path("cms_moveit_config")
    desc_share = get_package_share_path("cms_robot_description")

    xacro_args = {
        "name": ur_type,
        "ur_type": ur_type,
        "tf_prefix": tf_prefix,
        "robot_ip": robot_ip,
        # Same calibration as the UR driver, so planning matches the real robot.
        "kinematics_params": str(desc_share / "config" / "ur5e_calibration.yaml"),
    }

    if gripper == "epick":
        xacro_args["cup_profile"] = LaunchConfiguration("cup_profile").perform(context)

    ur_launch_args = {
        "ur_type": ur_type,
        "robot_ip": robot_ip,
        "tf_prefix": tf_prefix,
        "use_mock_hardware": use_mock_hardware,
        "launch_rviz": "false",
        "description_file": str(desc_share / "urdf" / config["urdf_xacro"]),
        "controllers_file": str(pkg_share / "config" / _BASE_CONTROLLERS),
        "kinematics_params_file": str(desc_share / "config" / "ur5e_calibration.yaml"),
        "use_tool_communication": (
            "false" if use_mock_hardware == "true"
            else config["use_tool_communication"]
        ),
        "tool_voltage": config["tool_voltage"],
        # Jazzy loads hardware asynchronously; give spawners time to wait for it.
        "controller_spawner_timeout": "30",
    }
    # RS485 settings for the tool connector.
    ur_launch_args.update(config["tool_comm_params"])

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("ur_robot_driver") / "launch" / "ur_control.launch.py")
        ),
        launch_arguments=ur_launch_args.items(),
    )

    # Explicit paths: per-gripper files are not where MoveItConfigsBuilder looks.
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit", package_name="cms_moveit_config")
        .robot_description(
            file_path=str(desc_share / "urdf" / config["urdf_xacro"]),
            mappings=xacro_args,
        )
        .robot_description_semantic(
            file_path=str(pkg_share / "srdf" / "ur.srdf.xacro"),
            mappings={"gripper": gripper},
        )
        .joint_limits(
            file_path=str(pkg_share / "config" / gripper / "joint_limits.yaml"),
        )
        .trajectory_execution(
            file_path=str(pkg_share / "config" / config["moveit_controllers"]),
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"]
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
            moveit_config.to_dict(),
            move_group_capabilities,
            # Controllers load asynchronously; wait for the first joint_states.
            {"planning_scene_monitor_options.wait_for_initial_state_timeout": 30.0},
        ],
    )

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

    actions = [ur_control_launch]
    if gripper == "epick":
        # The UR driver doesn't forward cup_profile; the xacro reads this instead.
        actions.insert(0, SetEnvironmentVariable(
            "BEAMBOT_EPICK_CUP_PROFILE", xacro_args["cup_profile"],
        ))
    actions.append(run_move_group_node)
    actions.append(rviz_node)

    # Optional gamepad Cartesian teleop via MoveIt Servo.
    joystick_enabled = IfCondition(LaunchConfiguration("enable_joystick"))
    joystick_config = str(pkg_share / "config" / "joystick_teleop.yaml")
    servo_params = {
        "moveit_servo": ParameterBuilder("cms_moveit_config")
        .yaml("config/servo.yaml")
        .to_dict()
    }
    actions.extend([
        Node(
            package="joy",
            executable="game_controller_node",
            name="game_controller_node",
            parameters=[{
                "device_id": LaunchConfiguration("joystick_device_id"),
                "deadzone": 0.12,
                "autorepeat_rate": 50.0,
            }],
            condition=joystick_enabled,
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="translation_teleop",
            parameters=[joystick_config],
            remappings=[("/cmd_vel", "/joystick/servo_node/delta_twist_cmds")],
            condition=joystick_enabled,
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="rotation_teleop",
            parameters=[joystick_config],
            remappings=[("/cmd_vel", "/joystick/servo_node/delta_twist_cmds")],
            condition=joystick_enabled,
        ),
        Node(
            package="cms_moveit_config",
            executable="cms_servo_node",
            name="servo_node",
            namespace="joystick",
            parameters=[
                servo_params,
                # AccelerationLimitedPlugin reads these without the servo namespace.
                {"update_period": 0.01},
                {"planning_group_name": "ur_arm"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
            condition=joystick_enabled,
        ),
        # Switch Servo to twist commands; the call waits until Servo is up.
        ExecuteProcess(
            cmd=[
                FindExecutable(name="ros2"),
                "service", "call",
                "/joystick/servo_node/switch_command_type",
                "moveit_msgs/srv/ServoCommandType",
                "{command_type: 1}",
            ],
            output="screen",
            condition=joystick_enabled,
        ),
    ])

    # Gripper controllers come from config/<gripper>_controllers.yaml via
    # --param-file. The spawner doesn't expand $(var ...), so use literal values.
    if gripper == "epick":
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_gripper_action_controller", "epick_status_publisher_controller",
                       "-c", "/controller_manager",
                       "--param-file", str(pkg_share / "config" / "epick_controllers.yaml")],
        ))
    elif gripper == "hande":
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_action_controller", "-c", "/controller_manager",
                       "--param-file", str(pkg_share / "config" / "hande_controllers.yaml")],
        ))
    elif gripper == "2fg7":
        actions.append(TimerAction(
            period=3.0,  # Wait for tool_communication to create /tmp/ttyUR
            actions=[
                Node(
                    package="onrobot_2fg7_driver",
                    executable="onrobot_2fg7_driver_node",
                    name="onrobot_2fg7_driver",
                    output="screen",
                    parameters=[{
                        "serial_port": "/tmp/ttyUR",
                        "slave_id": 65,
                        "baudrate": 1000000,
                        "use_mock_hardware": use_mock_hardware == "true",
                    }],
                ),
            ],
        ))
    elif gripper == "pipettor":
        actions.append(TimerAction(
            period=3.0,  # Wait for tool_communication to create /tmp/ttyUR
            actions=[
                Node(
                    package="pipette_driver",
                    executable="pipette_driver_node",
                    name="pipette_driver_node",
                    output="screen",
                    parameters=[
                        {"serial_port": "/tmp/ttyUR"},
                        {"use_fake_hardware": use_mock_hardware == "true"},
                    ],
                ),
            ],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gripper", default_value="none",
            choices=SUPPORTED_GRIPPERS,
            description="Gripper type: none, epick, hande, 2fg7, pipettor",
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.101",
            description="Default matches CMS beamline YAML; orchestrator always "
                        "overrides via robot_ip:= from $BEAMBOT_BEAMLINE_CONFIG.",
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
        DeclareLaunchArgument(
            "enable_joystick", default_value="false",
            description="Start 8BitDo/Xbox-compatible gamepad teleoperation",
        ),
        DeclareLaunchArgument(
            "joystick_device_id", default_value="0",
            description="SDL game-controller device ID from joy_enumerate_devices",
        ),
        DeclareLaunchArgument(
            "cup_profile", default_value="3mm_dia_external_air",
            description="ePick suction cup profile from suction_cups.yaml (epick only)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
