from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ## Arguments  

    ur_type = DeclareLaunchArgument('ur_type', default_value='ur5e')
    robot_ip = DeclareLaunchArgument('robot_ip', default_value='192.168.1.10')
    use_fake_hardware = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    description_package = DeclareLaunchArgument('description_package', default_value='ur_description')
    description_file = DeclareLaunchArgument('description_file', default_value=os.path.join(get_package_share_directory("ur5e_robot_description"), "urdf", "ur_with_zivid_pipettor.xacro"))
    controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(get_package_share_directory("ur_zivid_pipettor_moveit_config"), "config", "ur_pipettor_controllers.yaml"))


    xacro_args = {"name": LaunchConfiguration("ur_type"), "ur_type": LaunchConfiguration("ur_type"), "tf_prefix": "" }

    ## ur_driver
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "launch_rviz": "false",
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "kinematics_params_file": os.path.join(get_package_share_directory("ur5e_robot_description"), "config", "ur5e_calibration.yaml"),
            "tool_voltage": "24",
            "use_tool_communication": "true",
        }.items()
    )


    # Load MoveIt! configuration
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit",package_name="ur_zivid_pipettor_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("ur5e_robot_description"), "urdf", "ur_with_zivid_pipettor.xacro"),mappings=xacro_args)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
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
        ],
    )

    # RViz
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="view_robot_mtc.rviz",
        description="RViz config file"
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur_zivid_pipettor_moveit_config"), "rviz", LaunchConfiguration("rviz_config")
    ])

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

    # Tool Communication Node - launched automatically by ur_control when use_tool_communication=true


    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Pipettor driver node (must start after tool_communication creates /tmp/ttyUR)
    pipette_driver = TimerAction(
        period=3.0,  # Wait 3 seconds for tool_communication to create /tmp/ttyUR
        actions=[
            Node(
                package='pipette_driver',
                executable='pipette_driver_node',
                name='pipette_driver_node',
                output='screen',
                parameters=[
                    {'serial_port': '/tmp/ttyUR'},
                    {'use_fake_hardware': False}  # Using real hardware
                ]
            )
        ]
    )

    # Payload configuration for UR controller
    # Total: 1.630 kg = 0.170 kg (mount) + 1.260 kg (camera + housing) + 0.200 kg (pipettor)
    # CoG: Center of Gravity relative to flange frame [x, y, z] in meters
    set_payload = TimerAction(
        period=5.0,  # Wait 5 seconds for robot driver to start
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/io_and_status_controller/set_payload',
                     'ur_msgs/srv/SetPayload',
                     '{mass: 1.630, center_of_gravity: {x: 0.010, y: -0.010, z: -0.020}}'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ## arguments
        robot_ip,
        ur_type,
        use_fake_hardware,
        description_package,
        description_file,
        controllers_file,
        rviz_arg,


        ## Nodes
        ur_control_launch,  # Launches tool_communication automatically when use_tool_communication=true
        run_move_group_node,
        rviz_node,
        robot_state_publisher,
        pipette_driver,  # Pipettor driver node
        set_payload,  # Set UR payload
    ])
