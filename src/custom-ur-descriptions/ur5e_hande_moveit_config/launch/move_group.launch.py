from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ## Arguments  
    ## mock_hardware is for gripper sim , currently can only be setup through the xacro file, launch file arg doesn't work

    ur_type = DeclareLaunchArgument('ur_type', default_value='ur5e')
    robot_ip = DeclareLaunchArgument('robot_ip', default_value='192.168.1.101')
    use_fake_hardware = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    description_package = DeclareLaunchArgument('description_package', default_value='ur5e_hande_robot_description')
    description_file = DeclareLaunchArgument('description_file', default_value='ur_with_hande.xacro')
    controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(get_package_share_directory("ur5e_hande_moveit_config"), "config", "ur_hande_controllers.yaml"))
    mock_hardware_arg = DeclareLaunchArgument('mock_hardware',default_value="false")

    xacro_args = {"name": "ur", "ur_type": LaunchConfiguration("ur_type"), "tf_prefix": "", "mock_hardware": LaunchConfiguration("mock_hardware")}

    ## ur_driver 
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": "false",
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "tool_voltage": "24",
            "mock_hardware": LaunchConfiguration("mock_hardware"),
        }.items()
    )


    # Load MoveIt! configuration
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit",package_name="ur5e_hande_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("ur5e_hande_robot_description"), "urdf", "ur_with_hande.xacro"),mappings=xacro_args)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
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
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="view_robot.rviz",
        description="RViz config file"
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur5e_hande_moveit_config"), "rviz", LaunchConfiguration("rviz_config")
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

    # Tool Communication Node
    tool_communication = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="tool_communication_node",
        output="screen",
        parameters=[{"robot_ip": LaunchConfiguration("robot_ip")}]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Hand controller spawner
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "-c", "/controller_manager"],
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
        mock_hardware_arg,

        ## Nodes
        tool_communication,
        ur_control_launch,
        run_move_group_node,
        rviz_node,
        static_tf,
        robot_state_publisher,
        hand_controller_spawner,
    ])

    