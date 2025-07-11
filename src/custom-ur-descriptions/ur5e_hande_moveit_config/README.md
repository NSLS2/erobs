# ur5e_hande_moveit_config

This package contains the MoveIt configuration files for using a Universal Robots UR5e manipulator equipped with a Robotiq Hand-E gripper. It is intended for use with ROS 2 and MoveIt 2, and provides the necessary configuration to plan and execute motions with the UR5e + Hand-E system in simulation or on real hardware.

## Setup

### Network
Follow this [link](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/network_setup.html) to ensure that your robot's network is configured correctly and has the External Control URCAP installed.

### Gripper setup

Ensure that the gripper is powered on and has power by setting it up in the Installation tab / Tool IO in the teach pendant or opening the ur5e_with_hande.installation (Same configuration as follows):

Robotiq Hande Tool IO configuration: 

-Tool Output Voltage 24 \
-Communication Interface selected \
-Baud Rate 115200\
-Parity None\
-Stop Bits ONe\
-RX Idle Chars 1.5\
-TX Idle Chars 3.5\
-Standard Output Sinking NPN for both

## Launch 

Perform the Setup steps above before attempting to launch

`ros2 launch ur5e_hande_moveit_config move_group.launch.py robot_ip:=192.168.1.101`

This command will launch with the default parameters specified in the `move_group.launch.py` file. You can customize your own launch settings, using like how in the command we specify a custom ip.

## Examples

Once the move group has been launched, make sure the external control ur cap is added to the program and play the robot.

You can select the ur_arm or the end effector group in the motionplanning plugin and move the robot, then press plan and execute the plan.