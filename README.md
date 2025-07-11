# Extensible Robotic Beamline Scientist

Project repository for building extensible robotic beamline scientists at NSLS-II.

## Contents

The majority of the contents in this repository are ROS2 packages with associated container image manifests.
Each manifest in the [docker](./docker) directory is a container image that can be used to run a specific application in the system.

### Source Contents

- [ros2.repos](./src/ros2.repos): ROS2 workspace file for downloading the required external ROS2 dependencies.
- [robotiq-hande](./src/robotiq-hande): Source directory for the HandE gripper. TO BE DEPRECATED
  - [gripper_service](./src/robotiq-hande/gripper_service): ROS2 package for controlling the HandE gripper. TO BE DEPRECATED
- [custom-ur-descriptions](./src/custom-ur-descriptions): Source directory for custom UR robot arm descriptions (e.g., attaching grippers or other end effectors).
  - [ur3e_hande_robot_description](./src/custom-ur-descriptions/ur3e_hande_robot_description): ROS2 package for defining the UR3e robot arm with the HandE gripper. TO BE DEPRECATED
  - [ur3e_hande_moveit_config](./src/custom-ur-descriptions/ur3e_hande_moveit_config): ROS2 package for configuring MoveIt for the UR3e robot arm with the HandE gripper. TO BE DEPRECATED
- [bluesky_ros](./src/bluesky_ros): Python module for integrating Bluesky and ROS2.
- [aruco_pose](./src/aruco_pose): ROS2 package for detecting ArUco markers and calculating their pose.
- [pdf](./src/pdf): Source directory for PDF beamline specific applications.
  - [pdf_beamtime](./src/pdf/pdf_beamtime): ROS2 package for controlling the UR3e robot arm and HandE gripper at the PDF beamline.
  - [pdf_beamtime_interfaces](./src/pdf/pdf_beamtime_interfaces): ROS2 package for defining the interfaces used in the pdf_beamtime package.
- [cms](./src/cms): Source directory for CMS beamline specific applications. (Currently placeholder)
- [lix](./src/lix): Source directory for LIX beamline specific applications. (Currently placeholder)
- [demos](./src/demos): Source directory for demonstration applications.
  - [hello_moveit](./src/demos/hello_moveit): ROS2 package for demonstrating simple actions using the MoveIt library with the UR3e robot arm.
  - [hello_moveit_interfaces](./src/demos/hello_moveit_interfaces): ROS2 package for defining the interfaces used in the hello_moveit package.


### Docker Contents

We use Podman throughout this work, but have named the container images with Docker in mind.

- [erobs-common-img](./docker/erobs-common-img): Common container image for running the majority of applications herein, including: UR robot driver, gripper service, MoveIt service, and the pdf_beamtime_server (primary `Action` server).
- [bsui](./docker/bsui): Container image for running the Bluesky User Interface with mounts at NSLS-II.
- [azure-kinect](./docker/azure-kinect): Container image for running the Azure Kinect ROS2 driver.
- Other auxiliary container images that are not used in the main application, but are useful for development and testing:
  - [ursim](./docker/ursim): Container image for running a simulated UR3e robot arm with a teach pendant.
  - [ur-driver](./docker/ur-driver): Container image for running the UR3e robot arm ROS2 driver.
  - [ur-moveit](./docker/ur-moveit): Container image for running MoveIt with the UR3e robot arm.
  - [ur-example](./docker/ur-example): Container image for running a simple action with the UR3e robot arm.
  - [erobs-hello-moveit](./docker/erobs-hello-moveit): Container image for running a simple action with the UR3e robot arm.


### Hello Moveit

Demonstrations using a combination of the MoveIt tutorials and some UR specific tools, to show how to make simple actions
that can deploy MoveIt using the MoveGroupInterface.

### Bluesky ROS

Ongoing developments of integrating ROS2 and Bluesky. Currently targeted towards integrating Ophyd Objects as ROS2 Action Clients.

## Using Containers to Run the Full Application Suite

The complete application uses a 1-node-per-container model. The containers are currently orchestrated by bash scripts detailed in the READMEs of each container image. Specifically, the full application is detailed in [erobs-common-img](./docker/erobs-common-img/README.md).

# Local Testing

## Running some example applications

You can get a taste of the current development with move groups by running the setup on your computer locally.

The ur5e_hande_moveit_config package contains ROS2 launch files that can be used to launch Rviz, Robotiq Hande Gripper driver, ur robot driver, and the moveit plugin. You should be able to follow the [README](src/custom-ur-descriptions/ur5e_hande_moveit_config/README.md) and start 
controlling the actual robot. 

## Setup

Install ROS2 Humble on your computer: [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Install the ur driver from Universal robot with 

`sudo apt-get install ros-${ROS_DISTRO}-ur`

Clone this repository and change directory to the project root folder

run `bash setup.sh` To install all dependencies

run `bash build.sh` TO build the packages in this project

Go into each README and you should find instructions on how to use each package


## Notes on VSCode Workspace

VSCode ROS2 Workspace Template Borrowed from @althack.

This template will get you set up using ROS2 with VSCode as your IDE. And help ensure consistent development across the project.

See [how she develops with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace.

ROS2-approved formatters are included in the IDE.

- **c++** uncrustify; config from `ament_uncrustify`
- **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

## Notes on pdf_beamtime and its tests

pdf_beamtime is a work-in-progress package aiming to deploy the UR3e robot arm + HandE gripper at the PDF beamline.
This package depends on pdf_beamtime_interfaces. Follow the link below for information on the package and for the commands to call the servers implemented in the package.

[Link to pdf_beamtime README](./src/pdf_beamtime/README.md)
