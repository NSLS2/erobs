# Beamline Robotics Framework

This is the central repository for the NSLS-II Robotics division's work on integrating industrial robots and automated sample exchangers into beamline operations. Our goal is to reduce the manual burden on users during sample exchange, letting them focus on what matters most - their research.
We build modular, extensible robotics stacks that can accommodate a growing range of hardware, control systems, and AI-driven pipelines.

---

## Technology Stack

### Core Robotics
- **[ROS 2 Humble](https://docs.ros.org/en/humble/)** — OSRF's current LTS release, providing the backbone for our robot communication, lifecycle management, and tooling.
- **[MoveIt 2](https://moveit.picknik.ai/)** — for motion planning, kinematics, and trajectory execution
- **NVIDIA Ecosystem** — GPU-accelerated perception, Isaac ROS integration for real-time robotics simulation and workloads bridging the gap b/w Real world and simulated world.


### Hardware Integrations

We currently support and are actively developing integrations for:

| Hardware | Type |
|---|---|
| **Universal Robots (UR)** | Collaborative robotic arm |
| **Stäubli** | Industrial robotic arm |
| **EPICS-based sample exchangers** | Automated sample handling |

We plan to expand this list significantly as the division grows.

### Vision & AI Pipelines
- **Vision-based pipelines** — camera-driven perception for sample detection, localization, and error recovery
- **LLM-based pipelines** — natural language interfaces and intelligent decision-making layers for higher-level task orchestration

---

## NSLS-II Control System Integration

At NSLS-II, beamline hardware is controlled through:

- **[EPICS](https://epics-controls.org/)** — the underlying distributed control system for hardware I/O and interacting with instruments at NSLS-II. 
- **[Bluesky](https://blueskyproject.io/)** — a collection of python libraries used for running experiments and analysis. 

We expose robot control through a **Bluesky-compatible API framework** *(currently in development)* that allows users to command sample exchange operations directly from their experiment scripts without needing to interact with low-level robot interfaces manually. This dramatically reduces setup time and operator overhead per beamline visit.

---

## Long-Term Vision

The NSLS-II Robotics Division is committed to staying at the forefront of applied robotics in synchrotron science. Our roadmap includes:

- **Reducing sample exchange time** — streamlining the most time-consuming part of a beamline experiment
- **Expanding robot and gripper support** — modular drivers for new hardware with minimal integration overhead.
- **Autonomous experiment pipelines** — moving toward fully automated, closed-loop sample handling where the robot adapts to experiment feedback in real time.
- **Facility-wide standards** — building reusable components that can be adopted across beamlines at NSLS-II and potentially shared with the broader synchrotron community.

---


## Status

| Component | Status |
|---|---|
| ROS 2 Humble base stack | ✅ Active |
| UR robot integration | ✅ Active |
| Stäubli ros integration | 🔄 In progress |
| EPICS sample exchanger | 🔄 In progress |
| Bluesky API layer |  🔄 WIP |
| Vision pipelines | 🔄 In progress |
| LLM orchestration | 🧪 Experimental |


---

## Repo structure

The majority of the contents in this repository are ROS2 packages with associated continer image manifests.
Each manifest in the [docker](./docker) directory is a container image that can be used to run a specific application in the system.

### Source Contents

- [ros2.repos](./src/ros2.repos): ROS2 workspace file for downloading the required external ROS2 dependencies.
- [end_effectors](./src/end_effectors): End effector drivers and configuration for grippers and vacuum systems.
  - [end_effectors.repos](./src/end_effectors/end_effectors.repos): VCS file for downloading robotiq_hande_driver, robotiq_hande_description, ros2_epick_gripper, and serial packages.
  - [epick_config](./src/end_effectors/epick_config): Site-specific configuration overlay for EPick vacuum gripper.
- [custom-ur-descriptions](./src/custom-ur-descriptions): Source directory for custom UR robot arm descriptions (e.g., attaching grippers or other end effectors).
  - [ur3e_hande_robot_description](./src/custom-ur-descriptions/ur3e_hande_robot_description): ROS2 package for defining the UR3e robot arm with the HandE gripper.
  - [ur3e_hande_moveit_config](./src/custom-ur-descriptions/ur3e_hande_moveit_config): ROS2 package for configuring MoveIt for the UR3e robot arm with the HandE gripper.
  - The UR robot arm description is pulled in through rosdep.
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

## Using Containers to Run the Full Applicaiton Suite

The complete application uses a 1-node-per-container model. The containers are currently orchestrated by bash scripts detailed in the READMEs of each container image. Specifically, the full application is detailed in [erobs-common-img](./docker/erobs-common-img/README.md).

## Running some example applications

In order to run the `ur-example` with Docker, follow this procedure:

1. Create the required images.

  ```bash
  cd docker
  docker build -t ursim:latest ./ursim
  docker build -t ur-driver:latest ./ur-driver
  docker build -t ur-example:latest ./ur-example
  ```

2. Start the UR Simulator. In a new terminal, run

  ```bash
  docker compose up ursim
  ```

3. Open VNC client at `localhost:5900`.
4. Turn on and start the robot.
5. Go to the `Move` tab and click the `Home` button.
6. Press and hold the `Move robot to: New position` button to move the robot into position. Press `Continue`.
7. Verify the joint position is `[0, -90, 0, -90, 0, 0]` degrees.
8. Note: setting initial position is requried for the `ur-example` to start, as specified in the `test_goal_publisher_config.yaml` file in the official Unviersal_Robots_ROS2_Driver repo.

9. Start the ur-driver. In a new terminal, run

  ```bash
  docker compose up urdriver
  ```

Now, go back to the VNC client. In the `Program` tab, start the program.

10. Run the ur-example. In a new terminal, run

  ```bash
  docker compose up urexample
  ```

The in `Program/Graphics` tab, the robot should be moving between four poses every 6 seconds.

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
