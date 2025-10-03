# Extensible Robotic Beamline Scientist

Project repository for building extensible robotic beamline scientists at NSLS-II. ROS 2 Humble with Ubuntu 22.04 LTS

## Contents

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

**⚠️ The `docker/` directory and `scripts/pdf-launch-scripts/` are DEPRECATED. See below for the active deployment workflow.**

#### Active Deployment Workflow

For development and deployment

- **Development**: Use [.devcontainer/Dockerfile](./.devcontainer/Dockerfile) with VSCode devcontainer for local development
- **CI/CD**: Automated builds via [.github/workflows/docker-build.yaml](./.github/workflows/docker-build.yaml)
  - Push a version tag to trigger automatic build and publish to GitHub Container Registry (GHCR)
- **Deployment**: Pull the unified container image from GHCR at `ghcr.io/<your-org>/<your-repo>:latest`
  - All dependencies are pre-installed (no apt install required at deployment)
  - Launch ROS2 nodes using `ros2 launch` commands directly from the container

#### Deprecated Docker Contents (For Reference Only)

The following directories contain legacy container configurations and are **no longer maintained**:

- [docker/](./docker/): Legacy multi-container Dockerfiles - See [docker/DEPRECATED.md](./docker/DEPRECATED.md)
- [scripts/pdf-launch-scripts/](./scripts/pdf-launch-scripts/): Legacy deployment scripts referencing gatekept GHCR images - See [scripts/pdf-launch-scripts/DEPRECATED.md](./scripts/pdf-launch-scripts/DEPRECATED.md)

These can be used for learning purposes but should not be used for active development or deployment.

### Hello Moveit

Demonstrations using a combination of the MoveIt tutorials and some UR specific tools, to show how to make simple actions that can deploy MoveIt using the MoveGroupInterface.

### Bluesky ROS

Ongoing developments of integrating ROS2 and Bluesky. Currently targeted towards integrating Ophyd Objects as ROS2 Action Clients.

## Development Setup

### Local Development with VSCode

1. Install VSCode with the Remote-Containers extension
2. Open this repository in VSCode
3. VSCode will prompt to "Reopen in Container" - accept this
4. The devcontainer will automatically build using `.devcontainer/Dockerfile`
5. Once inside the container, build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### Deployment

For production deployment:

1. **Automated Build**: Push a version tag to trigger GitHub Actions
   ```bash
   git tag v1.0.0
   git push origin v1.0.0
   ```

2. **Pull from GHCR**: On your deployment machine
   ```bash
   podman pull ghcr.io/<your-org>/<your-repo>:latest
   ```

3. **Run containers**: Launch ROS2 nodes with appropriate environment variables
   ```bash
   podman run -it --network host --ipc=host \
     --env ROS_DOMAIN_ID=10 \
     ghcr.io/<your-org>/<your-repo>:latest \
     ros2 launch <package> <launch_file>
   ```

## Running Example Applications (Legacy - For Learning Only)

**⚠️ The following examples use deprecated docker configurations. See above for current deployment workflow.**

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
