"""Manage gripper-specific MoveIt launches, scene loading and UR hardware setup.

URDF/SRDF structure per gripper is checked offline by test_robot_models.py.
"""

import contextlib
import os
import signal
import socket
import subprocess
import threading
import time
import traceback
from pathlib import Path

import yaml
from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler

from beambot.config_loader import (
    arm_joint_names,
    load_beamline_config,
    resolve_beamline_path,
)
from beambot.utils import MODEL_REVISION_PREFIX, VERIFIED_MODEL_DESCRIPTION


class MoveItLifecycleManager:
    """Manage MoveIt startup, reuse and shutdown for each gripper configuration."""

    UR_SECONDARY_PORT = 30002  # URScript secondary interface
    SOCKET_TIMEOUT = 2.0

    def __init__(self, node: Node, grippers: dict, robot_ip: str, callback_group=None,
                 use_mock_hardware: bool = False, enable_joystick: bool = False):
        """Use the caller's ROS node and per-gripper beamline configurations.

        Mock hardware skips physical setup; joystick mode enables Servo gamepad
        control. Blocking waits rely on the caller's executor processing responses.
        """
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._robot_ip = robot_ip
        self._callback_group = callback_group
        self._use_mock_hardware = use_mock_hardware
        self._enable_joystick = enable_joystick

        self._moveit_process: subprocess.Popen | None = None
        # Deterministic: prefix + gripper [+ "__" + cup profile]; set only after
        # the launched model is verified.
        self._model_revision: str = ""
        self._current_voltage: int | None = None

        model_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._model_description_publishers = (
            node.create_publisher(String, VERIFIED_MODEL_DESCRIPTION, model_qos),
            node.create_publisher(
                String, f"{VERIFIED_MODEL_DESCRIPTION}_semantic", model_qos
            ),
        )

        # Read URDF/SRDF from move_group: shared description topics can mix launches
        # during restarts. Republish the verified pair on managed topics for MTC.
        self._move_group_parameters = AsyncParameterClient(
            node, "/move_group", callback_group=callback_group
        )

        # Orchestrator's ePick override; empty uses YAML. (2 epick grippers)
        self.cup_override: str = ""

        self._arm_joint_names = tuple(arm_joint_names())

        # Keep the joint-state subscription persistent to avoid executor teardown races.
        self._joint_positions: dict = {}
        # Program-status subscription is recreated by _restart_external_control.
        self._robot_program_running = threading.Event()
        if not use_mock_hardware:
            self._node.create_subscription(
                JointState, "/joint_states", self._joint_state_cb, 10,
                callback_group=self._callback_group,
            )

    def is_moveit_alive(self) -> bool:
        """Check if the MoveIt subprocess is still running."""
        if self._moveit_process is None:
            return False
        return self._moveit_process.poll() is None

    @property
    def model_revision(self) -> str:
        """Verified model revision, or an empty string when MoveIt is stopped."""
        return self._model_revision if self.is_moveit_alive() else ""

    def get_moveit_exit_info(self) -> str:
        """Describe why MoveIt is down; call after is_moveit_alive() returns False."""
        if self._moveit_process is None:
            return "MoveIt process not started"
        return f"MoveIt process exited with code {self._moveit_process.poll()}"

    def notify_voltage_change(self, voltage: int):
        """Record voltage set by the orchestrator through set_io."""
        self._current_voltage = voltage

    def _call_service(self, srv_type, name: str, request, timeout_sec: float = 5.0):
        """Call a service through a temporary client; return the response or None.

        Client.call blocks until another executor thread delivers the response.
        Logs unavailability and timeouts; exceptions propagate to the caller.
        """
        client = self._node.create_client(
            srv_type, name, callback_group=self._callback_group
        )
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                self._logger.error(f"{name} service not available")
                return None
            response = client.call(request, timeout_sec=timeout_sec)
            if response is None:
                self._logger.error(f"{name} call timed out")
            return response
        finally:
            self._node.destroy_client(client)

    def _joint_state_cb(self, msg):
        """Cache arm joint positions from /joint_states."""
        for name, pos in zip(msg.name, msg.position):
            if name in self._arm_joint_names:
                self._joint_positions[name] = pos

    def current_arm_joints(self):
        """Arm angles in radians, ordered by arm_joint_names(), or None.

        JointState order may differ and include gripper joints. Return None until
        every arm joint is known; mock hardware has no joint-state subscription here.
        None does not force a cache miss: the key then lacks a known start state.
        """
        positions = [self._joint_positions.get(n) for n in self._arm_joint_names]
        if any(p is None for p in positions):
            return None
        return positions

    def launch_moveit_with_gripper(self, gripper: str) -> bool:
        """Reuse matching verified MoveIt, or launch and verify a replacement.

        Reuse requires the same revision (gripper and cup profile). Hardware
        launches retry once for ros2_control startup failures: stale robot TCP
        sockets can kill the hardware process while the launch parent remains alive.
        """
        config = self._grippers[gripper]
        # Only grippers that define a cup profile (ePick) use one.
        cup_profile = (
            (self.cup_override or config["cup_profile"]) if "cup_profile" in config else ""
        )
        revision = MODEL_REVISION_PREFIX + "__".join(filter(None, (gripper, cup_profile)))

        if self.is_moveit_alive() and self._model_revision == revision:
            self._logger.info(f"MoveIt already running for {revision}, reusing")
            return True
        if self._moveit_process:
            self._logger.info(
                f"Restarting MoveIt model: {self._model_revision or 'stopped'} → {revision}"
            )
            self.kill_current_process()

        ok = self._attempt_launch(gripper, cup_profile) and self._publish_verified_model(gripper)
        if not ok and not self._use_mock_hardware:
            self._logger.error("Launch failed, retrying once")
            self.kill_current_process()
            ok = (
                self._attempt_launch(gripper, cup_profile)
                and self._publish_verified_model(gripper)
            )

        if ok:
            self._model_revision = revision
            self._logger.info(f"Robot ready with {revision}")
        return ok

    def _publish_verified_model(self, gripper: str) -> bool:
        """Publish move_group's verified URDF/SRDF pair on the managed topics."""
        descriptions = self._wait_for_verified_model(gripper)
        if descriptions is None:
            return False

        robot_description, semantic_description = descriptions
        self._model_description_publishers[0].publish(String(data=robot_description))
        self._model_description_publishers[1].publish(String(data=semantic_description))
        self._logger.info(f"Published verified {gripper} model")
        return True

    def _wait_for_verified_model(
        self, gripper: str, timeout_sec: float = 10.0
    ) -> tuple[str, str] | None:
        """Wait until move_group returns non-empty URDF/SRDF strings."""
        deadline = time.monotonic() + timeout_sec
        if not self._move_group_parameters.wait_for_services(timeout_sec=timeout_sec):
            self._logger.error("move_group parameter services are not available")
            return None

        last_error = "move_group returned no robot descriptions"
        while time.monotonic() < deadline:
            future = self._move_group_parameters.get_parameters(
                ["robot_description", "robot_description_semantic"]
            )
            if not future.done():
                replied = threading.Event()
                future.add_done_callback(lambda _: replied.set())
                if not replied.wait(deadline - time.monotonic()):
                    break

            try:
                values = future.result().values
                descriptions = tuple(parameter_value_to_python(v) for v in values)
                if len(descriptions) == 2 and all(
                    isinstance(value, str) and value for value in descriptions
                ):
                    return descriptions
                last_error = "move_group returned empty robot descriptions"
            except Exception as error:
                last_error = f"could not read move_group descriptions: {error}"
            time.sleep(0.05)

        self._logger.error(
            f"Robot model for {gripper} not ready within {timeout_sec:.0f}s: "
            f"{last_error}"
        )
        return None

    def _attempt_launch(self, gripper: str, cup_profile: str) -> bool:
        """Launch once, load obstacles, and configure/verify real hardware."""
        # Payload and tool_voltage were validated when the beamline config loaded.
        config = self._grippers[gripper]
        self._logger.info(f"Launching MoveIt for {gripper} ({config['moveit_package']})")

        # Set voltage before ros2_control activates gripper Modbus; omitted
        # tool_voltage means 0 V (off), suitable for passive tools.
        if not self._use_mock_hardware:
            desired_voltage = int(config.get("tool_voltage", 0))
            if self._current_voltage == desired_voltage:
                self._logger.info(f"Tool voltage already at {desired_voltage}V, skipping")
            elif not self._set_tool_voltage(desired_voltage):
                return False
            else:
                self._current_voltage = desired_voltage
                # Allow Hand-E/ePick power-up before Modbus activation.
                time.sleep(2.0)

        # Isolate launch signal handling from the ROS executor; manage child
        # cleanup via the process group (github.com/ros2/launch issues #126, #545, #724).
        try:
            gripper_arg = config.get("gripper_arg", gripper)
            cmd = [
                "ros2", "launch", config["moveit_package"], "robot_bringup.launch.py",
                f"robot_ip:={self._robot_ip}",
                f"use_mock_hardware:={'true' if self._use_mock_hardware else 'false'}",
                f"enable_joystick:={'true' if self._enable_joystick else 'false'}",
                f"gripper:={gripper_arg}",
            ]

            # Only ePick uses cup_profile; other gripper launches ignore it.
            if cup_profile:
                cmd.append(f"cup_profile:={cup_profile}")

            self._logger.info(f"Executing: {' '.join(cmd)}")
            # opencv-python sets this on import; inherited, it crashes RViz.
            env = os.environ.copy()
            env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
            # Give launch its own process group for killpg() during shutdown.
            self._moveit_process = subprocess.Popen(cmd, start_new_session=True, env=env)
        except Exception as e:
            self._logger.error(f"Failed to launch MoveIt process: {e}")
            return False

        if not self._wait_for_moveit_ready(timeout_sec=45.0):
            return False

        if not self._load_collision_obstacles():
            return False

        # Relaunching ros2_control drops external_control even without a voltage
        # change, so restart it after every hardware launch.
        if not self._use_mock_hardware:
            if not self._restart_external_control():
                return False
            if not self._verify_hardware_connected():
                return False
            # Payload setup waits for io_and_status_controller's service.
            if not self._set_payload(config):
                return False

        return True

    def _set_payload(self, config: dict) -> bool:
        """Set the arm payload (validated at config load) before declaring ready."""
        mass = config["payload_mass"]
        cog = config["payload_cog"]

        from ur_msgs.srv import SetPayload

        request = SetPayload.Request()
        request.mass = float(mass)
        request.center_of_gravity = Vector3(
            x=float(cog["x"]), y=float(cog["y"]), z=float(cog["z"])
        )
        try:
            response = self._call_service(
                SetPayload, "/io_and_status_controller/set_payload", request,
                timeout_sec=10.0,
            )
        except Exception as error:
            self._logger.error(f"set_payload failed: {error}")
            return False

        if response is None or not response.success:
            self._logger.error("set_payload call failed")
            return False

        self._logger.info(f"Payload set to {mass}kg")
        return True

    def kill_current_process(self, drain: bool = True):
        """Stop the launch process group, then drain stale action discovery.

        Every node gets SIGTERM directly and shuts down through its own handler;
        wait for the whole group, not just the launch parent, which exits first
        and leaves slow nodes running. SIGKILL whatever remains after 3 s.

        Old /execute_trajectory entries can make the next launch appear ready
        before the new move_group services are available. Shutdown passes
        drain=False because no launch follows.
        """
        self._model_revision = ""
        if not self._moveit_process:
            return

        self._logger.info("Stopping MoveIt process...")
        start = time.monotonic()
        # start_new_session=True makes the pid the pgid, even after the parent is reaped.
        pgid = self._moveit_process.pid
        with contextlib.suppress(ProcessLookupError):
            os.killpg(pgid, signal.SIGTERM)

        while time.monotonic() - start < 3.0:
            self._moveit_process.poll()  # Reap the parent so its zombie doesn't count.
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            self._logger.warning("MoveIt still running after 3 s, sending SIGKILL")
            with contextlib.suppress(ProcessLookupError):
                os.killpg(pgid, signal.SIGKILL)

        self._moveit_process.wait()
        self._logger.info(f"MoveIt stopped in {time.monotonic() - start:.2f}s")
        self._moveit_process = None
        if drain:
            self._drain_stale_execute_trajectory()

    def _drain_stale_execute_trajectory(self, max_wait_sec: float = 10.0):
        """Wait until /execute_trajectory is not advertised to our node.

        Read the graph without allocating DDS endpoints. ActionClient construction
        can hang on DDS entity locking during post-SIGKILL cleanup, outside the
        Python timeout.
        """
        deadline = time.monotonic() + max_wait_sec
        while time.monotonic() < deadline:
            names = [n for n, _ in get_action_names_and_types(self._node)]
            if "/execute_trajectory" not in names:
                return
            time.sleep(0.1)
        self._logger.warning(
            "Stale /execute_trajectory still advertised after "
            f"{max_wait_sec:.0f}s drain; readiness check may fire early"
        )

    def _wait_for_moveit_ready(self, timeout_sec: float = 45.0) -> bool:
        """Wait for /execute_trajectory discovery, not full hardware readiness.

        Requires stale discovery entries to be drained first; a drain timeout
        can make this check see the previous launch.
        """
        self._logger.info("Waiting for MoveIt to be ready...")

        client = ActionClient(
            self._node, ExecuteTrajectory, "/execute_trajectory",
            callback_group=self._callback_group,
        )
        try:
            if client.wait_for_server(timeout_sec=timeout_sec):
                self._logger.info("MoveIt ready (/execute_trajectory advertised)")
                return True
            self._logger.error(f"MoveIt not ready within {timeout_sec:.0f}s")
            return False
        finally:
            client.destroy()

    def _verify_hardware_connected(self, timeout_sec: float = 10.0) -> bool:
        """Require fresh readings for every arm joint with sum(abs(position)) > 0.01 rad.

        MoveIt can outlive a crashed ros2_control process. Reuse the persistent
        subscription to avoid executor create/destroy races.
        """
        self._logger.info("Verifying hardware interface connection...")
        self._joint_positions.clear()
        expected = len(self._arm_joint_names)

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if (len(self._joint_positions) == expected and
                    sum(abs(v) for v in self._joint_positions.values()) > 0.01):
                self._logger.info("Hardware connected (joint states verified)")
                return True
            time.sleep(0.1)

        if not self._joint_positions:
            self._logger.error("No joint states received — hardware interface is dead")
        else:
            self._logger.error(
                f"Joint states suspect (got {len(self._joint_positions)}/{expected} joints, "
                f"sum(abs)={sum(abs(v) for v in self._joint_positions.values()):.4f})"
            )
        return False

    def _load_collision_obstacles(self) -> bool:
        """Apply the scene_file selected by BEAMBOT_BEAMLINE_CONFIG.

        /apply_planning_scene confirms the update and avoids startup topic-discovery
        races. An omitted scene_file skips obstacle loading.
        """
        try:
            config, config_path = load_beamline_config()
            scene_rel = config.get("scene_file", "")
            if not scene_rel:
                self._logger.info("No scene_file declared in beamline config; skipping obstacles")
                return True
            scene_path = Path(resolve_beamline_path(scene_rel, config_path))

            if not scene_path.exists():
                self._logger.error(f"Obstacle config not found: {scene_path}")
                return False
            with scene_path.open() as f:
                scene = yaml.safe_load(f)

            if not scene or "obstacles" not in scene:
                self._logger.info("No obstacles defined in config")
                return True

            planning_scene = PlanningScene()
            planning_scene.is_diff = True

            for obs in scene["obstacles"]:
                collision_object = CollisionObject()
                collision_object.id = obs["name"]
                collision_object.header.frame_id = obs["frame"]
                collision_object.operation = CollisionObject.ADD

                pose = Pose()
                pose.position.x = float(obs["pose"]["x"])
                pose.position.y = float(obs["pose"]["y"])
                pose.position.z = float(obs["pose"]["z"])

                roll = float(obs["pose"].get("roll", 0))
                pitch = float(obs["pose"].get("pitch", 0))
                yaw = float(obs["pose"].get("yaw", 0))
                q = quaternion_from_euler(roll, pitch, yaw)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                primitive = SolidPrimitive()
                obs_type = obs["type"]

                if obs_type == "box":
                    primitive.type = SolidPrimitive.BOX
                    size = obs["size"]
                    primitive.dimensions = [
                        float(size[0]), float(size[1]), float(size[2])
                    ]
                elif obs_type == "cylinder":
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [
                        float(obs["height"]), float(obs["radius"])
                    ]
                elif obs_type == "sphere":
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [float(obs["radius"])]
                else:
                    self._logger.warning(f"Unknown obstacle type: {obs_type}")
                    continue

                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                planning_scene.world.collision_objects.append(collision_object)

                self._logger.info(
                    f"  - Added {obs_type} '{collision_object.id}' "
                    f"in frame '{collision_object.header.frame_id}'"
                )

            if not planning_scene.world.collision_objects:
                return True

            response = self._call_service(
                ApplyPlanningScene, "/apply_planning_scene",
                ApplyPlanningScene.Request(scene=planning_scene),
            )
            if response is None:
                return False
            if not response.success:
                self._logger.error("MoveGroup rejected the planning scene diff")
                return False

            self._logger.info(
                f"Loaded {len(planning_scene.world.collision_objects)} obstacles"
            )
            return True

        except Exception as e:
            self._logger.error(f"Failed to load obstacles: {e}")
            self._logger.error(traceback.format_exc())
            return False

    def _set_tool_voltage(self, voltage: int) -> bool:
        """Send 0, 12 or 24 V via URScript before ROS driver services are available.

        The value is validated at config load. Socket-send success does not
        confirm the controller's resulting voltage.
        """
        if not self._robot_ip:
            self._logger.error("Robot IP not set")
            return False

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(self.SOCKET_TIMEOUT)

                self._logger.info(f"Connecting to {self._robot_ip}:{self.UR_SECONDARY_PORT}")
                sock.connect((self._robot_ip, self.UR_SECONDARY_PORT))

                cmd = f"set_tool_voltage({int(voltage)})\n"
                sock.sendall(cmd.encode())

            self._logger.info(f"Tool voltage set to {voltage}V")
            return True

        except OSError as e:  # Includes connection timeouts.
            self._logger.error(
                f"Socket error with {self._robot_ip}:{self.UR_SECONDARY_PORT}: {e}"
            )
            return False
        except Exception as e:
            self._logger.error(f"Failed to set tool voltage: {e}")
            return False

    def _restart_external_control(self) -> bool:
        """Restart external_control after voltage changes or driver relaunches.

        Require the driver's running-state signal, not just dashboard acceptance.
        """
        # Recreate before /play: discovery lag after driver restarts has caused a
        # persistent subscription to miss the True edge and time out after 5 s.
        self._robot_program_running.clear()
        program_sub = self._node.create_subscription(
            Bool, "/io_and_status_controller/robot_program_running",
            self._robot_program_cb, 10, callback_group=self._callback_group,
        )
        try:
            self._logger.info("Calling /dashboard_client/play...")
            response = self._call_service(
                Trigger, "/dashboard_client/play", Trigger.Request()
            )
            if response is None:
                return False
            if not response.success:
                self._logger.error(f"Dashboard /play rejected: {response.message}")
                return False

            # /play acceptance alone can leave the controller unready for the first goal.
            if not self._robot_program_running.wait(5.0):
                self._logger.error(
                    "robot_program_running=True not observed within 5s after /play"
                )
                return False

            self._logger.info("External control program restarted")
            return True

        except Exception as e:
            self._logger.error(f"Failed to restart external_control: {e}")
            return False
        finally:
            self._node.destroy_subscription(program_sub)

    def _robot_program_cb(self, msg):
        """Signal that the UR external_control program is running."""
        if msg.data:
            self._robot_program_running.set()
