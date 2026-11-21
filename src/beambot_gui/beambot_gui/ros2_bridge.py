"""ROS2-Qt bridge: ROS2 node with pyqtSignal emissions for thread-safe UI updates."""

import math
import time
import threading

from PyQt6.QtCore import QObject, QThread, pyqtSignal

# ROS2 imports (optional — GUI works without ROS2 for UI development)
try:
    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from sensor_msgs.msg import Image as RosImage, JointState
    from std_msgs.msg import String as RosString
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge
    from moveit_msgs.msg import DisplayTrajectory
    from beambot_interfaces.action import BeambotExecution
    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ROS2 not available: {e}")
    ROS2_AVAILABLE = False

try:
    from zivid_interfaces.srv import CaptureAndDetectMarkers
    ZIVID_AVAILABLE = True
except ImportError:
    ZIVID_AVAILABLE = False


class ROS2Spinner(QObject):
    """Worker that spins rclpy in a QThread."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self._running = True

    def run(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                print(f"ROS2 spin error: {e}")
                break

    def stop(self):
        self._running = False


class ROS2Bridge(QObject):
    """Thread-safe bridge between ROS2 callbacks and Qt UI via signals."""

    # Signals (emitted from ROS2 thread, received on Qt main thread)
    image_received = pyqtSignal(object)          # numpy array (BGR)
    joint_state_received = pyqtSignal(list)      # [6 floats] degrees
    gripper_changed = pyqtSignal(str)            # current gripper name
    detection_received = pyqtSignal(list)        # MarkerShape list
    action_feedback_received = pyqtSignal(float, int, str, str, str)  # progress, step, action, gripper, msg
    action_result_received = pyqtSignal(int, str, int, int)  # status, error_msg, completed, total
    preview_trajectory_received = pyqtSignal(list, list)  # joint_names, waypoints[(positions, t_from_start_sec)]
    execution_state_changed = pyqtSignal(str)
    log = pyqtSignal(str)                        # thread-safe logging

    def __init__(self, parent=None):
        super().__init__(parent)
        self.node = None
        self._spin_thread = None
        self._spinner = None
        self._bridge = CvBridge() if ROS2_AVAILABLE else None
        self._action_client = None
        self._capture_client = None
        self._marker_client = None
        self._pause_client = None
        self._resume_client = None
        self._current_goal_handle = None
        self._stop_execution = False
        self.execution_state = None

    @property
    def available(self):
        return ROS2_AVAILABLE and self.node is not None

    def init_ros2(self):
        if not ROS2_AVAILABLE:
            return False
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("beambot_gui_client")

            # Subscriptions
            self.node.create_subscription(RosImage, "/color/image_color", self._on_image, 10)
            self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
            latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE)
            self.node.create_subscription(
                RosString, "/beambot/current_gripper", self._on_gripper, latched_qos)
            self.node.create_subscription(
                RosString,
                "/beambot/execution_state",
                self._on_execution_state,
                latched_qos,
            )

            # Dry-run preview trajectory: latched so the GUI sees the latest
            # preview even if it subscribes after the orchestrator publishes.
            self.node.create_subscription(
                DisplayTrajectory,
                "/beambot/preview_trajectory",
                self._on_preview_trajectory,
                latched_qos,
            )

            # Action client
            self._action_client = ActionClient(self.node, BeambotExecution, "beambot_execution")

            # Service clients
            self._capture_client = self.node.create_client(Trigger, "/capture_2d")
            self._pause_client = self.node.create_client(Trigger, "beambot/pause")
            self._resume_client = self.node.create_client(Trigger, "beambot/resume")
            if ZIVID_AVAILABLE:
                self._marker_client = self.node.create_client(
                    CaptureAndDetectMarkers, "/capture_and_detect_markers"
                )

            # Start spin thread
            self._spin_thread = QThread()
            self._spinner = ROS2Spinner(self.node)
            self._spinner.moveToThread(self._spin_thread)
            self._spin_thread.started.connect(self._spinner.run)
            self._spin_thread.start()

            self.log.emit("ROS2 initialized")
            return True
        except Exception as e:
            self.log.emit(f"ROS2 init failed: {e}")
            return False

    def shutdown(self):
        if self._spinner:
            self._spinner.stop()
        if self._spin_thread:
            self._spin_thread.quit()
            self._spin_thread.wait(3000)
        if self.node:
            self.node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # --- ROS2 callbacks (run on spin thread, emit signals) ---

    def _on_image(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_received.emit(cv_image)
        except Exception as e:
            print(f"Image callback error: {e}")

    def _on_joint_state(self, msg):
        try:
            from beambot.config_loader import arm_joint_names
            joint_dict = dict(zip(msg.name, msg.position))
            joint_order = arm_joint_names()
            if all(j in joint_dict for j in joint_order):
                pose_deg = [round(math.degrees(joint_dict[j]), 2) for j in joint_order]
                self.joint_state_received.emit(pose_deg)
        except Exception as e:
            print(f"Joint state callback error: {e}")

    def _on_gripper(self, msg):
        self.gripper_changed.emit(msg.data)

    def _on_execution_state(self, msg):
        self.execution_state = msg.data
        self.execution_state_changed.emit(msg.data)

    def _on_preview_trajectory(self, msg):
        """Forward a DisplayTrajectory to the viz panel as a flat waypoint list.

        Each waypoint is (joint_names_for_that_point, positions, t_seconds).
        Different MTC sub-trajectories may have different joint sets (arm-only
        vs. gripper-only), so we keep names per-waypoint and let the panel
        pick the joints it knows how to visualize. time_from_start in each
        sub-trajectory restarts at zero, so we offset by a running total.
        """
        try:
            waypoints = []  # list of (joint_names, positions, t_seconds)
            cumulative = 0.0
            for traj in msg.trajectory:
                jt = traj.joint_trajectory
                if not jt.joint_names or not jt.points:
                    continue
                names = list(jt.joint_names)
                segment_end = 0.0
                for pt in jt.points:
                    t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    waypoints.append((names, list(pt.positions), cumulative + t))
                    if t > segment_end:
                        segment_end = t
                cumulative += segment_end
            if waypoints:
                # First arg is unused now (kept for signal compatibility);
                # per-waypoint joint_names live in waypoints[i][0].
                self.preview_trajectory_received.emit([], waypoints)
                self.log.emit(
                    f"Preview trajectory: {len(waypoints)} waypoints, "
                    f"{cumulative:.2f}s total"
                )
        except Exception as e:
            print(f"Preview trajectory callback error: {e}")

    # --- Action client ---

    def execute_task(self, config_json: str, dry_run: bool = False):
        """Send task to orchestrator. Runs in background thread.

        Args:
            config_json: Full task script JSON.
            dry_run: When True, request a plan-only preview from the
                orchestrator (publishes a trajectory on
                /beambot/preview_trajectory; no robot motion).
        """
        self._stop_execution = False

        def _run():
            try:
                if not self._action_client:
                    self.log.emit("ERROR: Action client not available")
                    self.action_result_received.emit(
                        0, "Action client unavailable", 0, 0
                    )
                    return
                self.log.emit("Waiting for beambot_execution action server...")
                if not self._action_client.wait_for_server(timeout_sec=10.0):
                    self.log.emit("ERROR: Action server not available")
                    self.action_result_received.emit(0, "Action server unavailable", 0, 0)
                    return
                if self._stop_execution:
                    self.action_result_received.emit(
                        GoalStatus.STATUS_CANCELED, "Stopped before sending goal", 0, 0
                    )
                    return

                goal = BeambotExecution.Goal()
                goal.full_json = config_json
                goal.dry_run = dry_run
                self.log.emit(
                    "Sending dry-run preview goal..." if dry_run else "Sending goal..."
                )

                send_future = self._action_client.send_goal_async(
                    goal, feedback_callback=self._on_action_feedback
                )
                while not send_future.done():
                    time.sleep(0.1)

                self._current_goal_handle = send_future.result()
                if not self._current_goal_handle.accepted:
                    self.log.emit("ERROR: Goal rejected")
                    self.action_result_received.emit(0, "Goal rejected", 0, 0)
                    return

                self.log.emit("Goal accepted")
                result_future = self._current_goal_handle.get_result_async()
                cancel_sent = False
                while not result_future.done():
                    if self._stop_execution and not cancel_sent:
                        cancel_sent = True
                        try:
                            self._current_goal_handle.cancel_goal_async()
                        except Exception as error:
                            self.log.emit(
                                f"Cancellation request failed: {error}; waiting for task result"
                            )
                    time.sleep(0.1)

                result = result_future.result()
                r = result.result
                self.action_result_received.emit(
                    result.status, r.error_message, r.completed_steps, r.total_steps
                )
            except Exception as e:
                self.log.emit(f"Execution error: {e}")
                self.action_result_received.emit(0, str(e), 0, 0)
            finally:
                self._current_goal_handle = None

        threading.Thread(target=_run, daemon=True).start()

    def stop_execution(self):
        self._stop_execution = True
        self.log.emit("Stopping... waiting for task completion")

    def _on_action_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.action_feedback_received.emit(
            fb.progress_percentage, fb.current_step,
            fb.current_action, fb.current_gripper, fb.status_message,
        )

    # --- Service calls ---

    def trigger_capture(self):
        self._call_trigger_service(self._capture_client, "Capture")

    def pause_task(self):
        self._call_trigger_service(self._pause_client, "Pause")

    def resume_task(self):
        self._call_trigger_service(self._resume_client, "Resume")

    def _call_trigger_service(self, client, name):
        if not client:
            self.log.emit(f"{name}: service not available")
            return

        def _call():
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    self.log.emit(f"{name}: service not ready")
                    return
                future = client.call_async(Trigger.Request())
                start = time.time()
                while not future.done():
                    if time.time() - start > 5.0:
                        self.log.emit(f"{name}: timeout")
                        return
                    time.sleep(0.01)
                result = future.result()
                if result.success:
                    self.log.emit(f"{name}: {result.message}")
                else:
                    self.log.emit(f"{name} failed: {result.message}")
            except Exception as e:
                self.log.emit(f"{name} error: {e}")

        threading.Thread(target=_call, daemon=True).start()

    def trigger_marker_detection(self):
        if not self._marker_client:
            self.log.emit("Marker detection: service not available")
            return

        def _detect():
            try:
                if not self._marker_client.wait_for_service(timeout_sec=2.0):
                    self.log.emit("Marker detection: service not ready")
                    return
                request = CaptureAndDetectMarkers.Request()
                request.marker_ids = list(range(50))
                request.marker_dictionary = "aruco4x4_50"
                future = self._marker_client.call_async(request)
                start = time.time()
                while not future.done():
                    if time.time() - start > 10.0:
                        self.log.emit("Marker detection: timeout")
                        return
                    time.sleep(0.1)
                response = future.result()
                if response.success:
                    self.detection_received.emit(response.detection_result.detected_markers)
                else:
                    self.log.emit(f"Detection failed: {response.message}")
            except Exception as e:
                self.log.emit(f"Detection error: {e}")

        threading.Thread(target=_detect, daemon=True).start()

    def test_server(self):
        """Test if the action server is reachable."""
        def _test():
            if not self.available:
                self.log.emit("ROS2 not available")
                return
            self.log.emit("Testing beambot_execution server...")
            if self._action_client.wait_for_server(timeout_sec=3.0):
                self.log.emit("Action server is running")
            else:
                self.log.emit("Action server not available")
        threading.Thread(target=_test, daemon=True).start()
