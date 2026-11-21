#!/usr/bin/env python3
"""
ROS2 driver node for OnRobot 2FG7 parallel gripper.

Provides:
  - GripperCommand action server (for MTC/MoveIt integration)
  - Joint state publishing (for RViz visualization and MoveIt feedback)
  - Modbus RTU communication with the gripper via /tmp/ttyUR

The node translates MoveIt's joint position commands (in meters, representing
half the finger gap) into Modbus width commands (in mm, representing full
external width).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
import time

from onrobot_2fg7_driver.modbus_client import OnRobot2FG7Client


# Conversion: MoveIt joint position (meters, one-sided) <-> gripper width (mm, total external)
# The prismatic joint represents one finger's travel from center.
# External width = 2 * joint_position + min_external_width (when fingers are at 0)
# For simplicity: joint_pos=0 -> fingers at minimum width, joint_pos=max -> fingers at max width
# We map: joint_position (m) * 2 * 1000 = additional width in mm beyond minimum
MIN_EXTERNAL_WIDTH_MM = 1.0
MAX_EXTERNAL_WIDTH_MM = 73.0
# grip_pos_max in URDF = 0.0365 m -> 36.5mm one-sided -> 73mm total range + 1mm min = 74mm
# But actual hardware range is 1-73mm external


def joint_pos_to_width_mm(joint_pos_m: float) -> float:
    """Convert MoveIt joint position (m) to external grip width (mm)."""
    width_mm = MIN_EXTERNAL_WIDTH_MM + joint_pos_m * 2.0 * 1000.0
    return max(MIN_EXTERNAL_WIDTH_MM, min(MAX_EXTERNAL_WIDTH_MM, width_mm))


def width_mm_to_joint_pos(width_mm: float) -> float:
    """Convert external grip width (mm) to MoveIt joint position (m)."""
    return max(0.0, (width_mm - MIN_EXTERNAL_WIDTH_MM) / 2.0 / 1000.0)


class OnRobot2FG7DriverNode(Node):
    """ROS2 driver node for OnRobot 2FG7 gripper."""

    def __init__(self):
        super().__init__('onrobot_2fg7_driver')

        # Parameters
        self.declare_parameter('serial_port', '/tmp/ttyUR')
        self.declare_parameter('slave_id', 0x41)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('default_force_n', 40)
        self.declare_parameter('default_speed_pct', 100)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('use_mock_hardware', False)

        self.serial_port = self.get_parameter('serial_port').value
        self.slave_id = self.get_parameter('slave_id').value
        self.baudrate = self.get_parameter('baudrate').value
        self.default_force = self.get_parameter('default_force_n').value
        self.default_speed = self.get_parameter('default_speed_pct').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.use_mock_hardware = self.get_parameter('use_mock_hardware').value

        # Joint names (must match URDF)
        self.left_joint = 'twofg7_left_finger_joint'
        self.right_joint = 'twofg7_right_finger_joint'

        # Current state
        self._current_width_mm = 60.0  # Assume open

        # Initialize Modbus client
        self.gripper = None
        if not self.use_mock_hardware:
            self._connect_gripper()
        else:
            self.get_logger().info('Running in MOCK HARDWARE mode')

        # Callback group for concurrent action handling
        cb_group = ReentrantCallbackGroup()

        # GripperCommand action server (MTC/MoveIt interface)
        self._action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_action_controller/gripper_cmd',
            self._execute_gripper_command,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # Joint state publisher
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._publish_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_joint_states, callback_group=cb_group
        )

        self.get_logger().info(
            f'OnRobot 2FG7 driver started '
            f'(port={self.serial_port}, slave=0x{self.slave_id:02X}, baud={self.baudrate})'
        )

    def _connect_gripper(self, max_retries=10, retry_delay=2.0):
        """Connect to gripper via Modbus RTU with retries."""
        for attempt in range(max_retries):
            self.gripper = OnRobot2FG7Client(
                port=self.serial_port,
                slave_id=self.slave_id,
                baudrate=self.baudrate,
                logger=self.get_logger(),
            )
            if self.gripper.connect():
                status = self.gripper.read_status()
                if status:
                    self._current_width_mm = status.external_width_mm
                    self.get_logger().info(
                        f'Connected to 2FG7 gripper (width: {status.external_width_mm:.1f}mm)')
                    return
                else:
                    self.get_logger().warning(
                        f'Connected but no status response (attempt {attempt+1}/{max_retries})')
                    self.gripper.disconnect()
            else:
                self.get_logger().warning(
                    f'Connection failed (attempt {attempt+1}/{max_retries}), retrying in {retry_delay}s...')

            self.gripper = None
            if attempt < max_retries - 1:
                time.sleep(retry_delay)

        self.get_logger().error(f'Failed to connect to 2FG7 gripper after {max_retries} attempts')

    def _publish_joint_states(self):
        """Publish current finger joint positions."""
        width_mm = self._current_width_mm

        # Update from hardware periodically
        if self.gripper and not self.use_mock_hardware:
            status = self.gripper.read_status()
            if status:
                self._current_width_mm = status.external_width_mm
                width_mm = status.external_width_mm

        joint_pos = width_mm_to_joint_pos(width_mm)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.left_joint, self.right_joint]
        msg.position = [joint_pos, joint_pos]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self._joint_state_pub.publish(msg)

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'GripperCommand goal: position={goal_request.command.position:.4f}m, '
            f'max_effort={goal_request.command.max_effort:.1f}N'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('GripperCommand cancel requested')
        return CancelResponse.ACCEPT

    def _execute_gripper_command(self, goal_handle):
        """Execute a GripperCommand action goal."""
        target_pos = goal_handle.request.command.position  # meters (one-sided)
        max_effort = goal_handle.request.command.max_effort  # Newtons

        target_width_mm = joint_pos_to_width_mm(target_pos)
        force_n = int(max_effort) if max_effort > 0 else self.default_force

        self.get_logger().info(
            f'Moving to width={target_width_mm:.1f}mm (joint_pos={target_pos:.4f}m), '
            f'force={force_n}N, speed={self.default_speed}%'
        )

        result = GripperCommand.Result()

        if self.use_mock_hardware:
            time.sleep(0.5)
            self._current_width_mm = target_width_mm
            result.position = target_pos
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = True
            goal_handle.succeed()
            return result

        if not self.gripper:
            self.get_logger().error('Gripper not connected')
            goal_handle.abort()
            result.reached_goal = False
            return result

        # Send grip command
        success = self.gripper.grip_external(
            target_width_mm, force_n, self.default_speed
        )
        if not success:
            self.get_logger().error('Failed to send grip command')
            goal_handle.abort()
            result.reached_goal = False
            return result

        # Wait for completion (poll busy flag)
        timeout = 5.0
        elapsed = 0.0
        poll_interval = 0.1

        while elapsed < timeout:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('GripperCommand canceled')
                goal_handle.canceled()
                result.reached_goal = False
                return result

            time.sleep(poll_interval)
            elapsed += poll_interval

            status = self.gripper.read_status()
            if status:
                self._current_width_mm = status.external_width_mm

                # Publish feedback
                feedback = GripperCommand.Feedback()
                feedback.position = width_mm_to_joint_pos(status.external_width_mm)
                feedback.effort = 0.0
                feedback.stalled = False
                feedback.reached_goal = False
                goal_handle.publish_feedback(feedback)

                if not status.busy and elapsed > 0.3:
                    break

        # Read final state
        status = self.gripper.read_status()
        if status:
            self._current_width_mm = status.external_width_mm
            result.position = width_mm_to_joint_pos(status.external_width_mm)
            result.effort = 0.0
            result.stalled = False
            reached = abs(status.external_width_mm - target_width_mm) < 5.0  # 5mm tolerance
            result.reached_goal = reached
        else:
            result.position = target_pos
            result.reached_goal = False

        if result.reached_goal:
            self.get_logger().info(
                f'Grip complete: width={self._current_width_mm:.1f}mm'
            )
            goal_handle.succeed()
        else:
            self.get_logger().warning(
                f'Grip may not have reached target: '
                f'current={self._current_width_mm:.1f}mm, target={target_width_mm:.1f}mm'
            )
            goal_handle.succeed()  # Still succeed — gripper may have hit object

        return result

    def destroy_node(self):
        self.get_logger().info('Shutting down 2FG7 driver')
        if self.gripper:
            self.gripper.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OnRobot2FG7DriverNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            executor.shutdown()
    except Exception as e:
        print(f"Failed to start 2FG7 driver: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
