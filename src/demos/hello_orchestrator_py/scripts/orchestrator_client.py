#!/usr/bin/env python3
"""Client that sends JSON tasks to the orchestrator."""

import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from hello_orchestrator_py_interfaces.action import OrchestratorTask


class OrchestratorClient(Node):
    """Client for sending tasks to the orchestrator."""

    def __init__(self):
        super().__init__('orchestrator_client')
        self._action_client = ActionClient(self, OrchestratorTask, 'orchestrator_task_py')

    def send_task(self, task_dict):
        """Send task to orchestrator. Returns True on success."""
        goal = OrchestratorTask.Goal()
        goal.task_json = json.dumps(task_dict)

        self.get_logger().info('Waiting for orchestrator...')
        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending task with {len(task_dict['tasks'])} steps")
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        if result.success:
            self.get_logger().info(f'Task completed ({result.completed_steps} steps)')
        else:
            self.get_logger().error(f'Task failed: {result.error_message}')

        return result.success

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Step {feedback.current_step}/{feedback.total_steps}: {feedback.current_action}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = OrchestratorClient()

    if len(sys.argv) < 2:
        client.get_logger().error('Usage: orchestrator_client <task_file.json>')
        client.destroy_node()
        rclpy.shutdown()
        return 1
    task_file = sys.argv[1]

    try:
        with open(task_file, 'r') as f:
            task = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        client.get_logger().error(f'Failed to load task file: {e}')
        client.destroy_node()
        rclpy.shutdown()
        return 1

    client.get_logger().info(f'Loading task from: {task_file}')
    success = client.send_task(task)

    client.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
