#!/usr/bin/env python3
"""Orchestrator server - dispatches JSON tasks to specialized action servers."""

import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from hello_orchestrator_py_interfaces.action import (
    OrchestratorTask,
    PrintMessage,
    MoveToNamedState,
)


class OrchestratorServer(Node):
    """Orchestrator that dispatches tasks to print/move action servers."""

    def __init__(self):
        super().__init__("orchestrator_server_py")

        self._executing = False
        self._print_client = ActionClient(self, PrintMessage, "print_message_py")
        self._move_client = ActionClient(self, MoveToNamedState, "move_to_named_state_py")

        self._action_server = ActionServer(
            self,
            OrchestratorTask,
            "orchestrator_task_py",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info("Orchestrator server started on 'orchestrator_task_py'")

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._executing:
            self.get_logger().warning("Goal rejected: orchestrator busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        # Orchestrator accepts cancel - stops between tasks (not mid-task).
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        self._executing = True
        try:
            return self._execute(goal_handle)
        finally:
            self._executing = False

    def _execute(self, goal_handle: ServerGoalHandle) -> OrchestratorTask.Result:
        result = OrchestratorTask.Result()
        result.success = False
        result.completed_steps = 0
        feedback = OrchestratorTask.Feedback()

        try:
            task_json = json.loads(goal_handle.request.task_json)
            tasks = task_json["tasks"]
        except (json.JSONDecodeError, KeyError) as e:
            result.error_message = f"Invalid task JSON: {e}"
            goal_handle.abort()
            return result
        
        feedback.total_steps = len(tasks)
        self.get_logger().info(f"Executing {len(tasks)} tasks")

        for i, task in enumerate(tasks):
            if goal_handle.is_cancel_requested:
                result.error_message = "Task was canceled"
                result.completed_steps = i
                goal_handle.canceled()
                return result

            task_type = task.get("type")
            if not task_type:
                result.error_message = f"Task {i} missing 'type' field"
                result.completed_steps = i
                goal_handle.abort()
                return result

            feedback.current_step = i + 1
            feedback.current_action = task_type
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Step {i+1}/{len(tasks)}: {task_type}")

            if task_type == "print":
                success = self._dispatch_print(task)
            elif task_type == "move":
                success = self._dispatch_move(task)
            else:
                result.error_message = f"Unknown task type: {task_type}"
                result.completed_steps = i
                goal_handle.abort()
                return result

            if not success:
                result.error_message = f"Task {i} ({task_type}) failed"
                result.completed_steps = i
                goal_handle.abort()
                return result

            result.completed_steps = i + 1

        result.success = True
        result.error_message = ""
        goal_handle.succeed()
        self.get_logger().info("All tasks completed successfully")
        return result

    def _call_action(self, client, goal, timeout_sec: float = 60.0) -> bool:
        """Generic action client helper."""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return False

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("Goal acceptance timeout")
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error("Action timeout")
            return False

        return result_future.result().result.success

    def _dispatch_print(self, task: dict) -> bool:
        if "message" not in task:
            self.get_logger().error("Print task missing 'message' field")
            return False
        goal = PrintMessage.Goal()
        goal.message = task["message"]
        return self._call_action(self._print_client, goal, timeout_sec=10.0)

    def _dispatch_move(self, task: dict) -> bool:
        if "target_pose" not in task:
            self.get_logger().error("Move task missing 'target_pose' field")
            return False
        goal = MoveToNamedState.Goal()
        goal.target_pose = task["target_pose"]
        return self._call_action(self._move_client, goal, timeout_sec=60.0)


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
