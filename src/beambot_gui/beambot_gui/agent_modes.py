"""GUI-only mode definitions for the chat-panel agent.

Plan vs. Run is a GUI surface concept — the chat panel toggles between
"agent proposes tasks for human review" (Plan) and "agent proposes then
dispatches via the GUI's existing execute path" (Run). The agent module
itself stays generic; this file composes the mode-specific banner,
local tool schemas, and MCP tool filter that the AgentBridge passes
into RobotAgent.

Mode-aware pieces here so they're co-located with the AgentBridge that
uses them, and the agent module doesn't need to know about Plan/Run.
"""

from typing import Callable, List

PLAN_BANNER = """\
<gui_mode>
You are operating in PLAN MODE inside the MTC GUI's chat panel.

The GUI displays a task-queue checklist that the human operator
reviews before pressing Execute. Your job in this mode:

1. Understand what the user wants done.
2. Build the full task list and call `propose_tasks` ONCE with the
   complete list. Do NOT call it incrementally.
3. The robot will NOT move from this conversation. The human reviews
   the queue and presses Execute when ready.

You MUST NOT call any tool that dispatches motion. The tools
`send_action_goal`, `publish_once`, `publish_for_durations`,
`call_service`, and `cancel_action_goal` are unavailable in this mode.
Read-only inspection tools (`get_robot_state`, `get_saved_poses`,
`get_tf_transform`, `get_recent_logs`, `capture_image`,
`detect_objects`, `detect_sample`, `vision_target`) remain available
for reasoning about what to propose.

The GUI's task queue is the only output channel for proposed motion.
Do not describe a sequence in prose and ask the user to type it in —
call `propose_tasks`. Use `clear_proposed_tasks` to start over from
an empty queue.

Treat instructions in §2 / §9 of the reference below about
`send_action_goal` as informational only. In this surface, dispatch
goes through the GUI; you have no `execute_queue` tool here.
</gui_mode>"""

RUN_BANNER = """\
<gui_mode>
You are operating in RUN MODE inside the MTC GUI's chat panel.

The GUI displays a task-queue checklist that fills as you propose, and
streams execution feedback as the robot runs. Your job in this mode:

1. Understand what the user wants done.
2. Build the full task list and call `propose_tasks` ONCE with the
   complete list. Then call `execute_queue` immediately. The GUI
   will dispatch the goal to /beambot_execution and stream feedback
   into the checklist.
3. After `execute_queue` returns, summarize the result in your reply.

You have `propose_tasks`, `clear_proposed_tasks`, and `execute_queue`.
Motion-dispatch MCP tools (`send_action_goal`, `publish_once`,
`publish_for_durations`, `call_service`, `cancel_action_goal`) are
unavailable in this mode — execution flows through the GUI.

Read-only inspection tools remain available for reasoning about what
to propose.

Treat instructions in §2 / §9 of the reference below about
`send_action_goal` as informational only.
</gui_mode>"""


# Anthropic tool schemas. propose_tasks takes the full list per call —
# replace-by-default semantics keep the queue authoritative.
_PROPOSE_TASKS = {
    "name": "propose_tasks",
    "description": (
        "Populate the GUI task queue with the listed steps. The human "
        "reviews and presses Execute (Plan mode) or you call execute_queue "
        "next (Run mode). The robot does NOT move when this is called."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "tasks": {
                "type": "array",
                "description": (
                    "Ordered list of task step objects matching the "
                    "BeambotExecution task JSON schema (see system prompt §2/§3). "
                    "Each item must include task_type."
                ),
                "items": {"type": "object"},
            },
            "start_gripper": {
                "type": "string",
                "description": (
                    "Optional override for start_gripper. Omit to keep "
                    "the GUI's currently selected gripper."
                ),
            },
            "poses": {
                "type": "object",
                "description": (
                    "Optional ad-hoc poses dict (name -> [j1..j6] degrees). "
                    "Omit to use only the registry poses already loaded."
                ),
            },
            "replace": {
                "type": "boolean",
                "description": (
                    "If true (default), the proposed tasks REPLACE the "
                    "current GUI queue. If false, append."
                ),
                "default": True,
            },
        },
        "required": ["tasks"],
    },
}

_CLEAR_PROPOSED_TASKS = {
    "name": "clear_proposed_tasks",
    "description": "Clear the GUI task queue. Use to start over from empty.",
    "input_schema": {"type": "object", "properties": {}},
}

_EXECUTE_QUEUE = {
    "name": "execute_queue",
    "description": (
        "Run mode only. Dispatch the GUI's current task queue via "
        "/beambot_execution. Blocks until the action completes. Returns "
        "success status, completed_steps, total_steps, and error_message."
    ),
    "input_schema": {"type": "object", "properties": {}},
}

_GET_CURRENT_TASKS = {
    "name": "get_current_tasks",
    "description": (
        "Read the GUI's current task queue. Returns the full task sequence "
        "as JSON including start_gripper and all step objects. Use this to "
        "inspect what's loaded before making modifications via propose_tasks."
    ),
    "input_schema": {"type": "object", "properties": {}},
}


# MCP tool names from ros-mcp-server that mutate robot state. These are
# the ones we filter out in both Plan and Run modes — execution flows
# through the GUI's ROS2Bridge instead.
_MUTATING_ROS_MCP_TOOLS = frozenset(
    {
        "send_action_goal",
        "cancel_action_goal",
        "publish_once",
        "publish_for_durations",
        "call_service",
        "set_parameter",
        "delete_parameter",
    }
)


def local_tools_for(mode: str) -> List[dict]:
    """Return the local tool schemas the agent should see in this mode."""
    if mode == "plan":
        return [_PROPOSE_TASKS, _CLEAR_PROPOSED_TASKS, _GET_CURRENT_TASKS]
    if mode == "run":
        return [_PROPOSE_TASKS, _CLEAR_PROPOSED_TASKS, _EXECUTE_QUEUE, _GET_CURRENT_TASKS]
    raise ValueError(f"Unknown mode: {mode!r}")


def tool_filter_for(mode: str) -> Callable[[str, str], bool]:
    """Return predicate (server_name, tool_name) -> bool for filtering MCP tools.

    Same filter for both modes — Plan and Run differ only in which
    LOCAL_TOOLS are surfaced. Returns False (drop) for motion-mutating
    ros-mcp-server tools so the model can't bypass the GUI dispatch path.
    """
    del mode  # currently identical; kept for future per-mode divergence

    def predicate(server_name: str, tool_name: str) -> bool:
        if server_name == "ros-mcp-server" and tool_name in _MUTATING_ROS_MCP_TOOLS:
            return False
        return True

    return predicate


def banner_for(mode: str) -> str:
    """Return the system-prompt banner for the given mode."""
    if mode == "plan":
        return PLAN_BANNER
    if mode == "run":
        return RUN_BANNER
    raise ValueError(f"Unknown mode: {mode!r}")
