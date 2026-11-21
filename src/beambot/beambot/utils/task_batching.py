"""Group consecutive motion and end-effector tasks for the orchestrator."""

from typing import Any

BATCHABLE_TYPES = {"moveto", "end_effector"}


def group_into_batches(
    tasks: list[dict[str, Any]],
    enabled: bool = True,
) -> list[tuple[str, list[dict[str, Any]]]]:
    """Return ordered ("batched" or "single", tasks) groups.

    When enabled, group consecutive moveto/end_effector tasks; keep others single.
    When disabled, every task is single.
    """
    if not enabled:
        return [("single", [task]) for task in tasks]

    batches = []
    current_batch: list[dict[str, Any]] = []

    for task in tasks:
        task_type = task.get("task_type", "")
        if task_type in BATCHABLE_TYPES:
            current_batch.append(task)
        else:
            if current_batch:
                batches.append(("batched", current_batch))
            batches.append(("single", [task]))
            current_batch = []

    if current_batch:
        batches.append(("batched", current_batch))

    return batches
