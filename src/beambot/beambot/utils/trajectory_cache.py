"""Cache MTC solution messages for preview and repeated execution.

Keys include request JSON, model revision (which names the gripper and cup
profile) and normalized start joints.
A cache hit does not establish collision validity or execution readiness.
"""

import json
from collections import OrderedDict

MAX_ENTRIES = 64


def normalize_joints(positions_rad):
    """Round six arm-joint angles in radians, in canonical order.

    Angles are not wrapped: UR joints span +/-2*pi, so theta and theta + 2*pi are
    different start states and a plan from one would be rejected from the other.
    Return None when positions are missing or the count is not six.
    """
    if positions_rad is None:
        return None
    vals = list(positions_rad)
    if len(vals) != 6:
        return None
    # Round to 0.01 rad for cache matching; + 0.0 turns -0.0 into 0.0 so tiny
    # readings either side of zero match.
    return tuple(round(float(p), 2) + 0.0 for p in vals)


class TrajectoryCache:
    """Bounded least-recently-used cache of serialized MTC solutions."""

    def __init__(self, logger, max_entries: int = MAX_ENTRIES):
        self._logger = logger
        self._max_entries = max_entries
        # Ordered from least to most recently used.
        self._entries: "OrderedDict[tuple, object]" = OrderedDict()

    @staticmethod
    def compute_key(
        full_json: str,
        model_revision: str,
        start_joints_rad=None,
    ) -> tuple:
        """Key request JSON, model revision and normalized start joints."""
        try:
            normalized = json.dumps(json.loads(full_json), sort_keys=True)
        except json.JSONDecodeError:
            normalized = full_json
        return (normalized, model_revision, normalize_joints(start_joints_rad))

    def store(self, key: tuple, sol_msg) -> None:
        """Store or replace a solution and evict least-recently-used entries."""
        self._entries[key] = sol_msg
        self._entries.move_to_end(key)
        while len(self._entries) > self._max_entries:
            self._entries.popitem(last=False)
            self._logger.info("Plan cache evicted least-recently-used entry")

    def get(self, key: tuple):
        """Return the cached solution and mark it most recently used, or None."""
        sol_msg = self._entries.get(key)
        if sol_msg is not None:
            self._entries.move_to_end(key)
        return sol_msg
