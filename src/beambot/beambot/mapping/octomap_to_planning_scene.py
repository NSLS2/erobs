#!/usr/bin/env python3
"""Forward Octomap updates to MoveIt's planning scene."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap


class OctomapToPlanningScene(Node):
    def __init__(self):
        super().__init__('octomap_to_planning_scene')

        self.declare_parameter('octomap_topic', '/octomap_binary')
        self.declare_parameter('planning_scene_topic', '/planning_scene')
        self.declare_parameter('min_update_interval', 0.5)  # Seconds.
        self.declare_parameter('log_updates', True)

        octomap_topic = self.get_parameter('octomap_topic').value
        planning_scene_topic = self.get_parameter('planning_scene_topic').value
        self._min_interval = self.get_parameter('min_update_interval').value
        self._log_updates = self.get_parameter('log_updates').value

        self._last_update_time = None
        self._updates_sent = 0
        self._updates_throttled = 0

        scene_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self._octomap_sub = self.create_subscription(
            Octomap,
            octomap_topic,
            self._octomap_callback,
            scene_qos
        )

        self._scene_pub = self.create_publisher(
            PlanningScene,
            planning_scene_topic,
            scene_qos
        )

        self.get_logger().info(
            f'Octomap→PlanningScene bridge started: '
            f'{octomap_topic} → {planning_scene_topic} '
            f'(throttle: {self._min_interval}s)'
        )

    def _octomap_callback(self, octomap_msg: Octomap):
        """Forward an Octomap update when the throttle interval has elapsed."""
        now = time.monotonic()

        if (
            self._last_update_time is not None
            and now - self._last_update_time < self._min_interval
        ):
            self._updates_throttled += 1
            return

        scene_msg = PlanningScene()
        scene_msg.is_diff = True  # Preserve the rest of the planning scene.

        scene_msg.world.octomap.header = octomap_msg.header
        scene_msg.world.octomap.octomap = octomap_msg

        self._scene_pub.publish(scene_msg)
        self._last_update_time = now
        self._updates_sent += 1

        if self._log_updates:
            data_size_kb = len(octomap_msg.data) / 1024
            self.get_logger().info(
                f'Published octomap to planning scene '
                f'(frame: {octomap_msg.header.frame_id}, '
                f'res: {octomap_msg.resolution:.3f}m, '
                f'size: {data_size_kb:.1f}KB, '
                f'updates: {self._updates_sent}, '
                f'throttled: {self._updates_throttled})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OctomapToPlanningScene()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Total updates: {node._updates_sent}, '
            f'throttled: {node._updates_throttled}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
