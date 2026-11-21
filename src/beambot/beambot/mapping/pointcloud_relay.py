#!/usr/bin/env python3
"""Downsample camera point clouds for Octomap."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')

        self.declare_parameter('input_topic', '/points/xyz')
        self.declare_parameter('output_topic', '/points/xyz_relayed')
        self.declare_parameter('voxel_size', 0.02)  # Meters.
        self.declare_parameter('enable_downsampling', True)
        self.declare_parameter('enable_noise_filter', False)  # Statistical outlier removal.
        self.declare_parameter('noise_nb_neighbors', 20)
        self.declare_parameter('noise_std_ratio', 2.0)  # Standard-deviation multiplier.

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.enable_downsampling = self.get_parameter('enable_downsampling').value
        self.enable_noise_filter = self.get_parameter('enable_noise_filter').value
        self.noise_nb_neighbors = self.get_parameter('noise_nb_neighbors').value
        self.noise_std_ratio = self.get_parameter('noise_std_ratio').value

        # Relay unchanged clouds when Open3D is unavailable.
        if self.enable_downsampling and not OPEN3D_AVAILABLE:
            self.get_logger().warning(
                'Downsampling requested but Open3D not available. '
                'Install with: pip install open3d. Disabling downsampling.'
            )
            self.enable_downsampling = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            qos
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos
        )

        modes = []
        if self.enable_downsampling:
            modes.append(f"voxel={self.voxel_size}m")
        if self.enable_noise_filter and self.enable_downsampling:
            modes.append(f"noise_filter(k={self.noise_nb_neighbors}, std={self.noise_std_ratio})")
        elif self.enable_noise_filter:
            self.get_logger().info(
                'Noise filtering is inactive because downsampling is disabled.'
            )
        mode_str = ", ".join(modes) if modes else "passthrough"
        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic} ({mode_str})'
        )

    def callback(self, msg: PointCloud2):
        original_count = msg.width * msg.height

        if self.enable_downsampling:
            downsampled_msg = self._downsample_pointcloud(msg)
            if downsampled_msg is not None:
                self.publisher.publish(downsampled_msg)
                new_count = downsampled_msg.width * downsampled_msg.height
                reduction = (1 - new_count / original_count) * 100
                self.get_logger().info(
                    f'Relayed {original_count:,} -> {new_count:,} points '
                    f'({reduction:.1f}% reduction)'
                )
            else:
                self.publisher.publish(msg)
                self.get_logger().warning(
                    f'Downsampling failed, relayed original {original_count:,} points'
                )
        else:
            self.publisher.publish(msg)
            self.get_logger().info(f'Relayed {original_count:,} points (passthrough)')

    def _downsample_pointcloud(self, msg: PointCloud2) -> PointCloud2:
        """Filter and downsample XYZ points; return None on failure."""
        try:
            points_np = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'])
            # Filter NaNs even when the message is marked dense.
            points_np = points_np[~np.isnan(points_np).any(axis=1)]

            if len(points_np) == 0:
                self.get_logger().warning('No valid points in cloud after NaN filtering')
                return None

            points_np = points_np.astype(np.float64)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np)

            if self.enable_noise_filter:
                pcd, inlier_indices = pcd.remove_statistical_outlier(
                    nb_neighbors=self.noise_nb_neighbors,
                    std_ratio=self.noise_std_ratio
                )

            pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            points_down = np.asarray(pcd_down.points, dtype=np.float32)

            return point_cloud2.create_cloud_xyz32(msg.header, points_down)

        except Exception as e:
            self.get_logger().error(f'Downsampling error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
