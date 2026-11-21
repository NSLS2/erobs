"""ZED marker detection from streamed images and point clouds."""

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from beambot.vision.camera import DetectionResult
from beambot.vision.perception import get_3d_position
from beambot.vision.perception.image_detection import detect_aruco_markers


IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"

ZED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def create_client(node: Node):
    """Return None for the shared camera interface; ZED needs no service client."""
    return None


def _grab_latest(
    node: Node,
    timeout: float = 5.0,
    need_cloud: bool = True,
) -> tuple[Image | None, PointCloud2 | None]:
    """Collect an image and optional point cloud; unavailable messages remain None."""
    received_image: list[Image | None] = [None]
    received_cloud: list[PointCloud2 | None] = [None]

    def on_image(msg: Image):
        received_image[0] = msg

    def on_cloud(msg: PointCloud2):
        received_cloud[0] = msg

    image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, ZED_QOS)
    cloud_sub = node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, ZED_QOS) if need_cloud else None

    try:
        max_iterations = int(timeout * 10)
        for i in range(max_iterations):
            rclpy.spin_once(node, timeout_sec=0.1)
            image_ready = received_image[0] is not None
            cloud_ready = received_cloud[0] is not None or not need_cloud
            if image_ready and cloud_ready:
                break

        return received_image[0], received_cloud[0]
    finally:
        node.destroy_subscription(image_sub)
        if cloud_sub is not None:
            node.destroy_subscription(cloud_sub)


def detect_markers(
    client,
    node: Node,
    marker_ids: list[int] | None = None,
    dictionary: str = "aruco4x4_50",
    timeout: float = 10.0,
    settle_time: float = 0.0,
) -> DetectionResult:
    """Detect ArUco markers with OpenCV and look up their point-cloud positions."""
    # client and settle_time are unused; retained for the shared camera interface.
    import cv2
    logger = node.get_logger()
    bridge = CvBridge()

    image_msg, cloud_msg = _grab_latest(node, timeout=timeout, need_cloud=True)

    if image_msg is None:
        logger.error("No image received from ZED")
        return DetectionResult(markers=[], capture_stamp=None)

    capture_stamp = image_msg.header.stamp
    rgb_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')

    dict_map = {
        "aruco4x4_50": cv2.aruco.DICT_4X4_50,
        "aruco5x5_100": cv2.aruco.DICT_5X5_100,
        "aruco6x6_250": cv2.aruco.DICT_6X6_250,
    }
    dict_id = dict_map.get(dictionary, cv2.aruco.DICT_4X4_50)
    corners_list, ids = detect_aruco_markers(rgb_image, dict_id)

    if ids is None or len(ids) == 0:
        if marker_ids is None:
            logger.warning("No markers detected by ZED")
        else:
            logger.warning(f"No markers found for requested IDs: {marker_ids}")
        return DetectionResult(markers=[], capture_stamp=capture_stamp)

    detected = []
    for i, mid in enumerate(ids.flatten()):
        mid = int(mid)
        if marker_ids is not None and mid not in marker_ids:
            continue

        corners = corners_list[i][0]
        cx = int(corners[:, 0].mean())
        cy = int(corners[:, 1].mean())

        pose = Pose()
        if cloud_msg is not None:
            xyz = get_3d_position(cloud_msg, cx, cy)
            if xyz is not None:
                pose.position.x, pose.position.y, pose.position.z = xyz
                logger.info(
                    f"Marker {mid}: ZED OpenCV → "
                    f"({xyz[0]*1000:.2f}, {xyz[1]*1000:.2f}, {xyz[2]*1000:.2f}) mm"
                )
            else:
                logger.warning(f"Marker {mid}: No valid depth at ({cx}, {cy})")
        else:
            logger.warning(f"Marker {mid}: No point cloud — 2D only")

        # Marker orientation is not estimated.
        pose.orientation.w = 1.0
        detected.append((mid, pose))

    return DetectionResult(markers=detected, capture_stamp=capture_stamp)
