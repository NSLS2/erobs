"""Zivid camera backend for marker, sample ROI, and 2D image capture."""

import time

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from zivid_interfaces.srv import CaptureAndDetectMarkers

from beambot.vision.camera import DetectionResult
from beambot.vision.perception import (
    SampleRoiDetectionParams,
    detect_sample_in_roi,
    sample_roi_pickup_camera_xyz,
)


def _wait_for_future(future, timeout: float, poll_interval: float = 0.01) -> bool:
    """Wait for an executor-driven future without spinning this node."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_interval)
    return True


SERVICE_NAME = "/capture_and_detect_markers"
CAPTURE_2D_SERVICE = "/capture_2d"
IMAGE_TOPIC = "/color/image_color"

# Match the Zivid publisher.
_ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def capture_2d(node: Node, timeout: float = 15.0) -> np.ndarray | None:
    """Capture a flash-lit BGR image without the 3D projector."""
    deadline = time.monotonic() + max(0.0, timeout)
    bridge = CvBridge()
    received: list[Image | None] = [None]

    def on_image(msg: Image):
        received[0] = msg

    # Allow the executor to deliver the service and image callbacks concurrently.
    cb_group = ReentrantCallbackGroup()
    sub = node.create_subscription(
        Image, IMAGE_TOPIC, on_image, _ZIVID_QOS, callback_group=cb_group
    )
    client = node.create_client(Trigger, CAPTURE_2D_SERVICE, callback_group=cb_group)

    try:
        remaining = max(0.0, deadline - time.monotonic())
        if remaining <= 0:
            node.get_logger().error("2D capture timed out")
            return None
        if not client.wait_for_service(timeout_sec=min(2.0, remaining)):
            node.get_logger().error(f"Service '{CAPTURE_2D_SERVICE}' not available")
            return None

        # Let the subscription connect, then discard any stale frame.
        time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
        if time.monotonic() >= deadline:
            node.get_logger().error("2D capture timed out")
            return None
        received[0] = None

        try:
            future = client.call_async(Trigger.Request())
            if not _wait_for_future(future, max(0.0, deadline - time.monotonic())):
                node.get_logger().error("2D capture service timed out")
                return None
            result = future.result()
        except Exception as exc:
            node.get_logger().error(f"2D capture service failed: {exc}")
            return None

        if not result.success:
            node.get_logger().warning(f"2D capture failed: {result.message}")
            return None

        while received[0] is None and time.monotonic() < deadline:
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        if received[0] is None:
            node.get_logger().error("No image received after 2D capture")
            return None

        return bridge.imgmsg_to_cv2(received[0], desired_encoding="bgr8")

    finally:
        node.destroy_subscription(sub)
        node.destroy_client(client)


def create_client(node: Node):
    """Create the marker-capture service client."""
    return node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)


def detect_markers(
    client,
    node: Node,
    marker_ids: list[int] | None = None,
    dictionary: str = "aruco4x4_50",
    timeout: float = 45.0,
    settle_time: float = 0.0,
) -> DetectionResult:
    """Capture markers with Zivid's native 3D detector."""
    deadline = time.monotonic() + max(0.0, timeout)
    logger = node.get_logger()

    remaining = max(0.0, deadline - time.monotonic())
    if remaining <= 0:
        logger.warning("Zivid detection timed out")
        return DetectionResult(markers=[], capture_stamp=None)
    if not client.wait_for_service(timeout_sec=min(2.0, remaining)):
        logger.error(f"Zivid service '{SERVICE_NAME}' not available")
        return DetectionResult(markers=[], capture_stamp=None)

    if settle_time > 0:
        logger.debug(f"Settling for {settle_time:.2f}s before capture...")
        time.sleep(min(settle_time, max(0.0, deadline - time.monotonic())))
    if time.monotonic() >= deadline:
        logger.warning("Zivid detection timed out before capture")
        return DetectionResult(markers=[], capture_stamp=None)

    # The driver timestamps after processing; stamp before capture for accurate TF.
    pre_capture_stamp = node.get_clock().now().to_msg()
    logger.debug(
        f"Pre-capture timestamp: {pre_capture_stamp.sec}.{pre_capture_stamp.nanosec}"
    )

    # Zivid rejects an empty ID list; expand None to the dictionary's full range.
    request = CaptureAndDetectMarkers.Request()
    if marker_ids is not None:
        request.marker_ids = marker_ids
    else:
        if "4x4_50" in dictionary:
            request.marker_ids = list(range(50))
        elif "5x5_100" in dictionary:
            request.marker_ids = list(range(100))
        elif "5x5_250" in dictionary:
            request.marker_ids = list(range(250))
        elif "6x6_250" in dictionary:
            request.marker_ids = list(range(250))
        else:
            request.marker_ids = list(range(50))
        logger.debug(f"Detecting all markers (IDs 0-{len(request.marker_ids) - 1})")
    request.marker_dictionary = dictionary

    try:
        future = client.call_async(request)
        if not _wait_for_future(future, max(0.0, deadline - time.monotonic())):
            logger.warning("Zivid detection service timeout")
            return DetectionResult(markers=[], capture_stamp=None)
        result = future.result()
    except Exception as exc:
        logger.warning(f"Zivid detection service failed: {exc}")
        return DetectionResult(markers=[], capture_stamp=None)

    if not result.success:
        logger.warning(f"Zivid detection failed: {result.message}")
        return DetectionResult(markers=[], capture_stamp=None)

    detected = []
    for marker in result.detection_result.detected_markers:
        if marker_ids is None or marker.id in marker_ids:
            detected.append((marker.id, marker.pose))
            pos = marker.pose.position
            logger.info(
                f"Marker {marker.id}: Zivid native → "
                f"({pos.x * 1000:.2f}, {pos.y * 1000:.2f}, "
                f"{pos.z * 1000:.2f}) mm"
            )

    if not detected:
        if marker_ids is None:
            logger.warning("No markers detected")
        else:
            logger.warning(f"No markers found for requested IDs: {marker_ids}")

    return DetectionResult(markers=detected, capture_stamp=pre_capture_stamp)


def detect_sample_roi(
    node: Node,
    tag_id: int,
    strategy: str = "farthest_edge",
    edge_inset_mm: float = 0.0,
    dictionary: str = "aruco4x4_50",
    timeout: float = 45.0,
    params: SampleRoiDetectionParams | None = None,
) -> tuple[Pose, TimeMsg] | None:
    """Return a marker-anchored sample pickup pose and capture timestamp."""
    deadline = time.monotonic() + max(0.0, timeout)
    if params is None:
        params = SampleRoiDetectionParams()

    logger = node.get_logger()
    bridge = CvBridge()

    received_image: list[Image | None] = [None]

    def on_image(msg: Image):
        received_image[0] = msg

    image_sub = None
    marker_client = None

    try:
        image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, _ZIVID_QOS)
        marker_client = node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)

        remaining = max(0.0, deadline - time.monotonic())
        if remaining <= 0:
            logger.error("Zivid sample ROI detection timed out")
            return None
        if not marker_client.wait_for_service(timeout_sec=min(2.0, remaining)):
            logger.error(f"Zivid service '{SERVICE_NAME}' not available")
            return None

        # Let the temporary subscription match before capture.
        for _ in range(20):
            remaining = max(0.0, deadline - time.monotonic())
            if remaining <= 0:
                logger.error("Zivid sample ROI detection timed out")
                return None
            time.sleep(min(0.1, remaining))
            if received_image[0] is not None:
                break

        if time.monotonic() >= deadline:
            logger.error("Zivid sample ROI detection timed out")
            return None

        # Discard frames received before capture.
        received_image[0] = None

        # The driver timestamps after processing; stamp before capture for TF.
        pre_capture_stamp = node.get_clock().now().to_msg()

        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [tag_id]
        request.marker_dictionary = dictionary

        logger.info(
            f"Triggering Zivid capture for sample_roi detection (tag {tag_id})..."
        )
        try:
            future = marker_client.call_async(request)
            if not _wait_for_future(future, max(0.0, deadline - time.monotonic())):
                logger.error(f"Zivid service timeout after {timeout}s")
                return None
            result = future.result()
        except Exception as exc:
            logger.error(f"Zivid detection service failed: {exc}")
            return None

        if not result.success:
            logger.warning(f"Zivid detection failed: {result.message}")
            return None

        target_marker = None
        for marker in result.detection_result.detected_markers:
            if marker.id == tag_id:
                target_marker = marker
                break

        if target_marker is None:
            logger.warning(f"Tag {tag_id} not detected in image")
            return None

        marker_corners = np.array(
            [[p.x, p.y] for p in target_marker.corners_in_pixel_coordinates],
            dtype=float,
        )
        if marker_corners.shape != (4, 2) or not np.isfinite(marker_corners).all():
            logger.warning(f"Tag {tag_id} returned invalid pixel geometry")
            return None

        side_lengths = [
            np.linalg.norm(marker_corners[(i + 1) % 4] - marker_corners[i])
            for i in range(4)
        ]
        px_per_mm = np.mean(side_lengths) / params.marker_size_mm

        logger.info("Waiting for image data...")
        image_deadline = min(deadline, time.monotonic() + 20.0)
        while received_image[0] is None and time.monotonic() < image_deadline:
            time.sleep(min(0.1, max(0.0, image_deadline - time.monotonic())))

        if received_image[0] is None:
            logger.error("No image received after Zivid capture")
            return None

        bgr = bridge.imgmsg_to_cv2(received_image[0], desired_encoding="bgr8")

        detection = detect_sample_in_roi(
            bgr,
            marker_corners,
            px_per_mm,
            strategy=strategy,
            edge_inset_mm=edge_inset_mm,
            params=params,
        )

        if detection is None:
            logger.warning(f"No sample found in ROI near tag {tag_id}")
            return None

        pickup_px = detection["pickup_px"]
        logger.info(
            f"Sample detected: pickup=({pickup_px[0]}, {pickup_px[1]}), "
            f"size={detection['sample_size_mm']}mm, strategy={strategy}"
        )

        # Marker-plane projection avoids unstable depth on low-reflectivity wafers.
        mp = target_marker.pose
        pickup_xyz = sample_roi_pickup_camera_xyz(
            pickup_px,
            marker_corners,
            (mp.position.x, mp.position.y, mp.position.z),
            (mp.orientation.x, mp.orientation.y, mp.orientation.z, mp.orientation.w),
            px_per_mm,
            sample_thickness_mm=params.sample_thickness_mm,
        )
        if pickup_xyz is None:
            logger.error("Failed to compute pickup point from marker pose")
            return None

        pose = Pose()
        pose.position.x = pickup_xyz[0]
        pose.position.y = pickup_xyz[1]
        pose.position.z = pickup_xyz[2]
        pose.orientation.w = 1.0

        logger.info(
            f"Sample ROI pickup pose: ({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, {pose.position.z:.4f}) m in camera frame"
        )

        return pose, pre_capture_stamp

    finally:
        if image_sub is not None:
            node.destroy_subscription(image_sub)
        if marker_client is not None:
            node.destroy_client(marker_client)
