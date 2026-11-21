"""Detection algorithms and image-to-3D geometry without ROS imports."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class SampleRoiDetectionParams:
    """Geometry, contour filters and calibration for marker-relative samples."""
    # ROI geometry relative to the marker center (mm).
    roi_offset_x_mm: float = 19.3
    roi_offset_y_mm: float = 0.3
    roi_width_mm: float = 22.1
    roi_height_mm: float = 21.8

    # Contour area (pixels squared) and aspect ratio limits.
    min_area: int = 100
    max_area: int = 15000
    max_aspect_ratio: float = 3.0

    # Physical marker side length for pixel-to-millimetre conversion.
    marker_size_mm: float = 14.9
    # Sample height above the marker plane (mm); zero means coplanar.
    sample_thickness_mm: float = 0.0


def detect_aruco_markers(
    rgb_image: np.ndarray,
    dictionary_id: int,
) -> tuple[tuple[np.ndarray, ...], np.ndarray | None]:
    """Return marker corners and IDs from an RGB image, supporting both OpenCV APIs."""
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(dictionary_id)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=aruco_params
        )
    return corners, ids


def get_3d_position(
    cloud,
    cx: int,
    cy: int,
    search_radius: int = 10,
) -> tuple[float, float, float] | None:
    """Return the first valid XYZ (metres) at or near a pixel, or None."""
    # Assumes image-aligned, little-endian XYZ float32 point data.
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step

    def get_xyz_at(u: int, v: int):
        if u < 0 or u >= width or v < 0 or v >= height:
            return None
        offset = v * cloud.row_step + u * point_step
        try:
            x, y, z = struct.unpack_from("<fff", cloud.data, offset)
        except struct.error:
            return None
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            return None
        if x == 0.0 and y == 0.0 and z == 0.0:
            return None
        return (x, y, z)

    xyz = get_xyz_at(cx, cy)
    if xyz is not None:
        return xyz

    for r in range(1, search_radius + 1):
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                if abs(du) == r or abs(dv) == r:
                    xyz = get_xyz_at(cx + du, cy + dv)
                    if xyz is not None:
                        return xyz
    return None


def detect_sample_in_roi(
    rgb_image: np.ndarray,
    marker_corners: np.ndarray,
    px_per_mm: float,
    strategy: str = "farthest_edge",
    edge_inset_mm: float = 0.0,
    params: SampleRoiDetectionParams | None = None,
) -> dict[str, Any] | None:
    """Return sample geometry and a pickup pixel from a marker-relative ROI, or None.

    Expects BGR/grayscale input, TL/TR/BR/BL marker corners and pixels per mm.
    """
    if params is None:
        params = SampleRoiDetectionParams()

    if strategy not in {
        "center",
        "farthest_edge",
        "nearest_edge",
        "farthest_corner",
        "nearest_corner",
    }:
        return None

    try:
        marker_corners = np.asarray(marker_corners, dtype=float)
        geometry = np.asarray(
            (
                px_per_mm,
                edge_inset_mm,
                params.roi_offset_x_mm,
                params.roi_offset_y_mm,
                params.roi_width_mm,
                params.roi_height_mm,
            ),
            dtype=float,
        )
    except (TypeError, ValueError, OverflowError):
        return None

    (
        px_per_mm,
        edge_inset_mm,
        roi_offset_x_mm,
        roi_offset_y_mm,
        roi_width_mm,
        roi_height_mm,
    ) = geometry
    if (
        marker_corners.shape != (4, 2)
        or not np.isfinite(marker_corners).all()
        or not np.isfinite(geometry).all()
        or px_per_mm <= 0
        or edge_inset_mm < 0
        or roi_width_mm <= 0
        or roi_height_mm <= 0
    ):
        return None

    # Use the marker's image axes to orient the ROI offset.
    top_left, top_right = marker_corners[0], marker_corners[1]
    bottom_left = marker_corners[3]
    marker_x = top_right - top_left
    marker_y = bottom_left - top_left
    marker_x_norm = np.linalg.norm(marker_x)
    marker_y_norm = np.linalg.norm(marker_y)
    if not np.isfinite((marker_x_norm, marker_y_norm)).all() or min(
        marker_x_norm, marker_y_norm
    ) <= 0:
        return None
    marker_x = marker_x / marker_x_norm
    marker_y = marker_y / marker_y_norm
    tag_center = marker_corners.mean(axis=0)

    roi_center = (
        tag_center
        + marker_x * (roi_offset_x_mm * px_per_mm)
        + marker_y * (roi_offset_y_mm * px_per_mm)
    )

    half_w = (roi_width_mm * px_per_mm) / 2
    half_h = (roi_height_mm * px_per_mm) / 2
    if not np.isfinite((*roi_center, half_w, half_h)).all():
        return None

    h, w = rgb_image.shape[:2]
    roi_x1 = max(0, int(roi_center[0] - half_w))
    roi_y1 = max(0, int(roi_center[1] - half_h))
    roi_x2 = min(w, int(roi_center[0] + half_w))
    roi_y2 = min(h, int(roi_center[1] + half_h))

    roi = rgb_image[roi_y1:roi_y2, roi_x1:roi_x2]
    if roi.size == 0:
        return None

    if len(roi.shape) == 3:
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        roi_gray = roi

    # Find sample contours within the ROI.
    blurred = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < params.min_area or area > params.max_area:
            continue
        rect = cv2.minAreaRect(c)
        rw, rh = rect[1]
        if rw == 0 or rh == 0:
            continue
        if max(rw, rh) / min(rw, rh) > params.max_aspect_ratio:
            continue
        valid.append((area, c))

    if not valid:
        return None

    valid.sort(key=lambda x: x[0], reverse=True)
    sample_contour = valid[0][1]

    rect = cv2.minAreaRect(sample_contour)
    rect_center_roi = rect[0]
    rect_size = rect[1]
    rect_angle = rect[2]
    rect_corners_roi = cv2.boxPoints(rect)

    # Convert ROI-local coordinates to full-image pixels.
    rect_center_full = np.array([
        rect_center_roi[0] + roi_x1,
        rect_center_roi[1] + roi_y1,
    ])
    rect_corners_full = rect_corners_roi + np.array([roi_x1, roi_y1])

    tag_pt = tag_center
    center_pt = rect_center_full.copy()
    edge_inset_px = edge_inset_mm * px_per_mm

    if strategy == "center":
        pickup = center_pt.copy()
    else:
        if "corner" in strategy:
            candidates = rect_corners_full
        else:
            candidates = [
                (rect_corners_full[i] + rect_corners_full[(i + 1) % 4]) / 2
                for i in range(4)
            ]
        distances = [np.linalg.norm(point - tag_pt) for point in candidates]
        idx = np.argmax(distances) if "farthest" in strategy else np.argmin(distances)
        pickup = candidates[idx].copy()
        if edge_inset_px > 0:
            toward = center_pt - pickup
            norm = np.linalg.norm(toward)
            if norm > 0:
                pickup = pickup + toward / norm * edge_inset_px

    offset_from_center_mm = np.linalg.norm(pickup - center_pt) / px_per_mm

    return {
        "pickup_px": (int(round(pickup[0])), int(round(pickup[1]))),
        "center_px": (int(round(center_pt[0])), int(round(center_pt[1]))),
        "sample_size_mm": (
            round(rect_size[0] / px_per_mm, 1),
            round(rect_size[1] / px_per_mm, 1),
        ),
        "sample_angle": round(rect_angle, 1),
        "sample_area_px": int(cv2.contourArea(sample_contour)),
        "offset_from_center_mm": round(offset_from_center_mm, 1),
        "roi": (roi_x1, roi_y1, roi_x2, roi_y2),
        "strategy": strategy,
        "edge_inset_mm": edge_inset_mm,
    }


def sample_roi_pickup_camera_xyz(
    pickup_px: tuple[float, float],
    marker_corners: np.ndarray,
    marker_position: tuple[float, float, float],
    marker_quat_xyzw: tuple[float, float, float, float],
    px_per_mm: float,
    sample_thickness_mm: float = 0.0,
) -> tuple[float, float, float] | None:
    """Project a pickup pixel into camera XYZ (metres), or return None.

    Uses TL/TR/BR/BL corners and an XYZW marker quaternion, without sample depth.
    Assumes a near-top-down view; marker_position is in metres.
    """
    corners = np.asarray(marker_corners, dtype=float)
    pickup = np.asarray(pickup_px, dtype=float)
    if (
        corners.shape != (4, 2)
        or not np.isfinite(corners).all()
        or not np.isfinite(pickup).all()
        or not np.isfinite(px_per_mm)
        or px_per_mm <= 0
    ):
        return None

    # Marker image axes: +X = TL to TR, +Y = TL to BL.
    top_left, top_right, bottom_left = corners[0], corners[1], corners[3]
    ax = top_right - top_left
    ay = bottom_left - top_left
    ax_n, ay_n = np.linalg.norm(ax), np.linalg.norm(ay)
    if min(ax_n, ay_n) <= 0:
        return None
    ax = ax / ax_n
    ay = ay / ay_n

    # Convert pixel offsets to marker-frame metres; sample height follows -Z.
    tag_center = corners.mean(axis=0)
    d = pickup - tag_center
    offset_marker_m = np.array([
        float(np.dot(d, ax)) / px_per_mm / 1000.0,
        float(np.dot(d, ay)) / px_per_mm / 1000.0,
        -sample_thickness_mm / 1000.0,
    ])

    qx, qy, qz, qw = (float(v) for v in marker_quat_xyzw)
    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
    if not np.isfinite(norm_sq) or norm_sq <= 0:
        return None
    s = 2.0 / norm_sq
    # Rotate from marker coordinates into the camera frame.
    rot = np.array([
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
    ])
    cam = rot @ offset_marker_m + np.asarray(marker_position, dtype=float)
    if not np.isfinite(cam).all():
        return None
    return (float(cam[0]), float(cam[1]), float(cam[2]))
