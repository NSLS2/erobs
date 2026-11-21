"""Detect spincoater pockets and samples in BGR images.

Use 2D flash capture without the 3D projector; keep the chuck centered.
Positions and sizes are in pixels; angle_mod90 folds degrees into [0, 90).
"""

import logging
import threading

import cv2
import numpy as np

logger = logging.getLogger(__name__)

_sample_model = None
_sample_model_lock = threading.Lock()


def _red_mask(hsv: np.ndarray) -> np.ndarray:
    """Combine red hue ranges across OpenCV's 0/179 boundary."""
    m1 = cv2.inRange(hsv, (0, 60, 40), (14, 255, 255))
    m2 = cv2.inRange(hsv, (166, 60, 40), (179, 255, 255))
    return cv2.bitwise_or(m1, m2)


def _locate_chuck(image: np.ndarray) -> tuple[int, int, int] | None:
    """Return the chuck center and ROI half-size in pixels, or None."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = _red_mask(hsv)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    r = int(np.sqrt(cv2.contourArea(c) / np.pi))
    return cx, cy, max(120, int(r * 1.4))


def detect_spincoater_pocket(
    image: np.ndarray,
    min_area: int = 1200,
    max_aspect: float = 1.25,
    min_solidity: float = 0.85,
) -> dict | None:
    """Return empty-pocket geometry from the red chuck, or None."""
    loc = _locate_chuck(image)
    if loc is None:
        return None
    cx, cy, half = loc

    x0 = max(0, cx - half)
    y0 = max(0, cy - half)
    roi = image[y0:y0 + 2 * half, x0:x0 + 2 * half]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red = _red_mask(hsv)
    red_closed = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    cnts, _ = cv2.findContours(
        red_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not cnts:
        return None

    field = max(cnts, key=cv2.contourArea)
    field_mask = np.zeros_like(red)
    cv2.drawContours(field_mask, [field], -1, 255, -1)

    # Isolate bright, low-saturation metal within the red chuck.
    bright = cv2.inRange(hsv, (0, 0, 150), (179, 90, 255))
    bright = cv2.bitwise_and(bright, field_mask)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    cnts, _ = cv2.findContours(
        bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        (rx, ry), (rw, rh), ang = cv2.minAreaRect(c)
        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)
        solidity = area / (cv2.contourArea(cv2.convexHull(c)) + 1e-6)
        if aspect < max_aspect and solidity > min_solidity:
            return {
                "center_px": (int(rx + x0), int(ry + y0)),
                "angle_mod90": ang % 90,
                "width": rw,
                "height": rh,
                "aspect": round(aspect, 3),
                "solidity": round(solidity, 3),
                "area": int(area),
            }
    return None


def _get_sample_model():
    """Load and cache the segmentation model under the initialization lock."""
    global _sample_model
    if _sample_model is not None:
        return _sample_model

    with _sample_model_lock:
        # Another thread may have loaded the model while this caller waited.
        if _sample_model is not None:
            return _sample_model

        try:
            from ultralytics import YOLO
        except ImportError:
            raise RuntimeError(
                "ultralytics not installed. Run: pip install ultralytics"
            )

        from ament_index_python.packages import get_package_share_path

        model_path = (
            get_package_share_path("beambot") / "models" / "spincoater_sample_seg.pt"
        )
        if not model_path.exists():
            raise FileNotFoundError(
                f"Spincoater sample model not found at {model_path}"
            )

        logger.info(f"Loading spincoater sample model: {model_path}")
        _sample_model = YOLO(str(model_path))
        return _sample_model


def detect_spincoater_sample(
    image: np.ndarray,
    confidence: float = 0.3,
) -> dict | None:
    """Return sample geometry and confidence from YOLO segmentation, or None."""
    model = _get_sample_model()
    results = model(image, conf=confidence, verbose=False)

    for r in results:
        if r.masks is None or len(r.masks) == 0:
            continue

        best_idx = r.boxes.conf.argmax()
        mask_xy = r.masks[best_idx].xy[0]
        conf = r.boxes[best_idx].conf[0].item()

        if len(mask_xy) < 4:
            continue

        pts = mask_xy.astype(np.float32)
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)

        # Use the mask centroid, falling back to the rectangle center.
        M = cv2.moments(pts.astype(np.int32))
        if M["m00"] > 0:
            centroid_x = M["m10"] / M["m00"]
            centroid_y = M["m01"] / M["m00"]
        else:
            centroid_x, centroid_y = cx, cy

        return {
            "center_px": (int(round(centroid_x)), int(round(centroid_y))),
            "angle_mod90": ang % 90,
            "angle_raw": ang,
            "width": rw,
            "height": rh,
            "aspect": round(aspect, 3),
            "confidence": round(conf, 3),
            "mask_points": len(mask_xy),
        }

    return None
