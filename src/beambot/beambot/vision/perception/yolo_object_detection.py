"""YOLO object detection, lazy model loading and image annotation."""

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)

DEFAULT_MODEL = "yolov8n.pt"

# Fallback directory for custom weights.
MODEL_DIR = Path.home() / ".beambot" / "models"


@dataclass
class YoloDetectionParams:
    """Parameters for YOLO object detection."""
    model_path: str = DEFAULT_MODEL   # Model name or weights path.
    confidence: float = 0.25          # Minimum confidence (0-1).
    iou_threshold: float = 0.45       # Overlap threshold for suppressing duplicate boxes.
    classes: list[int] | None = None  # Class IDs; None includes all classes.
    max_detections: int = 50


# (class_name, confidence, center_x, center_y, x1, y1, x2, y2)
YoloDetection = tuple[str, float, int, int, int, int, int, int]


class YoloModel:
    """Object detector with a lazily loaded, cached YOLO model."""

    def __init__(self, model_path: str = DEFAULT_MODEL, device: str = ""):
        self._model = None
        self._model_path = model_path
        self._device = device

    def _ensure_model(self):
        """Load the selected model if it is not already cached."""
        if self._model is not None:
            return

        try:
            from ultralytics import YOLO
        except ImportError:
            raise RuntimeError(
                "ultralytics not installed. Run: pip install ultralytics"
            )

        model_path = Path(self._model_path)
        if not model_path.exists() and not self._model_path.startswith("yolo"):
            alt_path = MODEL_DIR / self._model_path
            if alt_path.exists():
                model_path = alt_path

        logger.info(f"Loading YOLO model: {model_path}")
        self._model = YOLO(str(model_path))

        if self._device:
            self._model.to(self._device)

        logger.info(
            f"YOLO model loaded: {self._model.model_name} "
            f"({len(self._model.names)} classes)"
        )

    def detect(
        self,
        image: np.ndarray,
        params: YoloDetectionParams | None = None,
    ) -> list[YoloDetection]:
        """Detect objects in a BGR image, sorted by descending confidence."""
        if params is None:
            params = YoloDetectionParams(model_path=self._model_path)

        if self._model_path != params.model_path:
            self._model = None
            self._model_path = params.model_path
        self._ensure_model()

        results = self._model(
            image,
            conf=params.confidence,
            iou=params.iou_threshold,
            classes=params.classes,
            max_det=params.max_detections,
            verbose=False,
        )

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue

            for box in boxes:
                # Convert box coordinates to Python integers for JSON output.
                coords = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self._model.names[cls_id]

                detections.append((cls_name, conf, cx, cy, x1, y1, x2, y2))

        detections.sort(key=lambda d: d[1], reverse=True)

        return detections

    def annotate(
        self,
        image: np.ndarray,
        detections: list[YoloDetection],
    ) -> np.ndarray:
        """Draw boxes, labels and centers on a copy of the BGR image."""
        annotated = image.copy()

        for i, (cls_name, conf, cx, cy, x1, y1, x2, y2) in enumerate(detections):
            color = (0, 255, 0)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            label = f"#{i+1} {cls_name} {conf:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(
                annotated,
                (x1, y1 - label_size[1] - 8),
                (x1 + label_size[0] + 4, y1),
                color, -1,
            )
            cv2.putText(
                annotated, label,
                (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2,
            )

            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

        return annotated


_shared_model: YoloModel | None = None


def get_yolo_model(model_path: str = DEFAULT_MODEL) -> YoloModel:
    """Return the shared detector for the requested model path."""
    global _shared_model
    if _shared_model is None or _shared_model._model_path != model_path:
        _shared_model = YoloModel(model_path=model_path)
    return _shared_model
