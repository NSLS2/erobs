"""Camera view panel: live image display, ArUco detection overlay."""

import cv2
import numpy as np

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit,
)
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt


class CameraPanel(QWidget):
    """Camera display with detection controls."""

    def __init__(self, ros2_bridge, parent=None):
        super().__init__(parent)
        self.ros2 = ros2_bridge
        self.current_image = None
        self.current_detections = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Buttons
        btn_row = QHBoxLayout()
        capture_btn = QPushButton("Capture")
        capture_btn.clicked.connect(self.ros2.trigger_capture)
        btn_row.addWidget(capture_btn)

        detect_btn = QPushButton("Detect Markers")
        detect_btn.clicked.connect(self.ros2.trigger_marker_detection)
        btn_row.addWidget(detect_btn)

        self.status_label = QLabel("No camera feed")
        self.status_label.setStyleSheet("color: #6B7385; font-size: 11px;")
        btn_row.addWidget(self.status_label)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        # Image display
        self.image_label = QLabel("No image")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(400, 300)
        self.image_label.setStyleSheet(
            "background-color: #0B0F18; color: #6B7385;"
            " border: 1px solid #2C3448; border-radius: 6px;"
        )
        layout.addWidget(self.image_label, stretch=1)

        # Detection info
        self.detection_info = QTextEdit()
        self.detection_info.setReadOnly(True)
        self.detection_info.setMaximumHeight(120)
        self.detection_info.setPlainText("No detections yet")
        layout.addWidget(self.detection_info)

        # Connect signals
        self.ros2.image_received.connect(self.on_image)
        self.ros2.detection_received.connect(self.on_detection)

    def on_image(self, cv_image):
        """Handle incoming camera image (called via signal on UI thread)."""
        self.current_image = cv_image
        self._display_image(cv_image)
        self.status_label.setText("Image received")
        self.status_label.setStyleSheet("color: #3DD68C; font-size: 11px; font-weight: 600;")

    def on_detection(self, markers):
        """Handle ArUco marker detection results."""
        self.current_detections = markers
        if not markers:
            self.status_label.setText("No markers detected")
            self.status_label.setStyleSheet("color: #F2B33D; font-size: 11px; font-weight: 600;")
            self.detection_info.setPlainText("No markers detected")
            return

        # Draw overlays on current image
        if self.current_image is not None:
            display = self.current_image.copy()
            for marker in markers:
                corners = marker.corners_in_pixel_coordinates
                if len(corners) == 4:
                    pts = np.array([[int(c.x), int(c.y)] for c in corners], np.int32)
                    cv2.polylines(display, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 3)
                    cx = int(sum(c.x for c in corners) / 4)
                    cy = int(sum(c.y for c in corners) / 4)
                    text = f"ID: {marker.id}"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    (tw, th), _ = cv2.getTextSize(text, font, 1.5, 3)
                    cv2.rectangle(display, (cx - 10, cy - th - 10), (cx + tw + 10, cy + 10), (0, 255, 0), -1)
                    cv2.putText(display, text, (cx, cy), font, 1.5, (0, 0, 0), 3)
            self._display_image(display)

        self.status_label.setText(f"{len(markers)} marker(s) detected")
        self.status_label.setStyleSheet("color: #3DD68C; font-size: 11px; font-weight: 600;")

        # Update info panel
        lines = [f"Detected {len(markers)} ArUco marker(s):\n"]
        for m in markers:
            pos = m.pose.position
            lines.append(f"ID {m.id}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m")
        self.detection_info.setPlainText("\n".join(lines))

    def _display_image(self, cv_image):
        """Convert OpenCV image to QPixmap and display."""
        h, w, ch = cv_image.shape
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(pixmap)
