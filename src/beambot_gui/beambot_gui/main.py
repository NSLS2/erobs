#!/usr/bin/env python3
"""MTC GUI Client — PyQt6 entry point."""

import sys

# Load OpenCV before PyQt6 to avoid Qt's bundled GLib conflicting with OpenCV.
try:
    import cv2  # noqa: F401
except Exception:
    pass

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

# Load QtWebEngine before creating QApplication.
try:
    from PyQt6 import QtWebEngineWidgets  # noqa: F401
except ImportError:
    pass

from .ros2_bridge import ROS2Bridge
from . import theme


def main():
    # QtWebEngine requires shared OpenGL contexts before QApplication creation.
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)

    app = QApplication(sys.argv)
    theme.apply(app)

    ros2 = ROS2Bridge()
    try:
        ros2.init_ros2()

        from .main_window import BeambotMainWindow

        window = BeambotMainWindow(ros2)
        window.show()
        ret = app.exec()
    finally:
        ros2.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
