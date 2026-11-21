#!/usr/bin/env python3
"""Select an image pixel for the MCP get_point_3d tool using Tkinter and Pillow.

Usage:
    python3 point_selector_gui.py <image_path> [--x 500] [--y 300] [--title "Point Selector"]

One JSON line on stdout, in original image coordinates:
    Confirmed: {"pixel_x": 485, "pixel_y": 412, "confirmed": true}
    Cancelled: {"confirmed": false}
"""

import argparse
import json
import os
import subprocess as _sp
import sys
import tkinter as tk
from pathlib import Path

from PIL import Image, ImageDraw, ImageTk


def _ensure_display():
    """Fill missing DISPLAY and XAUTHORITY for X11 access."""
    if not os.environ.get("DISPLAY"):
        try:
            result = _sp.run(
                ["bash", "-c",
                 "cat /proc/$(pgrep -u $USER -x gnome-shell || "
                 "pgrep -u $USER -x Xwayland || echo 1)/environ "
                 "2>/dev/null | tr '\\0' '\\n' | grep ^DISPLAY= | "
                 "head -1 | cut -d= -f2"],
                capture_output=True, text=True, timeout=3,
            )
            os.environ["DISPLAY"] = result.stdout.strip() or ":1"
        except Exception:
            os.environ["DISPLAY"] = ":1"

    if not os.environ.get("XAUTHORITY"):
        for candidate in [
            Path(f"/run/user/{os.getuid()}/gdm/Xauthority"),
            Path.home() / ".Xauthority",
        ]:
            if candidate.exists():
                os.environ["XAUTHORITY"] = str(candidate)
                break


class PointSelector:
    """Display an image and confirm or cancel a pixel selection."""

    CROSSHAIR_SIZE = 20
    MAX_WINDOW_W = 1280
    MAX_WINDOW_H = 800

    def __init__(
        self,
        image_path: str,
        initial_point: tuple[int, int] | None = None,
        title: str = "Point Selector",
    ):
        self.pil_image = Image.open(image_path)
        self.img_w, self.img_h = self.pil_image.size
        self.title = title
        # Selection uses original image pixels.
        self.current_point = initial_point
        self.result: dict | None = None

        # Fit the image without upscaling.
        self.scale = min(
            self.MAX_WINDOW_W / self.img_w,
            self.MAX_WINDOW_H / self.img_h,
            1.0,
        )
        self.disp_w = int(self.img_w * self.scale)
        self.disp_h = int(self.img_h * self.scale)

        self._base_display = self.pil_image.resize(
            (self.disp_w, self.disp_h), Image.LANCZOS,
        )

    def _draw_crosshair(self) -> ImageTk.PhotoImage:
        """Render the selection overlay on the scaled image."""
        img = self._base_display.copy()
        if self.current_point is not None:
            draw = ImageDraw.Draw(img)
            # Convert selection coordinates to display pixels.
            dx = int(self.current_point[0] * self.scale)
            dy = int(self.current_point[1] * self.scale)
            s = self.CROSSHAIR_SIZE

            draw.line([(dx - s, dy), (dx + s, dy)], fill="red", width=2)
            draw.line([(dx, dy - s), (dx, dy + s)], fill="red", width=2)

            label = f"({self.current_point[0]}, {self.current_point[1]})"
            draw.text((dx + 12, dy - 18), label, fill="white")
            draw.text((dx + 11, dy - 19), label, fill="black")

        return ImageTk.PhotoImage(img)

    def run(self) -> dict:
        """Return confirmed coordinates or a cancellation result."""
        root = tk.Tk()
        root.title(self.title)
        root.configure(bg="#282828")

        banner = tk.Label(
            root,
            text="Click to select point.  Enter/Space = confirm,  Esc = cancel",
            bg="#282828",
            fg="#cccccc",
            font=("monospace", 11),
            pady=8,
        )
        banner.pack(fill=tk.X)

        canvas = tk.Canvas(
            root, width=self.disp_w, height=self.disp_h,
            highlightthickness=0, bg="black",
        )
        canvas.pack()

        # Keep the PhotoImage alive for Tk.
        self._tk_photo = self._draw_crosshair()
        canvas_image = canvas.create_image(0, 0, anchor=tk.NW, image=self._tk_photo)

        def _update_display():
            self._tk_photo = self._draw_crosshair()
            canvas.itemconfig(canvas_image, image=self._tk_photo)

        def _on_click(event):
            # Map clicks back to original image pixels and clamp to bounds.
            orig_x = int(event.x / self.scale)
            orig_y = int(event.y / self.scale)
            orig_x = max(0, min(orig_x, self.img_w - 1))
            orig_y = max(0, min(orig_y, self.img_h - 1))
            self.current_point = (orig_x, orig_y)
            _update_display()

        def _on_confirm(event=None):
            if self.current_point is not None:
                self.result = {
                    "pixel_x": self.current_point[0],
                    "pixel_y": self.current_point[1],
                    "confirmed": True,
                }
                root.destroy()

        def _on_cancel(event=None):
            self.result = {"confirmed": False}
            root.destroy()

        canvas.bind("<Button-1>", _on_click)
        root.bind("<Return>", _on_confirm)
        root.bind("<space>", _on_confirm)
        root.bind("<Escape>", _on_cancel)
        root.protocol("WM_DELETE_WINDOW", _on_cancel)

        # Center the window.
        root.update_idletasks()
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        x = (sw - self.disp_w) // 2
        y = (sh - self.disp_h) // 2
        root.geometry(f"+{x}+{y}")

        root.mainloop()
        return self.result or {"confirmed": False}


def main():
    _ensure_display()
    parser = argparse.ArgumentParser(description="Point selector GUI")
    parser.add_argument("image_path", help="Path to the image file")
    parser.add_argument("--x", type=int, default=None, help="Initial X coordinate")
    parser.add_argument("--y", type=int, default=None, help="Initial Y coordinate")
    parser.add_argument("--title", default="Point Selector", help="Window title")
    args = parser.parse_args()

    initial_point = None
    if args.x is not None and args.y is not None:
        initial_point = (args.x, args.y)

    try:
        selector = PointSelector(args.image_path, initial_point, args.title)
        result = selector.run()
    except Exception as e:
        result = {"confirmed": False, "error": str(e)}
        print(json.dumps(result))
        sys.exit(1)

    print(json.dumps(result))


if __name__ == "__main__":
    main()
