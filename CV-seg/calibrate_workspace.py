"""Record image coordinates for the four measured workspace corners."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from object_locator.io import load_config, open_camera, save_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate an overhead-camera workspace.")
    parser.add_argument("--config", default="configs/workspace.json")
    parser.add_argument("--camera", help="Camera index, file, or stream URL. Overrides config.")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)
    source = args.camera or str(config.get("camera", {}).get("source", "0"))
    camera = open_camera(source)
    points: list[list[int]] = []

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append([x, y])
            print(f"Corner {len(points)}: ({x}, {y})")

    window = "Workspace calibration"
    try:
        cv2.namedWindow(window)
    except cv2.error as error:
        raise RuntimeError(
            "OpenCV cannot open GUI windows. Remove opencv-python-headless and "
            "install the desktop build with: "
            "python -m pip uninstall -y opencv-python-headless; "
            "python -m pip install --force-reinstall opencv-python"
        ) from error
    cv2.setMouseCallback(window, on_mouse)
    print("Click workspace corners: top-left, top-right, bottom-right, bottom-left.")
    print("The matching world_corners_mm in the config defines X/Y axes and origin.")
    print("Keys: r reset, s save, q quit")
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Could not read a camera frame.")
            display = frame.copy()
            if points:
                polygon = np.asarray(points, dtype=np.int32)
                cv2.polylines(display, [polygon], len(points) == 4, (0, 255, 0), 2)
            for index, point in enumerate(points, start=1):
                cv2.circle(display, tuple(point), 5, (0, 255, 0), -1)
                cv2.putText(display, str(index), (point[0] + 8, point[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                points.clear()
            elif key == ord("s"):
                if len(points) != 4:
                    print("Four corners are required before saving.")
                    continue
                config["camera"]["source"] = source
                config["workspace"]["image_corners_px"] = points
                save_config(config_path, config)
                print(f"Saved calibration to {config_path}")
                return
            elif key == ord("q"):
                return
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
