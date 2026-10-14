"""Capture an empty tabletop image for background-based object detection."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from object_locator.io import capture_median_frame, load_config, open_camera


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture an empty-workspace background image.")
    parser.add_argument("--config", default="configs/workspace.json")
    parser.add_argument("--output", default="data/empty_workspace.png")
    parser.add_argument("--frames", type=int, default=20, help="Frames used for a median image.")
    parser.add_argument("--camera", help="Camera index, file, or stream URL. Overrides config.")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be at least 1")

    config = load_config(args.config)
    source = args.camera or str(config.get("camera", {}).get("source", "0"))
    camera = open_camera(source)
    window = "Empty workspace preview"
    try:
        cv2.namedWindow(window)
    except cv2.error as error:
        camera.release()
        raise RuntimeError(
            "OpenCV cannot open GUI windows. Install the desktop OpenCV package "
            "with: python -m pip uninstall -y opencv-python-headless; "
            "python -m pip install --force-reinstall opencv-python"
        ) from error
    try:
        print("Check that this preview is the correct camera and that the workspace is empty.")
        print("Keys: c capture background, q quit")
        while True:
            ok, preview = camera.read()
            if not ok:
                raise RuntimeError("Could not read a camera frame.")
            mean_brightness = float(np.mean(cv2.cvtColor(preview, cv2.COLOR_BGR2GRAY)))
            cv2.putText(preview, f"Camera {source} | brightness {mean_brightness:.0f}", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(preview, "c: capture empty workspace   q: quit", (12, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                return
            if key == ord("c"):
                if mean_brightness < 8:
                    print("Frame is nearly black. Choose another --camera value and try again.")
                    continue
                background = capture_median_frame(camera, args.frames)
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), background):
        raise RuntimeError(f"Could not write background image: {output}")
    print(f"Saved empty-workspace background to {output}")


if __name__ == "__main__":
    main()
