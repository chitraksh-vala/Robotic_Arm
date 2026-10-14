"""Detect tabletop objects and emit standalone XYZ coordinate records as JSON."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2

from object_locator.detector import BackgroundObjectDetector
from object_locator.io import load_config, open_camera
from object_locator.workspace import Workspace


def draw_overlay(frame, workspace: Workspace, objects) -> None:
    cv2.polylines(frame, [workspace.image_corners_px.astype("int32")], True, (0, 220, 0), 2)
    for detected in objects:
        cv2.drawContours(frame, [detected.contour], -1, (0, 180, 255), 2)
        center = tuple(int(value) for value in detected.centroid_px)
        label = f"{detected.id}: ({detected.x_mm:.1f}, {detected.y_mm:.1f}, {detected.z_mm:.1f}) mm"
        cv2.circle(frame, center, 4, (0, 0, 255), -1)
        cv2.putText(frame, label, (center[0] + 8, center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Locate objects in a calibrated overhead-camera workspace.")
    parser.add_argument("--config", default="configs/workspace.json")
    parser.add_argument("--background", default="data/empty_workspace.png")
    parser.add_argument("--camera", help="Camera index, file, or stream URL. Overrides config.")
    parser.add_argument("--headless", action="store_true", help="Do not show OpenCV windows.")
    parser.add_argument("--emit-interval", type=float, default=0.5,
                        help="Minimum seconds between JSON output records.")
    args = parser.parse_args()

    config = load_config(args.config)
    workspace = Workspace.from_config(config)
    background = cv2.imread(str(Path(args.background)), cv2.IMREAD_COLOR)
    detector = BackgroundObjectDetector(background, config.get("detection", {}))
    source = args.camera or str(config.get("camera", {}).get("source", "0"))
    camera = open_camera(source)
    last_emit = 0.0
    print("Locating objects. JSON records are written to stdout. Press q in the preview to quit.")
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Could not read a camera frame.")
            objects, mask = detector.detect(frame, workspace)
            now = time.monotonic()
            if objects and now - last_emit >= args.emit_interval:
                print(json.dumps({"objects": [item.as_output() for item in objects]}), flush=True)
                last_emit = now
            if not args.headless:
                overlay = frame.copy()
                draw_overlay(overlay, workspace, objects)
                cv2.imshow("Object locator", overlay)
                cv2.imshow("Object mask", mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    return
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
