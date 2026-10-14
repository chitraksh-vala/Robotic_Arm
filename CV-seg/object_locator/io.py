"""Configuration, camera, and image helpers shared by the CLI tools."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as config_file:
        return json.load(config_file)


def save_config(path: str | Path, config: dict[str, Any]) -> None:
    with Path(path).open("w", encoding="utf-8") as config_file:
        json.dump(config, config_file, indent=2)
        config_file.write("\n")


def parse_camera_source(source: str) -> int | str:
    return int(source) if source.lstrip("-").isdigit() else source


def open_camera(source: str) -> cv2.VideoCapture:
    camera = cv2.VideoCapture(parse_camera_source(source))
    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera source: {source}")
    return camera


def capture_median_frame(camera: cv2.VideoCapture, frame_count: int) -> np.ndarray:
    frames: list[np.ndarray] = []
    while len(frames) < frame_count:
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError("Camera stopped while capturing the background.")
        frames.append(frame)
    return np.median(np.stack(frames, axis=0), axis=0).astype(np.uint8)
