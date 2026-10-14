"""Planar image-to-workspace coordinate conversion."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class Workspace:
    """A fixed tabletop workspace described by matching image/world corners.

    Corner order is clockwise: top-left, top-right, bottom-right, bottom-left
    as viewed in the camera image.  The world points may use any orientation,
    as long as they use the same physical corners and are measured in mm.
    """

    image_corners_px: np.ndarray
    world_corners_mm: np.ndarray
    z_mm: float

    def __post_init__(self) -> None:
        image = np.asarray(self.image_corners_px, dtype=np.float32)
        world = np.asarray(self.world_corners_mm, dtype=np.float32)
        if image.shape != (4, 2) or world.shape != (4, 2):
            raise ValueError("Workspace needs exactly four [x, y] image and world corners.")
        if abs(cv2.contourArea(image)) < 1.0 or abs(cv2.contourArea(world)) < 1.0:
            raise ValueError("Workspace corners must enclose a non-zero area.")
        object.__setattr__(self, "image_corners_px", image)
        object.__setattr__(self, "world_corners_mm", world)
        object.__setattr__(self, "_homography", cv2.getPerspectiveTransform(image, world))

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "Workspace":
        workspace = config["workspace"]
        image_corners = workspace.get("image_corners_px")
        if image_corners is None:
            raise ValueError(
                "Workspace is not calibrated. Run calibrate_workspace.py before locating objects."
            )
        return cls(
            image_corners_px=np.asarray(image_corners, dtype=np.float32),
            world_corners_mm=np.asarray(workspace["world_corners_mm"], dtype=np.float32),
            z_mm=float(workspace.get("pick_z_mm", 0.0)),
        )

    def image_to_world(self, point_px: tuple[float, float] | np.ndarray) -> tuple[float, float]:
        point = np.asarray(point_px, dtype=np.float32).reshape(1, 1, 2)
        world = cv2.perspectiveTransform(point, self._homography)[0, 0]
        return float(world[0]), float(world[1])

    def workspace_mask(self, frame_shape: tuple[int, ...]) -> np.ndarray:
        mask = np.zeros(frame_shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, self.image_corners_px.astype(np.int32), 255)
        return mask
