"""Background-based object segmentation for a fixed tabletop workspace."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import cv2
import numpy as np

from .workspace import Workspace


@dataclass(frozen=True)
class DetectedObject:
    """A detected item expressed in the future arm coordinate convention."""

    id: int
    x_mm: float
    y_mm: float
    z_mm: float
    centroid_px: tuple[float, float]
    area_px: float
    contour: np.ndarray

    def as_output(self) -> dict[str, Any]:
        result = asdict(self)
        result.pop("contour")
        result["centroid_px"] = [round(v, 1) for v in self.centroid_px]
        result["x_mm"] = round(self.x_mm, 2)
        result["y_mm"] = round(self.y_mm, 2)
        result["z_mm"] = round(self.z_mm, 2)
        result["area_px"] = round(self.area_px, 1)
        result["z_source"] = "workspace_pick_plane"
        return result


class BackgroundObjectDetector:
    """Segments changes from an empty-workspace reference image."""

    def __init__(self, background_bgr: np.ndarray, settings: dict[str, Any]) -> None:
        if background_bgr is None or background_bgr.size == 0:
            raise ValueError("A valid empty-workspace background image is required.")
        self.background_bgr = background_bgr
        self.background_lab = cv2.cvtColor(background_bgr, cv2.COLOR_BGR2LAB)
        self.threshold = float(settings.get("lab_threshold", 28))
        self.min_area_px = float(settings.get("min_area_px", 600))
        self.open_kernel_px = self._odd_kernel(settings.get("open_kernel_px", 3))
        self.close_kernel_px = self._odd_kernel(settings.get("close_kernel_px", 7))

    @staticmethod
    def _odd_kernel(value: Any) -> int:
        size = max(1, int(value))
        return size if size % 2 else size + 1

    def detect(self, frame_bgr: np.ndarray, workspace: Workspace) -> tuple[list[DetectedObject], np.ndarray]:
        if frame_bgr.shape[:2] != self.background_bgr.shape[:2]:
            raise ValueError(
                "Camera resolution changed from the captured background. Capture a new background."
            )
        current_lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        delta = np.linalg.norm(
            current_lab.astype(np.float32) - self.background_lab.astype(np.float32), axis=2
        )
        mask = np.where(delta >= self.threshold, 255, 0).astype(np.uint8)
        mask = cv2.bitwise_and(mask, workspace.workspace_mask(frame_bgr.shape))

        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.open_kernel_px, self.open_kernel_px)
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.close_kernel_px, self.close_kernel_px)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects: list[DetectedObject] = []
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = float(cv2.contourArea(contour))
            if area < self.min_area_px:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            centroid = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
            x_mm, y_mm = workspace.image_to_world(centroid)
            objects.append(
                DetectedObject(
                    id=len(objects) + 1,
                    x_mm=x_mm,
                    y_mm=y_mm,
                    z_mm=workspace.z_mm,
                    centroid_px=centroid,
                    area_px=area,
                    contour=contour,
                )
            )
        return objects, mask
