from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from object_locator.detector import BackgroundObjectDetector
from object_locator.workspace import Workspace


class WorkspaceTests(unittest.TestCase):
    def test_maps_a_perspective_point_to_world_coordinates(self) -> None:
        workspace = Workspace(
            image_corners_px=np.array([[20, 10], [220, 20], [210, 220], [10, 210]]),
            world_corners_mm=np.array([[0, 0], [200, 0], [200, 200], [0, 200]]),
            z_mm=12,
        )
        for image, expected in [((20, 10), (0, 0)), ((210, 220), (200, 200))]:
            actual = workspace.image_to_world(image)
            self.assertTrue(np.allclose(actual, expected, atol=0.01))

    def test_detects_an_object_and_reports_pick_plane_z(self) -> None:
        background = np.full((240, 320, 3), (80, 110, 140), dtype=np.uint8)
        frame = background.copy()
        cv2.rectangle(frame, (130, 80), (190, 140), (10, 10, 240), -1)
        workspace = Workspace(
            image_corners_px=np.array([[0, 0], [319, 0], [319, 239], [0, 239]]),
            world_corners_mm=np.array([[0, 0], [319, 0], [319, 239], [0, 239]]),
            z_mm=5,
        )
        detector = BackgroundObjectDetector(background, {"lab_threshold": 20, "min_area_px": 500})
        objects, _ = detector.detect(frame, workspace)
        self.assertEqual(len(objects), 1)
        self.assertAlmostEqual(objects[0].x_mm, 160, delta=2)
        self.assertAlmostEqual(objects[0].y_mm, 110, delta=2)
        self.assertEqual(objects[0].z_mm, 5)


if __name__ == "__main__":
    unittest.main()
