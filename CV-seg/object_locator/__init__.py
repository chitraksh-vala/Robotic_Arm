"""Standalone overhead-camera object localization utilities."""

from .detector import BackgroundObjectDetector, DetectedObject
from .workspace import Workspace

__all__ = ["BackgroundObjectDetector", "DetectedObject", "Workspace"]
