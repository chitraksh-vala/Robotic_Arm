"""
SO Arm Simulator Package
Simulation environment for SO arm control and training
"""

__version__ = "0.1.0"
__author__ = "Robotics Team"

from .controller import DualSenseController, ControllerMapper
from .recorder import MotionRecorder, MotionPlayer
from .simulator import ArmSimulator

__all__ = [
    'DualSenseController',
    'ControllerMapper',
    'MotionRecorder',
    'MotionPlayer',
    'ArmSimulator',
]
