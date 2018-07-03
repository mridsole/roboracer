"""
Module for processing a single frame (color and depth buffer).
"""

from procframe.frameinfo import FrameInfo
from procframe.frameobjects import FrameObjects
from procframe.trajectory import TrajectoryPlanner

__all__ = [
    'FrameInfo',
    'FrameObjects',
    'TrajectoryPlanner'
]
