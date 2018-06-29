import numpy as np
import cv2

class TrajectoryPlanner:
    """
    Use a FrameInfo object to plan a trajectory:
    """

    def __init__(self, frameinfo):
        """
        :param frameinfo: The FrameInfo object constructed from color and 
            depth.
        """
        
        self.frameinfo = frameinfo
