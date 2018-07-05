import numpy as np
import cv2
from boltons.cacheutils import cachedproperty

class Trajectory:
    """
    Use a FrameInfo object to plan a local trajectory.
    """

    SLOW_SPEED = 1.0
    FAST_SPEED = 1.4

    LINE_DESIRED_DIST = 0.4

    R_MIN = 0.1

    CURVE_WEIGHT = 1.5

    def __init__(self, frameobjects):
        """
        :param frameobjects: The FrameObjects thing.
        """
        
        self.frameobjects = frameobjects


    @cachedproperty
    def immediate_path(self):

        if self.frameobjects.target_line is None:
            # If we have no line, just go forward.
            return (1000., Trajectory.SLOW_SPEED) 

        v, xint = self.frameobjects.target_line
        n, k = self.frameobjects.target_line_nk

        # Project the origin onto the target line and move upwards
        z = k * n + 0.4 * v
        z = z / np.linalg.norm(z)

        # TODO: generate (vl, vr) instead of (r, v)


        # fact = self.CURVE_WEIGHT * np.arccos(np.dot(z, [0,1])) * np.sign(n.dot([0,1]))
        # fact = self.CURVE_WEIGHT * np.cross(z, [0,1])# * np.sign(n.dot([0,1]))
        fact = Trajectory.CURVE_WEIGHT * v.dot([1,0])
        print(fact)
        r = (1 / (fact + 2e-4)) * Trajectory.R_MIN

        v = Trajectory.SLOW_SPEED

        return (r,v)

    
    @cachedproperty
    def immediate_path_old(self):

        # Is this the right-hand line?
        line_right = False

        # Work with either the left or right line.
        # TODO: take the closer one if we have both.
        line = None
        if self.frameobjects.left_line is not None:
            line = self.frameobjects.left_line
        elif self.frameobjects.right_line is not None:
            line = self.frameobjects.right_line
            line_right = True

        # If we have no lines to work with, just try to go forward (slowly)
        if line is None:
            return (1000, Trajectory.SLOW_SPEED)

        # Extract normal and k (how close the line is).
        n, k = line

        # Positive => turn away from line.
        # Negative => turn toward line.
        fact = self.CURVE_WEIGHT * n.dot([0,1])
        
        r = (1 / (fact + 1e-4)) * Trajectory.R_MIN

        # Flip direction for the right hand line.
        # TODO: make the signs cleaner here
        if line_right: r = -r

        # TODO: dynamic speed choice, just go slow for now
        v = Trajectory.SLOW_SPEED

        return (r, v)
