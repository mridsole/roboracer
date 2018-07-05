import numpy as np
import cv2
from boltons.cacheutils import cachedproperty

class Trajectory:
    """
    Use a FrameInfo object to plan a local trajectory.
    """

    SLOW_SPEED = 0.2
    FAST_SPEED = 1.4

    LEFT_TURN = (0.15, 0.65)
    RIGHT_TURN = (0.65, 0.15)

    LINE_DESIRED_DIST = 0.4

    R_MIN = 0.1

    CURVE_WEIGHT = 1.5

    def __init__(self, frameobjects):
        """
        :param frameobjects: The FrameObjects thing.
        """
        
        self.frameobjects = frameobjects


    @cachedproperty
    def avoid_line(self):
        """
        This is the _actual_ target line, taking into account
        a possibly visible obstacle.
        """

        # If we don't have an obstacle, just return the target line.
        if self.frameobjects.obstacle is None:
            return self.frameobjects.target_line_nk

        c_o, r_o = self.frameobjects.obstacle

        line_l = self.frameobjects.left_line
        line_r = self.frameobjects.right_line

        # If we have no lines, there's no avoid line.
        if line_l is None and line_r is None:
            return None

        # If we have two lines:
        if line_l is not None and line_r is not None:

            nl, kl = line_l
            nr, kr = line_r

            # Decide which line is closest to the obstacle.
            dl = np.abs(kl - nl.dot(c_o))
            dr = np.abs(kr - nr.dot(c_o))

            # Take the line that has the greater distance, and
            # follow that one - this is our target.
            is_left = dl > dr
            is_right = not is_left

            # Note: n points outwards.
            n, k = line_l if is_left else line_r

            if is_left:

                # Take the right-pointing normal.
                if n.dot([1,0]) < 0:
                    n = -n
                    k = -k

                # Project to the right.
                d_t = 0.4
                return (n, k + d_t)


            else:

                # Take the left-pointing normal.
                if n.dot([-1,0]) < 0:
                    n = -n
                    k = -k

                # Project to the left.
                d_t = 0.4
                return (n, k + d_t)


        # Otherewise, we only have one line: find this line.
        line = None
        is_left = None
        if line_l is not None:
            is_left = True
            line = line_l
        else:
            is_left = False
            line = line_r

        n, k = line
        if is_left:
            # Take the right-pointing normal.
            if n.dot([1,0]) < 0:
                n = -n
                k = -k
        else:
            # Take the left-pointing normal.
            if n.dot([-1,0]) < 0:
                n = -n
                k = -k
        
        d_o = np.abs(k - n.dot(c_o))
        if d_o < 0.7:
            d_t = d_o + r_o + 0.4
            # d_t = d_o # + 0.4
        else:
            d_t = 0.4

        return (n, k + d_t)


    @cachedproperty
    def immediate_path(self):

        if self.avoid_line is None:
            # If we have no line, just go forward.
            return ((Trajectory.SLOW_SPEED, Trajectory.SLOW_SPEED), 0)

        n, k = self.avoid_line
        v = np.array([-n[1], n[0]])
        if v.dot([0,1]) < 0: v = -v

        # Project the origin onto the target line and move upwards
        z = k * n + 0.5 * v
        z = z / np.linalg.norm(z)

        # TODO: generate (vl, vr) instead of (r, v)

        # This is correct (or is it??).
        fact = np.arcsin(np.dot([1,0], z))

        # TODO: What about these thresholds? This is about a 22 degree window
        # (which is reasonable.
        if fact > 0.2:
            return (Trajectory.RIGHT_TURN, 1)
        elif fact < -0.2:
            return (Trajectory.LEFT_TURN, -1)
        
        # Otherwise, just go forward slowly.
        return (
            (Trajectory.SLOW_SPEED, Trajectory.SLOW_SPEED),
            0
        )

        # v_l = Trajectory.SLOW_SPEED * max(min(1.2, 1 + fact), 0.7)
        # v_r = Trajectory.SLOW_SPEED * max(min(1.2, 1 - fact), 0.7)
        # return (v_l, v_r)

        # # fact = self.CURVE_WEIGHT * np.arccos(np.dot(z, [0,1])) * np.sign(n.dot([0,1]))
        # # fact = self.CURVE_WEIGHT * np.cross(z, [0,1])# * np.sign(n.dot([0,1]))
        # fact = Trajectory.CURVE_WEIGHT * v.dot([1,0])
        # print(fact)
        # r = (1 / (fact + 2e-4)) * Trajectory.R_MIN

        # v = Trajectory.SLOW_SPEED

        # return (r,v)

    
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
