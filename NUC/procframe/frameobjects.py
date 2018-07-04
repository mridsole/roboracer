import numpy as np
import cv2
from sklearn.decomposition import PCA
import pyrealsense2 as rs
from boltons.cacheutils import cachedproperty

class FrameObjects:

    DEFAULT_OPTIONS = {

        # Minimum points on a line to be detected as a line.
        'MIN_LINE_POINTS': 20,

        # TODO: min line variance explained (PCA)

        # Minimum points on an obstacle to be detected.
        'MIN_OBSTACLE_POINTS': 30,
    }

    def __init__(self, frinfo, opts=DEFAULT_OPTIONS):
        
        self.frinfo = frinfo
        self.options = opts


    @cachedproperty
    def left_line_pca(self):
        return self._best_line(self.frinfo.line_l_pts_plane[:, 0:2])

    
    @cachedproperty
    def right_line_pca(self):
        return self._best_line(self.frinfo.line_r_pts_plane[:, 0:2])


    @cachedproperty
    def left_line(self):
        """
        Best-fit left line.
        """
        if self.left_line_pca is None: return None
        n, k, pca = self.left_line_pca
        return (n, k)

    
    @cachedproperty
    def right_line(self):
        """
        Best-fit right line.
        """
        if self.right_line_pca is None: return None
        n, k, pca = self.right_line_pca
        return (n, k)


    @cachedproperty
    def left_v(self):
        
        if self.left_line is None: return None
        n, k = self.left_line
        v = np.array([n[1], -n[0]])
        if v[1] < 0:
            return -v 
        else:
            return v


    @cachedproperty
    def right_v(self):
        
        if self.right_line is None: return None
        n, k = self.right_line
        v = np.array([n[1], -n[0]])
        if v[1] < 0:
            return -v 
        else:
            return v

    
    @cachedproperty
    def line_intersection(self):
        """
        The intersection of the two lines, if both are defined.
        """

        if self.left_line is None or self.right_line is None:
            return None

        n1, k1 = self.left_line
        n2, k2 = self.right_line

        x = (k1 * n2[1] - n1[1] * k2) / (n1[0] * n2[1] - n1[1] * n2[0])
        y = (n1[0] * k2 - k1 * n2[0]) / (n1[0] * n2[1] - n1[1] * n2[0])

        return np.array([x, y])

    
    @cachedproperty
    def target_line(self):

        v1, v2 = self.left_v, self.right_v

        # If we have no lines, we can't do anything.
        if v1 is None and v2 is None:
            return None

        # If we have both lines, 
        if v1 is not None and v2 is not None:

            # Line defined by v and a point:
            v = (v1 + v2) / np.linalg.norm(v1 + v2)
            if v[1] < 0: v = -v
            xint = self.line_intersection
            return (v, xint)

        v = None
        n_in = None
        if v1 is not None:
            v = v1
            n_in = np.array([v[1], -v[0]])
            n, k = self.left_line
        else:
            v = v2
            n_in = np.array([-v[1], v[0]])
            n, k = self.right_line

        xint = k * n + 0.3 * n_in
        return (v, xint)


    @cachedproperty
    def target_line_nk(self):

        v, xint = self.target_line
        n = np.array([-v[1], v[0]])
        k = n.dot(xint)
        return (n, k)


    @cachedproperty
    def obstacle(self):
        """
        Best-fit circle for obstacle.
        """

        # Use PCA, take the greater axis, look at explained variance for the radius.
        pts = self.frinfo.obstacle_pts_plane[:, 0:2]
        N, _ = pts.shape
        if N < self.options['MIN_OBSTACLE_POINTS']:
            return None

        pca = PCA(n_components=2)
        pca.fit(pts)

        e1 = pca.components_[0]
        r = 4 * np.sqrt(pca.explained_variance_[0]) + 0.02

        # TODO: Manual outlier removal
        c = pca.mean_

        return (c, r)


    def _best_line(self, pts):

        N, _ = pts.shape
        if N < self.options['MIN_LINE_POINTS']:
            return None

        pca = PCA(n_components=2)
        pca.fit(pts)

        # Outwards pointing normal.
        e2 = pca.components_[1]
        k = np.abs(np.dot(e2, pca.mean_))
        n = np.sign(np.dot(pca.mean_, e2)) * e2

        return (n, k, pca)
