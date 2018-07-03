import numpy as np
import cv2
from sklearn.decomposition import PCA
import pyrealsense2 as rs
from boltons.cacheutils import cachedproperty

class FrameObjects:

    DEFAULT_OPTIONS = {

        # Minimum points on a line to be detected as a line.
        'MIN_LINE_POINTS': 10,

        # TODO: min line variance explained (PCA)

        # Minimum points on an obstacle to be detected.
        'MIN_OBSTACLE_POINTS': 15,
    }

    def __init__(self, frinfo, opts=DEFAULT_OPTIONS):
        
        self.frinfo = frinfo
        self.options = opts


    @cachedproperty
    def left_line(self):
        """
        Best-fit left line.
        """

        return self._best_line(self.frinfo.line_l_pts_plane[:, 0:2])

    
    @cachedproperty
    def right_line(self):
        """
        Best-fit right line.
        """

        return self._best_line(self.frinfo.line_r_pts_plane[:, 0:2])


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
        print(r)

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

        return (n, k)
