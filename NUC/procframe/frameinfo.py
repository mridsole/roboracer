import numpy as np
import cv2
from sklearn.decomposition import PCA
import pyrealsense2 as rs
from boltons.cacheutils import cachedproperty


class FrameInfo:
    """
    Extract relevant information from a RealSense frame, such as:
     - Thresholds for lines and obstacles
     - Randomly sampled points in lines and obstacles
     - Somehow find the 'ground plane' (RANSAC?)
    """


    DEFAULT_OPTIONS = {

        # Depth buffer resolution in metres.
        'DEPTH_SCALE': 0.001,

        # Minimum z distance of the depth buffer (below which values are invalid).
        'DEPTH_MINIMUM': 0.3,

        # Take only this many points for the ground plane estimation.
        'PCA_SUBSAMPLE_N': 100,
        
        # Threshold of the left line, in HLS, (min, max).
        'THRESH_L': ( np.array([211/2, 15, 120]), np.array([249/2, 153, 255]) ),

        # Threshold of the right line, in HLS, (min, max).
        'THRESH_R': ( np.array([50/2, 15, 120]), np.array([70/2, 153, 255]) ),

        # Threshold of obstacles, in HLS, (min, max). TODO: change from red to purple
        'THRESH_OBSTACLES': ( np.array([]), np.array([]) ),

        # Threshold of other cars, in HLS, (min, max). TODO: etc
        'THRESH_CARS': ( np.array([]), np.array([]) )
    }

    def __init__(
            self, color_frame, depth_frame,
            options=DEFAULT_OPTIONS
    ):
        """
        :param color_frame: The realsense color frame (BGR8).
        :param depth_frame: The realsense depth frame (uint8).
        :param config: Configuration options (see DEFAULT_OPTIONS).
        """
        
        # Store the two frames.
        self._color_frame = color_frame
        self._depth_frame = depth_frame

        # Store direct reference to the numpy data.
        self._color = np.asanyarray(color_frame.get_data())
        self._depth = np.asanyarray(depth_frame.get_data())

        # Pointcloud object for 3D calculations.
        self._pointcloud = rs.pointcloud()

        # Get color image in HLS space (hue-lightness-saturation).
        self._hls = cv2.cvtColor(self._color, cv2.COLOR_BGR2HLS)

        # Store the options.
        self.options = options
        # self._depth_scale = options['DEPTH_SCALE']

        # Dimensions of the frames.
        H_c, W_c, K = self._color.shape
        H_d, W_d = self._depth.shape

        # Dimensionality sanity check.
        assert K == 3, 'Need only 3 color channels (BGR).'
        assert H_c == H_d and W_c == W_d, 'Color-depth dimension mismatch.'

        # Store the frame dimensions.
        self.frame_dims = (H_c, W_c)


    @cachedproperty
    def line_l_mask(self):
        """
        Image mask for the left line.
        """
        
        (low, high) = self.options['THRESH_L']
        mask = cv2.inRange(self._hls, low, high)
        return mask


    @cachedproperty
    def line_r_mask(self):
        """
        Image mask for the right line.
        """

        (low, high) = self.options['THRESH_R']
        mask = cv2.inRange(self._hls, low, high)
        return mask


    @cachedproperty
    def obstacle_mask(self):
        """
        Image mask for the obstacles.
        """

        (low, high) = self.options['THRESH_OBSTACLES']
        mask = cv2.inRange(self._hls, low, high)
        return mask


    @cachedproperty
    def cars_mask(self):
        """
        Image mask for the other cars.
        """

        (low, high) = self.options['THRESH_CARS']
        mask = cv2.inRange(self._hls, low, high)
        return mask

    
    @cachedproperty
    def pts3d(self):
        """
        Deprojected points with shape (W*H, 3). Note this still includes
        points with z < 0.3 (these should be removed after).
        """

        # TODO: Reshape this to self.frame_dims (not sure what the correct
        # order is, however.) OTOH might be better to reshape later

        pts = self._pointcloud.calculate(self._depth_frame)
        pts3d = np.asanyarray(pts.get_vertices())
        pts3d = pts3d.view(np.float32).reshape(pts3d.shape + (-1,))
        # TODO: reshape here
        return pts3d


    @cachedproperty
    def pts3d_vmask(self):
        """
        Validity mask (mask of points with z > 0.3).
        """

        # TODO: Account for future reshape of pts3d
        mask = self.pts3d[:, 2] > self.options['DEPTH_MINIMUM']
        return mask


    @cachedproperty
    def ground_pca(self):
        """
        Scikit-learn PCA object for the ground plane estimation,
        based on the left and right line points.
        """

        # TODO: fit to pts3d, masked by valid LR points.

        # Mask by z validity.
        pts = self.pts3d[self.pts3d_vmask]

        # Subsample and compute PCA.
        pts_subsampled = self.pts3d[
            np.random.randint(pts.shape[0], size=200), :
        ]
        pca = PCA(n_components=3)
        pca.fit(pts_subsampled)

        return pca


    @cachedproperty
    def ground_plane(self):
        """
        Ground plane in the camera's coordinate frame, as (n,k). Where:
            `np.dot(n, x) == k`
        This uses the left and right line thresholds, and assumes that those
        points should be on a plane. Just using least squares for now, maybe
        RANSAC later if there are other points in the threshold.
        """

        # TODO: Use PCA from scikit-learn here, take the third principal
        # component, then n = -sign( z dot e3) e3
        # Then can recover a coordinate system for the plane from there

        pca = PCA(n_components=3)
        pca.fit(self.pts3d)
