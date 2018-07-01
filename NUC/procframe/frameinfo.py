import numpy as np
import cv2
from sklearn.decomposition import PCA
import pyrealsense2 as rs
from boltons.cacheutils import cachedproperty

# TODO: Can we somehow re-use memory here? Creating a new one of these
# each frame seems wasteful somehow, probably uses a lot of time for
# allocation.

class FrameInfo:
    """
    Extract relevant information from a RealSense frame, such as:
     [x] Thresholds for lines and obstacles
     [x] Randomly sampled points in lines and obstacles
     [x] Somehow find the 'ground plane' (RANSAC?)
    """


    DEFAULT_OPTIONS = {

        # Depth buffer resolution in metres.
        'DEPTH_SCALE': 0.001,

        # Minimum z distance of the depth buffer (below which values are invalid).
        'DEPTH_MINIMUM': 0.3,

        # Sample only this many points for the lines in 3D.
        'LINES_SUBSAMPLE_N': 100,
        
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
        return mask.ravel()


    @cachedproperty
    def line_r_mask(self):
        """
        Image mask for the right line.
        """

        (low, high) = self.options['THRESH_R']
        mask = cv2.inRange(self._hls, low, high)
        return mask.ravel()


    @cachedproperty
    def obstacle_mask(self):
        """
        Image mask for the obstacles.
        """

        (low, high) = self.options['THRESH_OBSTACLES']
        mask = cv2.inRange(self._hls, low, high)
        return mask.ravel()


    @cachedproperty
    def cars_mask(self):
        """
        Image mask for the other cars.
        """

        (low, high) = self.options['THRESH_CARS']
        mask = cv2.inRange(self._hls, low, high)
        return mask.ravel()

    
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
    def lines_vmask(self):
        """
        Validity mask for the lines.
        """

        lines_mask = np.logical_or(self.line_l_mask, self.line_r_mask)
        mask = np.logical_and(lines_mask, self.pts3d_vmask)
        return mask


    @cachedproperty
    def lines_pts(self):
        """
        Sampled points on the lines vmask.
        """

        # Filter all 3D points by the lines validity mask.
        pts = self.pts3d[self.lines_vmask]

        # How many points should we sample?
        n_to_sample = self.options['LINES_SUBSAMPLE_N']

        # If we want to sample more points than we have,
        # just return all the points - otherwise, randomly sample.
        if n_to_sample >= pts.shape[0]:
            return pts
        else:
            return pts[
                np.random.choice(pts.shape[0], n_to_sample, replace=False),
                :
            ]


    @cachedproperty
    def lines_pts_plane(self):
        return self.pts_camera_to_plane(self.lines_pts)


    @cachedproperty
    def lines_pts_plane_2D(self):
        return self.lines_pts_plane[:, 0:2]


    @cachedproperty
    def ground_pca(self):
        """
        Scikit-learn PCA object for the ground plane estimation,
        based on the left and right line points.

        # If points are available, returns None.
        """

        # TODO: fit to pts3d, masked by valid LR points.

        # Mask by z validity.
        pts = self.pts3d[self.pts3d_vmask]

        if pts.shape[0] < 10:
            return None

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
            x . n = k
        This uses the left and right line thresholds, and assumes that those
        points should be on a plane. Just using least squares for now, maybe
        RANSAC later if there are other points in the threshold.

        If no ground plane is found, returns None.
        """

        if self.ground_pca == None: return None

        # Take the third component, e3, as the (up to direction reversal)
        # normal of the ground plane.
        pca = self.ground_pca
        e3 = pca.components_[2]

        # Compute the normal, ensuring that it points 'upwards' (roughly
        # in the direction of the negative y axis)
        n = np.sign(np.dot([0, -1, 0], e3)) * e3

        # Compute the perpendicular distance to the plane, so that we have
        # the plane in the form:
        #   x . n = k
        k = np.dot(n, pca.mean_)

        return (n, k)


    @cachedproperty
    def Tpc(self):
        """
        Homogeneous transformation from ground plane space to camera space.
        """

        # Plane normal and k s.t.:  x . n = k
        n, k = self.ground_plane

        # Camera axes in camera frame:
        x_c = np.array([1,0,0])
        y_c = np.array([0,1,0])
        z_c = np.array([0,0,1])

        # Get the plane's coordinate axes vectors in the camera frame.
        # This defines the rotation matrix of Tpc.
        z_p = n
        y_p = z_c + np.dot(z_c, n) * n
        y_p = y_p / np.linalg.norm(y_p)
        x_p = np.cross(y_p, z_p)

        R = np.array([x_p, y_p, z_p]).T
        t = k * n

        return np.vstack((
            np.hstack((R, np.asmatrix(t).T)),
            np.array([[0, 0, 0, 1]])
        ))


    @cachedproperty
    def Tcp(self):
        """
        Homogeneous transformation from camera space to ground plane space.
        Inverse of Tpc.
        """

        R = self.Tpc[0:3, 0:3].T
        t = np.dot(R, self.Tpc[0:3, 3:])

        return np.vstack((
            np.hstack((R, t)),
            np.array([[0, 0, 0, 1]])
        ))


    def pts_camera_to_plane(self, pts):
        """
        Transform points in camera space to plane space.
        :param pts: (N,3)-shaped points in camera space.
        :returns: (N,3)-shaped points in plane space.
        """

        # Homogeneous coordinates of camera-space points.
        pts_homo = np.hstack((pts, np.ones((pts.shape[0],1)))).T

        # Return transformed points.
        return np.dot(self.Tcp, pts_homo)[:3, :].T
