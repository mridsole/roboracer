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

    def THRESH_BLUE_LINE(x_bgr):

        THRESH_1 = (
            np.array([193/2, 50, 55]),
            np.array([220/2, 210, 255]) 
        )

        # THRESH_2 = (
        #     np.array([199/2, 80, 60]),
        #     np.array([219/2, 140, 255]) 
        # )
        THRESH_2 = THRESH_1

        t1 = cv2.inRange(x_bgr, THRESH_1[0], THRESH_1[1])
        t2 = cv2.inRange(x_bgr, THRESH_2[0], THRESH_2[1])

        return cv2.bitwise_or(t1, t2)


    def THRESH_YELLOW_LINE(x_bgr):

        THRESH_1 = (
            np.array([38/2, 25, 60]),
            np.array([75/2, 200, 255]) 
        )

        return cv2.inRange(x_bgr, THRESH_1[0], THRESH_1[1])
        # t2 = cv2.inRange(x_bgr, THRESH_1[0], THRESH_1[1])

        # return cv2.bitwise_or(t1, t2)

    
    def THRESH_OBSTACLES(x_bgr):

        THRESH_1 = (
            np.array([245/2, 20, 40]),
            np.array([300/2, 190, 255]) 
        )

        return cv2.inRange(x_bgr, THRESH_1[0], THRESH_1[1])



    DEFAULT_OPTIONS = {

        'DEPTH_SCALE': 0.001,

        # Minimum z distance of the depth buffer (below which values are invalid).
        'DEPTH_MINIMUM': 0.3,

        # Sample only this many points for the lines in 3D.
        'LINES_SUBSAMPLE_N': 100,

        # Sample only this many points for the lines in 3D.
        'OBSTACLE_SUBSAMPLE_N': 100,

        # Maximum line distance to consider
        # TODO: we should be able to increase this once we don't
        # have to deal with the lockers
        'MAX_LINE_DISTANCE': 3.2,
        
        # Threshold of the left line, in HLS, (min, max).
        'THRESH_L': THRESH_BLUE_LINE,

        # Function to threshold the right line.
        'THRESH_R': THRESH_YELLOW_LINE,

        # Threshold of obstacles, in HLS, (min, max).
        'THRESH_OBSTACLES': THRESH_OBSTACLES,

        # Threshold of other cars, in HLS, (min, max). TODO: etc
        'THRESH_CARS': ( np.array([]), np.array([]) ),

        # Kernel for threshold cleaning (morphological open)
        'THRESH_OPEN_KERNEL': np.ones((3,3)),

        'OBS_CLOSE_KERNEL': cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(30,30)),
        'OBS_ERODE_KERNEL': cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(40,40)),

        # Minimum y value (in camera space) for line points to register)
        # TODO: deprecated
        'MIN_LINES_Y': -0.2,

        # Canny thresholds for side lines.
        'LINE_CANNY_MIN': 90,
        'LINE_CANNY_MAX': 130,

        # How far from the ground plane are line points allowed to be?
        'GROUND_PLANE_HEIGHT_TOL': 0.05,

        # Provide a ground plane to be used (instead of estimating
        # based on lines). If none, one will be calculated from lines.
        'GROUND_PLANE': None,

        # Debug plots
        'DEBUG': True
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
        # self._hls = cv2.blur(self._hls, (5,5))
        # self._hls = cv2.medianBlur(self._hls, 3)
        # self._gray = cv2.cvtColor(self._color, cv2.COLOR_BGR2GRAY)

        # Store the options.
        self.options = options

        # Dimensions of the frames.
        H_c, W_c, K = self._color.shape
        H_d, W_d = self._depth.shape

        # Dimensionality sanity check.
        assert K == 3, 'Need only 3 color channels (BGR).'
        assert H_c == H_d and W_c == W_d, 'Color-depth dimension mismatch.'

        # Store the frame dimensions.
        self.frame_dims = (H_c, W_c)

    
    # @cachedproperty
    # def line_edges(self):
    #     """
    #     Canny edges computed on saturation, used for refining masks for
    #     the side lines.
    #     """

    #     mask = cv2.Canny(
    #         self._hls[:, :, 2],
    #         self.options['LINE_CANNY_MIN'],
    #         self.options['LINE_CANNY_MAX']
    #     )

    #     kernel = np.ones((4,4), np.uint8)
    #     mask = cv2.dilate(mask, kernel)

    #     if self.options['DEBUG']:
    #         cv2.imshow('canny', mask)


    #     return mask.ravel()


    @cachedproperty
    def line_l_mask(self):
        """
        Image mask for the left line.
        """
        
        mask = self.options['THRESH_L'](self._hls)
        kernel = self.options['THRESH_OPEN_KERNEL']
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        if self.options['DEBUG']:
            cv2.imshow('left', mask)

        return mask.ravel()


    @cachedproperty
    def line_r_mask(self):
        """
        Image mask for the right line.
        """

        mask = self.options['THRESH_R'](self._hls)
        kernel = self.options['THRESH_OPEN_KERNEL']
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        if self.options['DEBUG']:
            cv2.imshow('right', mask)

        return mask.ravel()


    @cachedproperty
    def obstacle_mask(self):
        """
        Image mask for the obstacles.
        """

        mask = self.options['THRESH_OBSTACLES'](self._hls)

        mask = cv2.morphologyEx(
            mask, 
            cv2.MORPH_CLOSE, 
            self.options['OBS_CLOSE_KERNEL']
        )

        mask = cv2.morphologyEx(
            mask, 
            cv2.MORPH_ERODE, 
            self.options['OBS_ERODE_KERNEL']
        )

        if self.options['DEBUG']:
            cv2.imshow('obstacle', mask)

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

        pts = self._pointcloud.calculate(self._depth_frame)
        pts3d = np.asanyarray(pts.get_vertices())
        pts3d = pts3d.view(np.float32).reshape(pts3d.shape + (-1,))
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
    def local_object_vmask(self):
        """
        Validity mask for local objects (cars/obstacles).
        """
        # Only consider close points
        return np.logical_and(
            self.pts3d_vmask, 
            self.pts3d[:,2] < self.options['MAX_LINE_DISTANCE']
        )

        # return np.logical_and(
        #     self.pts3d_vmask,
        #     self._depth.flatten() * self.options['DEPTH_SCALE'] < \
        #         self.options['MAX_LINE_DISTANCE']
        # )

    
    @cachedproperty
    def ground_plane_pts_vmask(self):
        """
        Validity mask for points on the local ground plane. Just take
        the image-space region below the horizon for now ...
        """

        H, W = self.frame_dims
        mask = np.ones(self.frame_dims)
        mask[0:int(0.32 * H),:] = 0.
        return mask.flatten()
        # mask = np.logical_and(mask.flatten(), self.local_object_vmask)

        return mask


    @cachedproperty
    def center_vmask(self):

        H, W = self.frame_dims
        vmask = np.ones(self.frame_dims)
        vmask[:, :int(0.15*W)] = 0.
        vmask[:, int(0.85*W):] = 0.
        return vmask.flatten()

    
    @cachedproperty
    def line_l_vmask(self):
        return np.logical_and(self.line_l_mask, self.ground_plane_pts_vmask)


    @cachedproperty
    def line_r_vmask(self):

        mask = np.logical_and(self.line_r_mask, self.ground_plane_pts_vmask)

        if self.options['DEBUG']:
            H,W = self.frame_dims
            cv2.imshow('r_vmask', 255*np.array(mask, dtype=np.uint8).reshape((H,W)))

        return mask



    @cachedproperty
    def obstacle_vmask(self):
        mask = np.logical_and(self.obstacle_mask, self.local_object_vmask)
        mask = np.logical_and(mask, self.center_vmask)
        mask = np.logical_and(mask, self.ground_plane_pts_vmask)
        return mask


    @cachedproperty
    def lines_vmask(self):
        """
        Validity mask for the lines.
        """

        mask = np.logical_or(self.line_l_vmask, self.line_r_vmask)

        if self.options['DEBUG']:
            H,W = self.frame_dims
            cv2.imshow('whew', 255*np.array(mask, dtype=np.uint8).reshape((H,W)))

        return mask


    @cachedproperty
    def line_l_pts(self):

        return self.subsample_pts(
            self.pts3d[self.line_l_vmask],
            self.options['LINES_SUBSAMPLE_N']
        )
    

    @cachedproperty
    def line_r_pts(self):

        return self.subsample_pts(
            self.pts3d[self.line_r_vmask],
            self.options['LINES_SUBSAMPLE_N']
        )


    @cachedproperty
    def line_l_pts_plane(self):

        # A final filtering operation: only consider points that are
        # within a certain distance of the plane.
        pts_plane = self.pts_camera_to_plane(self.line_l_pts)

        in_plane = np.abs(pts_plane[:,2]) < self.options['GROUND_PLANE_HEIGHT_TOL']
        return pts_plane[in_plane, :]
    

    @cachedproperty
    def line_r_pts_plane(self):
        pts_plane = self.pts_camera_to_plane(self.line_r_pts)

        in_plane = np.abs(pts_plane[:,2]) < self.options['GROUND_PLANE_HEIGHT_TOL']
        return pts_plane[in_plane, :]
    

    @cachedproperty
    def lines_pts(self):
        """
        Sampled points on the lines vmask.
        """
        return np.vstack((self.line_l_pts, self.line_r_pts))


    @cachedproperty
    def obstacle_pts(self):

        return self.subsample_pts(
            self.pts3d[self.obstacle_vmask],
            self.options['OBSTACLE_SUBSAMPLE_N'])


    @cachedproperty
    def obstacle_pts_plane(self):
        return self.pts_camera_to_plane(self.obstacle_pts)


    @cachedproperty
    def lines_pts_plane(self):
        return np.vstack((self.line_l_pts_plane, self.line_r_pts_plane))


    @cachedproperty
    def lines_pts_plane_2D(self):
        return self.lines_pts_plane[:, 0:2]


    
    @cachedproperty
    def pts_ground_plane_mask(self):

        mask = np.abs(self.pts_ground_plane_height) < \
            self.options['GROUND_PLANE_HEIGHT_TOL']

        if self.options['DEBUG']:
            cv2.imshow('gpheight', 255*np.array(
                mask.reshape(self.frame_dims), dtype=np.uint8
            ))

        return mask


    @cachedproperty
    def ground_pca(self):
        """
        Scikit-learn PCA object for the ground plane estimation.

        This uses the points in the front bottom of the camera's view,
        so it assumes the camera is somewhat pointing down at the ground.
        """

        # TODO: fit to pts3d, masked by valid LR points.

        # Mask by z validity.
        # pts = self.pts3d[self.pts3d_vmask]

        pts = self.lines_pts
        if pts.shape[0] < 10:
            return None

        # Subsample and compute PCA.
        # pts_subsampled = self.pts3d[
        #     np.random.randint(pts.shape[0], size=200), :
        # ]
        pca = PCA(n_components=3)
        pca.fit(pts)

        return pca


    @cachedproperty
    def front_ground_plane(self):
        """
        A ground plane computed from the pixels immediately in front.
        This is a bit of a hack, just using this for testing.
        """

        H, W = self.frame_dims
        pts = self.pts3d.reshape(self.frame_dims + (3,))
        pts = pts[
            int((1/3) * H) : int((2/3) * H),
            int((1/3) * W) : int((2/3) * W),
        ]
        pts = pts.reshape((pts.shape[0] * pts.shape[1], 3))
        pts = self.subsample_pts(pts, 100)

        if len(pts) < 5: return None

        pca = PCA(n_components=3)
        pca.fit(pts)
        e3 = pca.components_[2]
        n = np.sign(np.dot([0, -1, 0], e3)) * e3
        k = -np.dot(n, pca.mean_)
        return (n, k)


    @cachedproperty
    def ground_plane(self):
        """
        Ground plane in the camera's coordinate frame, as (n,k). Where:
            x . n = k
        This uses the points in the front bottom of the camera's view,
        so it assumes the camera is somewhat pointing down at the ground.
        """

        # If we were given a ground plane, use that.
        if self.options['GROUND_PLANE'] is not None:
            return self.options['GROUND_PLANE']


        # Just compute ground plane based on points immediately in front.
        # This is a bit of a hack and shouldn't be used in realtime (just
        # for calibration / getting the plane to begin with). The actual
        # ground plane should be specified as a constant.
        return self.front_ground_plane

        # if self.ground_pca == None: return None

        # # Take the third component, e3, as the (up to direction reversal)
        # # normal of the ground plane.
        # pca = self.ground_pca
        # e3 = pca.components_[2]

        # # Compute the normal, ensuring that it points 'upwards' (roughly
        # # in the direction of the negative y axis)
        # n = np.sign(np.dot([0, -1, 0], e3)) * e3

        # # Compute the perpendicular distance to the plane, so that we have
        # # the plane in the form:
        # #   x . n = k
        # k = np.dot(n, pca.mean_)

        # return (n, k)


    @cachedproperty
    def Tpc(self):
        """
        Homogeneous transformation from ground plane space to camera space.
        """

        if self.ground_plane == None:
            return np.eye(4)

        # Plane normal and k s.t.:  x . n = k
        n, k = self.ground_plane

        # Camera axes in camera frame:
        j_c = np.array([1,0,0])
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
            np.hstack((R, np.asarray([t]).T)),
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

    
    def subsample_pts(self, pts, N):
        """
        Subsample at most N points from pts.
        """

        # If we want to sample more points than we have,
        # just return all the points - otherwise, randomly sample.
        if N >= pts.shape[0]:
            return pts
        else:
            return pts[np.random.choice(pts.shape[0], N, replace=False), :]
