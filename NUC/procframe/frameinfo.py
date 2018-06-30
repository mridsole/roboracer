import numpy as np
import cv2


class FrameInfo:
    """
    Extract relevant information from a RealSense frame, such as:
     - Thresholds for lines and obstacles
     - Randomly sampled points in lines and obstacles
     - Somehow find the 'ground plane' (RANSAC?)
    """


    DEFAULT_OPTIONS = {

        # Scale of the depth scale.
        'DEPTH_SCALE': 0.001,
        
        # Threshold of the left line, in HLS, (min, max).
        'THRESH_L': ( np.array([]), np.array([]) ),

        # Threshold of the right line, in HLS, (min, max).
        'THRESH_R': ( np.array([]), np.array([]) ),

        # Threshold of obstacles, in HLS, (min, max).
        'THRESH_OBSTACLES': ( np.array([]), np.array([]) ),

        # Threshold of other cars, in HLS, (min, max).
        'THRESH_CARS': ( np.array([]), np.array([]) )
    }

    def __init__(
            self, color_frame, depth_frame, 
            options=DEFAULT_OPTIONS
    ):
        """
        :param color_frame: The [3,H,W] BGR np array color image (uint8).
        :param color_frame: The [H,W] np.array depth image (uint16).
        :param config: Configuration options (see DEFAULT_OPTIONS).
        """
        
        # Store the two frames.
        self._color = color_frame
        self._depth = depth_frame

        # Get color image in HLS space (hue-lightness-saturation).
        self._hls = cv2.cvtColor(self._color, cv2.COLOR_BGR2HLS)

        # Store the options.
        self.options = options
        # self._depth_scale = options['DEPTH_SCALE']

        # Dimensions of the frames.
        K, H_c, W_c = color_frame.shape
        H_d, W_d = depth_frame.shape

        # Dimensionality sanity check.
        assert K == 3, 'Need only 3 color channels (BGR).'
        assert H_c == H_d and W_c == W_d, 'Color-depth dimension mismatch.'

        # Store the frame dimensions.
        self.frame_dims = (H_c, W_c)

        # Lazily computed
        self._line_l_mask = None
        self._line_r_mask = None
        self._obstacle_mask = None
        self._cars_mask = None
        self._ground_plane = None


    @property
    def line_l_mask(self):
        """
        Image mask for the left line.
        """
        
        if self._line_l_mask is None:
            (low, high) = self.options['THRESH_L']
            self.line_l_mask = cv2.inRange(self._hls, low, high)
        
        return self._line_l_mask


    @property
    def line_r_mask(self):
        """
        Image mask for the right line.
        """

        if self._line_r_mask is None:
            (low, high) = self.options['THRESH_R']
            self.line_r_mask = cv2.inRange(self._hls, low, high)
        
        return self._line_r_mask


    @property
    def obstacle_mask(self):
        """
        Image mask for the obstacles.
        """

        if self._obstacle_mask is None:
            (low, high) = self.options['THRESH_OBSTACLES']
            self._obstacle_mask = cv2.inRange(self._hls, low, high)
        
        return self._obstacle_mask


    @property
    def cars_mask(self):
        """
        Image mask for the other cars.
        """

        if self._cars_mask is None:
            (low, high) = self.options['THRESH_CARS']
            self._cars_mask = cv2.inRange(self._hls, low, high)
        
        return self._cars_mask


    @property
    def ground_plane(self):
        """
        Ground plane in the camera's coordinate frame, as (n,k). Where: 
            np.dot(n, x) = k
        """
        pass

