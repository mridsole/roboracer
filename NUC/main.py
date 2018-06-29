# Try to capture depth data and color data.
import pyrealsense2 as rs
import numpy as np
import cv2

# Create a context object. This object owns the 
# handles to all connected realsense devices.

config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline = rs.pipeline()
pipeline.start(config)

while True:

    # This call waits until a new coherent set of frames is available 
    # on a device.
    frames = pipeline.wait_for_frames()
    depth_fr = frames.get_depth_frame()
    color_fr = frames.get_color_frame()
    if not depth_fr or not color_fr: continue

    # Get depth and color as numpy arrays.
    depth_data = np.asanyarray(depth_fr.as_frame().get_data())
    color_data = np.asanyarray(color_fr.as_frame().get_data())

    # Get depth color map
    import IPython; IPython.embed()
    # depth_data = (2**8) * np.round(depth_data / 2**8)
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_data, alpha=0.03), 
        cv2.COLORMAP_JET
    )

    cv2.imshow('depth', depth_colormap)
    cv2.imshow('color', color_data)
    cv2.waitKey(1)
