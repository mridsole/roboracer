import pyrealsense2 as rs
import numpy as np
import cv2
import click
import IPython

# Assorted testing for device capture and processing ...

# Device setup.
# DIMS = (1280, 720)
DIMS = (848, 480)
FPS = 30
config = rs.config()
config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, FPS)
config.enable_stream(rs.stream.color, DIMS[0], DIMS[1], rs.format.bgr8, FPS)
pipeline = rs.pipeline()
profile = pipeline.start(config)

IPython.embed()

# Capture frames for debugging
i = 0
while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame: continue
    
    # Deprojection example
    depth_data = np.asanyarray(depth_frame.get_data())
    # depth_data[:,:] = 0
    pcl = rs.pointcloud()
    pts = pcl.calculate(depth_frame)
    # pcl.map_to(color_frame)
    x = np.asanyarray(pts.get_vertices())
    xx = x.view(np.float32).reshape(x.shape + (-1,))
    # IPython.embed()
    print('x: ', np.mean(xx[:,0]))
    print('y: ', np.mean(xx[:,1]))
    print('z: ', np.mean(xx[:,2]))
    # This shouldn't use much processing! and yet it still does.
