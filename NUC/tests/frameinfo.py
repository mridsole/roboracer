import pyrealsense2 as rs
import numpy as np
from procframe import FrameInfo
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import IPython

# Testing FrameInfo

# Device setup.
config = rs.config()
config.enable_device_from_file('../data/home2.bag')
config.enable_all_streams()
pipeline = rs.pipeline()
pipeline.start(config)

# Capture frames for debugging
while True:

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame: continue
    
    frinfo = FrameInfo(color_frame, depth_frame)
    # print(frinfo.line_l_mask.dtype)
    # print(frinfo.ground_plane)
    # print(frinfo.Tcp)
    # x = frinfo.Tcp
    x = frinfo.ground_plane
    pts = frinfo.pts_camera_to_plane(frinfo.lines_pts)
    print(pts.shape)
