"""
Basic frame-by-frame line-following test.
"""

import pyrealsense2 as rs
import numpy as np
from procframe import FrameInfo, FrameObjects, Trajectory
from motors import MotorHAL
import cv2
import time

DIMS = (848, 480)
FPS = 15

# RealSense configuration.
config = rs.config()
config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, FPS)
config.enable_stream(rs.stream.color, DIMS[0], DIMS[1], rs.format.bgr8, FPS)

# Pipeline for frame capture.
pipeline = rs.pipeline()
pipeline.start(config)

# Initialize the motor HAL
mhal = MotorHAL()

# No debug.
opts = FrameInfo.DEFAULT_OPTIONS.copy()
opts['DEBUG'] = False

input('Enter to start: ')

i = 0

# Capture frames for debugging
circ = None
line_l = None
line_r = None
while True:

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame: continue
    
    frinfo = FrameInfo(color_frame, depth_frame, options=opts)

    # TODO: we need to permanently store this ground plane transformation.
    if opts['GROUND_PLANE'] is None:
        opts['GROUND_PLANE'] = frinfo.front_ground_plane

    frobj = FrameObjects(frinfo)
    traj = Trajectory(frobj)

    print(traj.immediate_path)
    r, v = traj.immediate_path
    
    # Set motor command.
    mhal.set_cmd(r, v)

    # Poll for velocity
    mhal.get_vel()

    print(i) 
    i += 1
