"""
The final real-time control loop.
"""

import pyrealsense2 as rs
import numpy as np
from procframe import FrameInfo, FrameObjects, Trajectory
from controller import Controller
from motors import MotorHAL
import cv2
import time


# First initialize the motor HAL which opens a connection with the Arduino.
# This will make the wheels spin for a couple of seconds.

DIMS = (848, 480)
FPS = 15

# RealSense configuration.
config = rs.config()
config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, FPS)
config.enable_stream(rs.stream.color, DIMS[0], DIMS[1], rs.format.bgr8, FPS)

# Pipeline for frame capture.
pipeline = rs.pipeline()
pipeline.start(config)

input('Enter to start: ')
mhal = MotorHAL()
time.sleep(2)
controller = Controller(mhal)

# No debug.
opts = FrameInfo.DEFAULT_OPTIONS.copy()
opts['DEBUG'] = False


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

    # This isn't actually the "tick", this is setting the move reference based
    # on state.
    controller.tick(traj)
    # print(traj.immediate_path[1])

    # Set motor command. TODO: deprecate, this is handled by the controller.
    # mhal.set_cmd(*cmd)

    # Poll for velocity (why not?)
    # :w
    # print(mhal.get_vel())
    mhal.get_vel()
