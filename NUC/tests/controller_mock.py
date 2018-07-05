"""
Basic frame-by-frame line-following test.
"""

import pyrealsense2 as rs
import numpy as np
from controller import Controller
from motors import MotorHAL
import time
from boltons.cacheutils import cachedproperty


# Initialize the motor HAL

DIMS = (848, 480)
FPS = 15

# RealSense configuration.
config = rs.config()
config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, FPS)
config.enable_stream(rs.stream.color, DIMS[0], DIMS[1], rs.format.bgr8, FPS)

# Pipeline for frame capture.
pipeline = rs.pipeline()
pipeline.start(config)

controller = Controller()

# Mock up a trajectory object.
class MockTrajectory:

    SLOW_SPEED = 0.2
    LEFT_TURN = (0.15, 0.65)
    RIGHT_TURN = (0.65, 0.15)

    @cachedproperty
    def immediate_path(self):
        return (MockTrajectory.RIGHT_TURN, 1)


mhal = MotorHAL()

input('waiting ...')

mhal.set_cmd(0.2, 0.2)
time.sleep(1.5)

while True:

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame: continue

    mt = MockTrajectory()
    cmd = controller.tick(mt)
    print(cmd)

    # Set motor command.
    mhal.set_cmd(*cmd)

    # Poll for velocity
    mhal.get_vel()
    # print(mhal.get_vel())

    if controller.slow_tick >= 150:
        mhal.set_cmd(0,0)
        time.sleep(100)
        break
