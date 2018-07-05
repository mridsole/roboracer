"""
Basic frame-by-frame line-following test.
"""

import numpy as np
from controller import Controller
from motors import MotorHAL
import time
from boltons.cacheutils import cachedproperty


# Initialize the motor HAL

input('Enter to start: ')

DIMS = (848, 480)
FPS = 15

# RealSense configuration.
config = rs.config()
config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, FPS)
config.enable_stream(rs.stream.color, DIMS[0], DIMS[1], rs.format.bgr8, FPS)

# Pipeline for frame capture.
pipeline = rs.pipeline()
pipeline.start(config)

# No debug.
opts = FrameInfo.DEFAULT_OPTIONS.copy()
opts['DEBUG'] = False

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
time.sleep(2)

mhal.set_cmd(0.2, 0.2)
time.sleep(0.2)

while True:

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame: continue
    
    frinfo = FrameInfo(color_frame, depth_frame, options=opts)

    # TODO: we need to permanently store this ground plane transformation.
    if opts['GROUND_PLANE'] is None:
        opts['GROUND_PLANE'] = frinfo.front_ground_plane

    mt = MockTrajectory()
    cmd = controller.tick(mt)

    # Set motor command.
    mhal.set_cmd(*cmd)

    # Poll for velocity
    print(mhal.get_vel())

    if controller.slow_tick >= 50:
        mhal.set_cmd(0,0)
        break
