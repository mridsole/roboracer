"""
The final real-time control loop.
"""

import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle, ConnectionPatch
from matplotlib.collections import PatchCollection
from procframe import FrameInfo, FrameObjects, Trajectory
from controller import Controller
from motors import MotorHAL
import cv2
import time


# First initialize the motor HAL which opens a connection with the Arduino.
# This will make the wheels spin for a couple of seconds.
# mhal = MotorHAL()
# input('Enter to start: ')

class MotorHALMock():

    def __init__(self, serialpath=''):
        pass

    def set_cmd(self, vl, vr):
        pass

    def get_vel(self):
        return (0, 0)

mhal = MotorHALMock()

controller = Controller(mhal)

DIMS = (848, 480)
FPS = 15

# Device setup.
config = rs.config()
config.enable_device_from_file('../data/finaltrack3.bag')
config.enable_all_streams()
pipeline = rs.pipeline()
pipeline.start(config)

# No debug.
opts = FrameInfo.DEFAULT_OPTIONS.copy()
opts['DEBUG'] = True

# Initialize plotting.
plt.ion()
fig, ax = plt.subplots()
sc_l = ax.scatter([],[], s=10, color=(0., 0., 0.9))
sc_r = ax.scatter([],[], s=10, color=(0.8, 0.8, 0))
sc_obs = ax.scatter([],[], s=10)
# ax.axis('equal')
plt.xlim(-3, 3)
plt.ylim(-1, 5)
plt.grid(True)
plt.draw()


circ = None
line_l = None
line_r = None
line_target = None
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
    cmd = controller.tick(traj)
    print('Turn: ', traj.immediate_path[1])

    if line_l is not None: 
        w = line_l.pop(0)
        if w is not None: w.remove()
        line_l = None
    if line_r is not None: 
        w = line_r.pop(0)
        if w is not None: w.remove()
        line_r = None
    if line_target is not None: 
        w = line_target.pop(0)
        if w is not None: w.remove()
        line_target = None

    # Scatter points
    pts_l = frinfo.line_l_pts_plane
    sc_l.set_offsets(np.c_[pts_l[:,0], pts_l[:,1]])
    pts_r = frinfo.line_r_pts_plane
    sc_r.set_offsets(np.c_[pts_r[:,0], pts_r[:,1]])
    pts_obs = frinfo.obstacle_pts_plane
    sc_obs.set_offsets(np.c_[pts_obs[:,0], pts_obs[:,1]])

    # Plot left line
    if frobj.left_line:
        n, k = frobj.left_line
        p1 = k * n
        v = np.array([n[1], -n[0]])
        if v.dot([0,1]) < 0: v = -v
        p2 = p1 + 3*v

        # ax.add_patch(ConnectionPatch(xyA=p1, xyB=p2, coordsA='data', coordsB='data'))
        line_l = plt.plot([p1[0], p2[0]], [p1[1], p2[1]])

    if frobj.right_line:
        n, k = frobj.right_line
        p1 = k * n
        v = np.array([-n[1], n[0]])
        if v.dot([0,1]) < 0: v = -v
        p2 = p1 + 3*v
        # ax.add_patch(ConnectionPatch(xyA=p1, xyB=p2, coordsA='data', coordsB='data'))
        line_r = plt.plot([p1[0], p2[0]], [p1[1], p2[1]])

    if traj.avoid_line:
        n, k = traj.avoid_line
        p1 = k * n
        v = np.array([n[1], -n[0]])
        if v.dot([0,1]) < 0: v = -v
        p2 = p1 + 3*v
        line_target = plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='green')

    fig.canvas.draw_idle()
    plt.pause(0.001)

    # time.sleep(0.5)

    cv2.imshow('color', np.asanyarray(color_frame.get_data()))
    cv2.imshow('depth', cv2.applyColorMap(
        cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03), 
        cv2.COLORMAP_JET
    ))

    cv2.waitKey(1)


    # Set motor command. TODO: deprecate, this is handled by the controller.
    # mhal.set_cmd(*cmd)

    # Poll for velocity (why not?)
