import pyrealsense2 as rs
import numpy as np
from procframe import FrameInfo, FrameObjects
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle, ConnectionPatch
from matplotlib.collections import PatchCollection
import cv2
import IPython

# Testing FrameInfo

# Record at most one minute of data.
MAX_FRAMES = None
DIMS = (848, 480)
FPS = 15

# # RealSense configuration.
# config = rs.config()
# config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, FPS)
# config.enable_stream(rs.stream.color, DIMS[0], DIMS[1], rs.format.bgr8, FPS)
# 
# # Pipeline for frame capture.
# pipeline = rs.pipeline()
# pipeline.start(config)

# Device setup.
config = rs.config()
config.enable_device_from_file('../data/real2.bag')
config.enable_all_streams()
pipeline = rs.pipeline()
pipeline.start(config)


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

opts = FrameInfo.DEFAULT_OPTIONS.copy()
opts['DEBUG'] = True


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
    if opts['GROUND_PLANE'] is None:
        opts['GROUND_PLANE'] = frinfo.front_ground_plane
    # pts = frinfo.pts_camera_to_plane(frinfo.lines_pts)

    frobj = FrameObjects(frinfo)

    [p.remove() for p in reversed(ax.patches)]

    vm = frinfo.lines_vmask

    if line_l is not None: 
        w = line_l.pop(0)
        if w is not None: w.remove()
        line_l = None
    if line_r is not None: 
        w = line_r.pop(0)
        if w is not None: w.remove()
        line_r = None

    # Scatter points
    pts_l = frinfo.line_l_pts_plane
    sc_l.set_offsets(np.c_[pts_l[:,0], pts_l[:,1]])
    pts_r = frinfo.line_r_pts_plane
    sc_r.set_offsets(np.c_[pts_r[:,0], pts_r[:,1]])
    pts_obs = frinfo.obstacle_pts_plane
    sc_obs.set_offsets(np.c_[pts_obs[:,0], pts_obs[:,1]])

    # Plot obstacle circle
    if frobj.obstacle is not None:
        c, r = frobj.obstacle
        circ = Circle((c[0], c[1]), r, color=(0.5,0,0.5))
        ax.add_patch(circ)

    # Plot left line
    if frobj.left_line:
        n, k = frobj.left_line
        p1 = k * n
        v = np.array([n[1], -n[0]])
        p2 = p1 + 3*v

        # ax.add_patch(ConnectionPatch(xyA=p1, xyB=p2, coordsA='data', coordsB='data'))
        line_l = plt.plot([p1[0], p2[0]], [p1[1], p2[1]])

    if frobj.right_line:
        n, k = frobj.right_line
        p1 = k * n
        v = np.array([-n[1], n[0]])
        p2 = p1 + 3*v
        ax.add_patch(ConnectionPatch(xyA=p1, xyB=p2, coordsA='data', coordsB='data'))
        line_r = plt.plot([p1[0], p2[0]], [p1[1], p2[1]])

    fig.canvas.draw_idle()
    plt.pause(0.001)

    cv2.imshow('color', np.asanyarray(color_frame.get_data()))
    cv2.imshow('deptth', cv2.applyColorMap(
        cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03), 
        cv2.COLORMAP_JET
    ))

    cv2.waitKey(1)
