import pyrealsense2 as rs
import numpy as np
from procframe import FrameInfo, FrameObjects
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
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
config.enable_device_from_file('../data/hm1.bag')
config.enable_all_streams()
pipeline = rs.pipeline()
pipeline.start(config)


plt.ion()
fig, ax = plt.subplots()
sc_l = ax.scatter([],[], s=10)
sc_r = ax.scatter([],[], s=10)
sc_obs = ax.scatter([],[], s=10)
# ax.axis('equal')
plt.xlim(-3, 3)
plt.ylim(-3, 3)
plt.grid(True)
plt.draw()

opts = FrameInfo.DEFAULT_OPTIONS.copy()
opts['DEBUG'] = True


# Capture frames for debugging
circ = None
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
        circ = Circle((c[0], c[1]), r)
        ax.add_patch(circ)

    fig.canvas.draw_idle()
    plt.pause(0.001)

    cv2.imshow('color', np.asanyarray(color_frame.get_data()))
    cv2.imshow('deptth', cv2.applyColorMap(
        cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03), 
        cv2.COLORMAP_JET
    ))

    cv2.waitKey(1)
