import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import IPython

# Assorted testing for device capture and processing ...

# Device setup.

config = rs.config()
config.enable_device_from_file('../data/home2.bag')
config.enable_all_streams()
pipeline = rs.pipeline()
pipeline.start(config)

# This segfaults ... need to use above when using pipelines
# context = rs.context()
# context.load_device('../data/home2.bag')
# pipeline = rs.pipeline(context)
# profile = pipeline.start()

# IPython.embed()

# plt.ion()
# time.sleep(0.001)
# fig = plt.figure(1)

# Capture frames for debugging
i = 0
pcl = rs.pointcloud()
while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame: continue
    
    # Deprojection example
    depth_data = np.asanyarray(depth_frame.get_data())
    pts = pcl.calculate(depth_frame)
    # pcl.map_to(color_frame)
    pts3d = np.asanyarray(pts.get_vertices())
    pts3d = pts3d.view(np.float32).reshape(pts3d.shape + (-1,))

    print('Points before removal: ', pts3d.shape[0])

    # IPython.embed()

    # Remove points that have z < 0.3
    pts3d = pts3d[pts3d[:, 2] > 0.3, :]

    print('Points after removal: ', pts3d.shape[0])

    # Sample 1000
    pts3d_sampled = pts3d[np.random.randint(pts3d.shape[0], size=1000), :]

    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pts3d_sampled[:,0], pts3d_sampled[:,1], pts3d_sampled[:,2])
    plt.show()

    # hist = plt.hist(pts3d[:,2], bins=300)
    # plt.grid(True)
    # plt.show()

    # IPython.embed()

    # Take a look at the distribution of z

    # This shouldn't use much processing! and yet it still does.
