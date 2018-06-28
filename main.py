# Try to capture depth data and color data.

# First import the library
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import IPython

# Create a context object. This object owns the 
# handles to all connected realsense devices.
pipeline = rs.pipeline()
pipeline.start()

# plt.ion()

nx = 1280
ny = 720

plt.ion()

fig = plt.figure()
data = np.zeros((ny, nx))
im = plt.imshow(data, cmap="inferno", vmin=0, vmax=65535/32)
im.set_data(data)
fig.canvas.draw()
plt.show(block=False)

i = 0
while True:

    # This call waits until a new coherent set of frames is available 
    # on a device.
    frames = pipeline.wait_for_frames()
    depth_fr = frames.get_depth_frame()
    color_fr = frames.get_color_frame()
    if not depth_fr or not color_fr:
        continue

    # Get depth and color as numpy arrays.
    depth_data = np.asanyarray(depth_fr.as_frame().get_data())
    color_data = np.asanyarray(color_fr.as_frame().get_data())

    print(depth_data.shape)
    print(color_data.shape)

    print(i)
    i = i + 1
    # print(data.shape)
    # print(np.max(data))
    # print(np.min(data))

    # Plotting
    # im.set_data(data)
    # fig.canvas.draw()
    # plt.show(block=False)
