import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle, ConnectionPatch
from matplotlib.collections import PatchCollection
import os


plt.ion()
fig, ax = plt.subplots()
sc_l = ax.scatter([],[], s=10, color=(0., 0., 0.9))
sc_r = ax.scatter([],[], s=10, color=(0.8, 0.8, 0))
sc_obs = ax.scatter([],[], s=10)
# ax.axis('equal')
plt.xlim(-2, 2)
plt.ylim(-1, 3)
plt.grid(True)
plt.draw()


load_i = 0
path = 'framedata/'

while True:

    # Load points
    path_l = os.path.join(path, 'l_' + str(load_i) + '.npy')
    path_r = os.path.join(path, 'r_' + str(load_i) + '.npy')
    path_o = os.path.join(path, 'obs_' + str(load_i) + '.npy')
    
    # If no more data, quit
    if not os.path.exists(path_l): exit()

    pts_l = np.load(path_l)
    pts_r = np.load(path_r)
    pts_obs = np.load(path_o)

    load_i += 1

    # Plot left points
    sc_l.set_offsets(np.c_[pts_l[:,0], pts_l[:,1]])

    # Plot right points
    sc_r.set_offsets(np.c_[pts_r[:,0], pts_r[:,1]])

    # Plot obstacle poitns
    sc_obs.set_offsets(np.c_[pts_obs[:,0], pts_obs[:,1]])

    # Draw
    fig.canvas.draw_idle()
    plt.pause(0.001)
