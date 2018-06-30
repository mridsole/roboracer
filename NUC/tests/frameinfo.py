from procframe.frameinfo import FrameInfo
import cv2
import h5py

# Load some data

color_cap = cv2.VideoCapture('../data/color-obs2.avi')
depth_data = h5py.File('../data/depth-obs2.h5', 'r')['depth']

i = 0
while True:
    ret, color_frame = color_cap.read()
    if not ret: break

    depth_frame = depth_data[i,:,:]
    finfo = FrameInfo(color_frame, depth_frame)

    i += 1
