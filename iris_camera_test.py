import pyrealsense2 as rs
import numpy as np
import os
import six.moves.urllib as urllib
import sys
print(sys.version_info)
import tarfile
import tensorrt as tf
import zipfile
import cv2
import math

from distutils.version import StrictVersion
from collections import defaultdict
from io import StringIO
#from PIL import Image
import time

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

try:
    for i in range(5):
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        print(depth_image)
        test = cv2.resize(color_image, (1920, 1080))
        test_depth = cv2.resize(depth_image, (1920, 1080))
        #cv2.imwrite('color_image.jpg', test)
        #cv2.imwrite('depth_image.jpg', test_depth)
        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())        
        colorized_depth = cv2.resize(colorized_depth, (1920, 1080))
        #cv2.imwrite('colorized_depth_image.jpg', colorized_depth)
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #print((coordinates[1]*bg_removed.shape[1], coordinates[0]*bg_removed.shape[0]))
        #cv2.rectangle(color_image, (int(coordinates[1]*bg_removed.shape[1]), int(coordinates[0]*bg_removed.shape[0])), (int(coordinates[3]*bg_removed.shape[1]), int(coordinates[2]*bg_removed.shape[0])), 100, 5)
        #print("1")
        # images = np.hstack((depth_colormap, color_image))
        # print("2")
        # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        # print("3")
        # cv2.imshow('Align Example', color_image)
        # print("4")
        # test2 = cv2.resize(depth_colormap, (1920, 1080))
        # print("5")
        # if(math.sqrt((inx+ 0.06985)**2 + ( iny - 0.10795)**2 + (inz-0.05715)**2) < .7):
        #     kinematics(inx+ 0.06985, iny - 0.10795, inz-0.05715);
        cv2.imshow('color_image.jpg', test)
        cv2.imshow('depth_image.jpg', colorized_depth)
        key = cv2.waitKey(0)
        
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
