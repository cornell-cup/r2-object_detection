import numpy as np
from camera import Camera
import cv2

def get_depth_frame():
    with Camera(640,480) as cam:
      color_frame, depth_frame=cam.get_frames()
      depth_image = np.asanyarray(depth_frame.get_data())
      color_image = np.asanyarray(color_frame.get_data())
      b, g, r = cv2.split(color_image)
      depth_image = depth_image.astype('float32')
      r, g, b = r.astype('float32'), g.astype('float32'), b.astype('float32')
      print ((b.shape), (g.shape), (r.shape), (depth_image.shape))
      rgbd = cv2.merge([r, g, b, depth_image])
      print("Rgbd image shape:",rgbd.shape)
      print("Depth image shape:", depth_image.shape)
      print("Color image shape: ",color_image.shape)

      col_img, depth_img, dgr = cam.get_imgs_from_frames(color_frame, depth_frame)
    return  col_img, depth_img, rgbd
