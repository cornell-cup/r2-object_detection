import numpy as np
from camera import Camera
import cv2

def get_depth_frame():
    with Camera(640,480) as cam:
      color_frame, depth_frame=cam.get_frames()

      # Normalize depth image to a value between 0 and 255
      depth_image = np.asanyarray(depth_frame.get_data())
      #depth_img_norm = np.zeros(depth_image.shape)
      #depth_image = cv2.normalize(depth_image, depth_img_norm, 0, 255, cv2.NORM_MINMAX)
      cv2.imshow("other depth image: ", depth_image)


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
      cv2.imshow("Color image: ", col_img)
      
      cv2.imshow("depth image: ", depth_img)
      key = cv2.waitKey(0)

      if key & 0xFF == ord('q') or key == 27:
          cv2.destroyAllWindows()

      if key & 0xFF == ord('r'):
          cv2.destroyAllWindows()
      print (depth_img.shape)
    return  col_img, depth_img, rgbd
