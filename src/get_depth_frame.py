import numpy as np
from camera import Camera

def get_depth_frame():
    with Camera(640,480) as cam:
      color_frame, depth_frame=cam.get_frames()
      dist_matrix = np.zeros((480,640))
      for y in range(480):
        for x in range(640):
          dist = depth_frame.get_distance(x, y)
          dist_matrix[y,x] = dist
      col_img, depth_img, dgr = cam.get_imgs_from_frames(color_frame, depth_frame)
    return dist_matrix, col_img
"""    
    file = open("dist_matrix1.txt", "w+")
      for rows in dist_matrix:
        for element in rows:
          content = str(element)
          file.write(content + " ")
        file.write("\n")
      file.close()
      """
