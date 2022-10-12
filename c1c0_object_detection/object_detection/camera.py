import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

class Camera:
  """
  Instance Variables:
    w - width of the streamed camera frames
    h - height of the streamed camera frames
    pipeline - pyrs camera object with specific configuration settings
    depth_scale - unit of z to meters
    clipping_distance - objects past this distance are turned to gray
    align - used to align frames together
  """

  def __init__(self, w=640, h=480, clipping_dist_meters=1):
    self.w, self.h = w, h
    # Create a pipeline
    self.pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    config = rs.config()
    config.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
    # Start streaming
    profile = self.pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    self.depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", self.depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    self.clipping_distance = clipping_dist_meters / self.depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    self.align = rs.align(align_to)

    #set up depth post-processing filters
    self.decimation_filter = rs.decimation_filter() #default is 2
    self.spatial_filter = rs.spatial_filter(smooth_alpha=.6, smooth_delta=20, magnitude=2, hole_fill=0)
    self.hole_fill_filter = rs.hole_filling_filter() #default is fill according to neighboring pixel farthest from sensor

  def get_frames(self):
    """
    Gets the color and depth frames. Both of which are  pyrealsense objects. 
    """
    # Get frameset of color and depth
    frames = self.pipeline.wait_for_frames()

    #post process the depth
    frames = self.decimation_filter.process(frames).as_frameset()
    frames = self.spatial_filter.process(frames).as_frameset()
    frames = self.hole_fill_filter.process(frames).as_frameset()

    # Align the depth frame to color frame
    aligned_frames = self.align.process(frames)

    # Get aligned frames
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    print("depth_frame", depth_frame)
    
    return color_frame, depth_frame

  def get_imgs_from_frames(self, color_frame, depth_frame, display=False):
    # get_data * depth_scale to get actual distance?
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # TODO: Currently not profile = self.pipeline.start(config) actually being used
    # Remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    # depth image is 1 channel, color is 3 channels
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
    bg_removed = np.where((depth_image_3d > self.clipping_distance) | (
        depth_image_3d <= 0), grey_color, color_image)
    # Render images
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # make dgr image
    b, g, r = cv2.split(color_image)
    print("colorizer mode: ", 2)
    colorizer = rs.colorizer(2)
    depth_colorized = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())
    dgr = cv2.merge((depth_colorized[:, :, 0], g, r))

    if display:
      self.display_images(color_image, depth_colorized)

    return color_image, depth_colorized, dgr

  def display_images(self, color_img, depth_img):
    # colorized depth image
    cv2.imshow("original", cv2.cvtColor(color_img, cv2.COLOR_RGBA2BGR))
    cv2.imshow("white to black depth", depth_img)
  
  def __enter__(self):
      return self

  def __exit__(self, exc_type, exc_val, exc_tb):
      self.stop()

  def stop(self):
    self.pipeline.stop()
