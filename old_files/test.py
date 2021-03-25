import numpy as np
import os
import sys
print(sys.version_info)
import tensorflow as tf

from distutils.version import StrictVersion
import matplotlib
matplotlib.use('TkAgg')
matplotlib.rcParams["backend"] = "TkAgg"
import matplotlib.pyplot as plt
print(matplotlib.get_backend())
plt.switch_backend("TkAgg")
from PIL import Image
import time

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import ops as utils_ops

if StrictVersion(tf.__version__) < StrictVersion('1.9.0'):
  raise ImportError('Please upgrade your TensorFlow installation to v1.9.* or later!')


# This is needed to display the images.
#%matplotlib inline
#exec(%matplotlib inline)

from utils import label_map_util

from utils import visualization_utils as vis_util
# What model to download.
#MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
MODEL_NAME = 'faster_rcnn_nas_coco_2018_01_28'
#MODEL_FILE = MODEL_NAME + '.tar.gz'
#DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
#PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'
PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'
# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')

#opener = urllib.request.URLopener()
#opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
#tar_file = tarfile.open(MODEL_FILE)
#for file in tar_file.getmembers():
#  file_name = os.path.basename(file.name)
#  if 'frozen_inference_graph.pb' in file_name:
#    tar_file.extract(file, os.getcwd())

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
        
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
      

def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)
#
#PATH_TO_TEST_IMAGES_DIR = 'test_images'
#TEST_IMAGE_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR, 'image{}.jpg'.format(i)) for i in range(1, 5) ]
##print(TEST_IMAGE_PATHS)
## Size, in inches, of the output images.
IMAGE_SIZE = (12, 8)
def run_inference_for_single_image(image, graph):
  with graph.as_default():
    with tf.Session() as sess:
      # Get handles to input and output tensors
      ops = tf.get_default_graph().get_operations()
      all_tensor_names = {output.name for op in ops for output in op.outputs}
      tensor_dict = {}
      for key in [
          'num_detections', 'detection_boxes', 'detection_scores',
          'detection_classes', 'detection_masks'
      ]:
        tensor_name = key + ':0'
        if tensor_name in all_tensor_names:
          tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
              tensor_name)
      if 'detection_masks' in tensor_dict:
        # The following processing is only for single image
        detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
        detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
        # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
        real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
        detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
        detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
            detection_masks, detection_boxes, image.shape[0], image.shape[1])
        detection_masks_reframed = tf.cast(
            tf.greater(detection_masks_reframed, 0.5), tf.uint8)
        # Follow the convention by adding back the batch dimension
        tensor_dict['detection_masks'] = tf.expand_dims(
            detection_masks_reframed, 0)
      image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

      # Run inference
      t1=time.time()
      output_dict = sess.run(tensor_dict,
                             feed_dict={image_tensor: np.expand_dims(image, 0)})
      t2=time.time()
      print("runtime")
      print(t2-t1)

      # all outputs are float32 numpy arrays, so convert types as appropriate
      output_dict['num_detections'] = int(output_dict['num_detections'][0])
      output_dict['detection_classes'] = output_dict[
          'detection_classes'][0].astype(np.uint8)
      output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
      output_dict['detection_scores'] = output_dict['detection_scores'][0]
      if 'detection_masks' in output_dict:
        output_dict['detection_masks'] = output_dict['detection_masks'][0]
  return output_dict


#for image_path in TEST_IMAGE_PATHS:
#  image = Image.open(image_path)
##  # the array based representation of the image will be used later in order to prepare the
##  # result image with boxes and labels on it.
#  image_np = load_image_into_numpy_array(image)
##  # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
#  image_np_expanded = np.expand_dims(image_np, axis=0)
##  # Actual detection.
#  output_dict = run_inference_for_single_image(image_np, detection_graph)
##  # Visualization of the results of a detection.
#  vis_util.visualize_boxes_and_labels_on_image_array(
#      image_np,
#      output_dict['detection_boxes'],
#      output_dict['detection_classes'],
#      output_dict['detection_scores'],
#      category_index,
#      instance_masks=output_dict.get('detection_masks'),
#      use_normalized_coordinates=True,
#      line_thickness=8)
#  plt.figure(figsize=IMAGE_SIZE)
#  plt.imshow(image_np)
#  print(output_dict['detection_classes'])
#  print(type(image_np))
#  output=[]
#  classes= output_dict['detection_classes']
#  scores=output_dict['detection_scores']
#  for i in list(range(len(classes))):
#      if scores[i]>0.5:
#          output.append(category_index[classes[i]]['name'])
#  print(output)
##  #print("IMAGE PATH")
##
##  #print(image_path)
##  #f=open("test_images/image1.txt")
##  f=open(image_path[:len(image_path)-3]+"txt")
##  l=f.read()
##  l=l[2:]
##  l=l[:len(l)-3]
##  #print("l")
##  #print(l)
##  if str(l)==str(output[0]):
##      print("prediction matches verification")
#
#
#
#  img = Image.fromarray(image_np, 'RGB')
#  img.show()
#  plt.show()
#  plt.show(block=True)

def infer_and_match(image_path):
    print("Image received! starting inference and verification...")
    image = Image.open(image_path)
    # the array based representation of the image will be used later in order to prepare the
    # result image with boxes and labels on it.
    image_np = load_image_into_numpy_array(image)
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    # Actual detection.
    t1=time.time()
    output_dict = run_inference_for_single_image(image_np, detection_graph)
    t2=time.time()
    print(t2-t1)
    # Visualization of the results of a detection.
    (box_image) = vis_util.visualize_boxes_and_labels_on_image_array(    image_np,
                                                               output_dict['detection_boxes'],
                                                               output_dict['detection_classes'],
                                                               output_dict['detection_scores'],
                                                               category_index,
                                                               instance_masks=output_dict.get('detection_masks'),
                                                               use_normalized_coordinates=True,
                                                               line_thickness=8)
    plt.figure(figsize=IMAGE_SIZE)
    plt.imshow(image_np)
    #print(output_dict['detection_classes'])
    #print(type(image_np))
    output=[]
    classes= output_dict['detection_classes']
    scores=output_dict['detection_scores']
    for i in list(range(len(classes))):
        if scores[i]>0.5:
            output.append(category_index[classes[i]]['name'])
    print(output)
    return(output_dict['detection_boxes'],
            output_dict['detection_classes'], box_image)
    
    #verif=verif[:len(verif)-3]
    #print("output is ")
    #print(output[0])
    #print("verification is")
    

    img = Image.fromarray(image_np, 'RGB')
    img.show()
