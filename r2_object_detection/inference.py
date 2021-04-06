import cv2
import pyrealsense2 as rs
import numpy as np
import sys
import tensorflow as tf
from tensorflow.python.compiler.tensorrt import trt_convert as trt
import math
from object_detection.utils import label_map_util
from object_detection.utils import ops as utils_ops
from cam_grasp_integration import get_grasp_on_image
from utitlity_constants import PATH_TO_FROZEN_GRAPH, PATH_TO_LABELS
import time

tf.compat.v1.disable_eager_execution()

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

# First deserialize your frozen graph:
with tf.compat.v1.Session() as sess:
    with tf.compat.v2.io.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as f:
        frozen_graph = tf.compat.v1.GraphDef()
        frozen_graph.ParseFromString(f.read())
outputs = ['num_detections', 'detection_boxes', 'detection_scores', 'detection_classes']
trt_graph = trt.create_inference_graph(
    input_graph_def=frozen_graph, outputs=outputs, max_batch_size=1, max_workspace_size_bytes=1 << 3,
    precision_mode="FP32", minimum_segment_size=5
)


def run_inference_for_single_image(image):
    with tf.compat.v1.Graph().as_default() as g:
        tf.import_graph_def(trt_graph, name='')
        # Get handles to input and output tensors
        inputs_ = g.get_tensor_by_name('input_images:0')
        outputs_ = [o+':0' for o in outputs]
        all_tensor_names = outputs_
        tensor_dict = {}
        for tensor_key in outputs:
            tensor_name = tensor_key + ':0'
            if tensor_name in all_tensor_names:
                tensor_dict[tensor_key] = g.get_tensor_by_name(
                    tensor_name)
        if 'detection_masks' in tensor_dict:
            # The following processing is only for single image
            detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
            detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
            # Re-frame is required to translate mask from box coordinates to image coordinates and fit the image size.
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
        image_tensor = g.get_tensor_by_name('image_tensor:0')

        # Run inference
        t1 = time.time()
        with tf.compat.v1.Session() as sess:
            output_dict = sess.run(tensor_dict, feed_dict={image_tensor: np.expand_dims(image, 0)})
        t2 = time.time()
        print("runtime")
        print(t2 - t1)

        # all outputs are float32 numpy arrays, so convert types as appropriate
        classes = output_dict['detection_classes'][0]
        scores = output_dict['detection_scores'][0]
        print(output_dict['detection_classes'])
        print(output_dict['detection_scores'])
        output_classes = []
        output_coord = []
        for i in list(range(len(classes))):
            if scores[i] > 0.5:
                output_classes.append(category_index[classes[i]]['name'])
                output_coord.append(output_dict['detection_boxes'][0][i])

        return output_coord, output_classes


def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1  # 1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while True:
            # Get frame_set of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            print(depth_image)
            test = cv2.resize(color_image, (1920, 1080))
            test_depth = cv2.resize(depth_image, (1920, 1080))
            cv2.imwrite('test.jpg', test)

            output_coord, output_classes = run_inference_for_single_image(test)
            try:

                # coordinates = output_coord[output_classes.index("bottle")]
                coordinates = output_coord[0]
                x = (coordinates[1] + coordinates[3]) / 2 * 1920
                y = (coordinates[0] + coordinates[2]) / 2 * 1080
                inx = test_depth[int(y)][int(x)] * depth_scale

                print("grasp coordinates", get_grasp_on_image(aligned_frames, coordinates))
            except:
                print('not found')
                coordinates = [0, 0, 0, 0]
                x = 0
                y = 0
                inx = 0
            print(output_classes)
            # print(coordinates)

            iny = math.tan(math.radians(43.5)) * inx * 2 * (x - 960) / 1920
            inz = math.tan(math.radians(29)) * inx * 2 * (y - 540) / 1080
            print(inx)
            print(inx, iny, inz)

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack(
                (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            # Render images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            print((coordinates[1] * bg_removed.shape[1], coordinates[0] * bg_removed.shape[0]))
            cv2.rectangle(color_image,
                          (int(coordinates[1] * bg_removed.shape[1]), int(coordinates[0] * bg_removed.shape[0])),
                          (int(coordinates[3] * bg_removed.shape[1]), int(coordinates[2] * bg_removed.shape[0])), 100,
                          5)
            # print("1")
            # images = np.hstack((depth_colormap, color_image))
            # print("2")
            # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            # print("3")
            # cv2.imshow('Align Example', color_image)
            # print("4")
            # test2 = cv2.resize(depth_colormap, (1920, 1080))
            # print("5")
            # # if math.sqrt((inx + 0.06985) ** 2 + (iny - 0.10795) ** 2 + (inz - 0.05715) ** 2) < .7:
            # #     kinematics(inx + 0.06985, iny - 0.10795, inz - 0.05715);
            # # cv2.imshow('potato', test2)
            # print("6")
            # key = cv2.waitKey(1)

            # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q') or key == 27:
            #    cv2.destroyAllWindows()
            #    break
    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()
