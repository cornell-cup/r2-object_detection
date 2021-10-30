import os
MODELS_DIR = 'ssd_mobilenet_v1_coco_2018_01_28/'
PATH_TO_FROZEN_GRAPH = MODELS_DIR + 'frozen_inference_graph.pb'
PATH_TO_OPTIMIZED_GRAPH = MODELS_DIR + 'optimized_inference_graph.'
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')