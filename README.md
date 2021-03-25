# r2-object_detection

Our goal is to be able to detect objects dynamically with the camera on the robot.

# Requirements
See [`requirements.txt`](./requirements.txt) for Python dependencies.

Additionally, install Protobuf for your system and run:
`protoc object_detection/protos/*.proto --python_out=.`

# CREDITS
Tensorflow's research models for a frozen Object Detection model (SSD Mobilenet V1 COCO, [retrieved from previous commit](https://github.com/tensorflow/models/blob/d41fc8f4437e585656ac50a2d73bcc8146e05579/research/object_detection/g3doc/detection_model_zoo.md))
Tensorflow's research models for their `object_detection` library found under `research/` in [models](https://github.com/tensorflow/models/tree/master/research/object_detection).
Tensorflow's research models for the labels provided by the [`object_detection` library](https://github.com/tensorflow/models/blob/master/research/object_detection/data/mscoco_label_map.pbtxt).
