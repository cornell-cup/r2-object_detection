# r2-object_detection

Our goal is to be able to detect objects dynamically with the camera on the robot.

# Requirements
See [`requirements.txt`](./requirements.txt) for Python dependencies.

Additionally, install Protobuf for your system and run:
`protoc object_detection/protos/*.proto --python_out=.`

To install protobuf, [go here](https://github.com/protocolbuffers/protobuf/releases), download the appropriate zip for your OS (not language necessarily), unzip it to a folder, and set your PATH to also point to the bin.

Later, in order to avoid an import error, run the following line (must do for each new terminal opened): `export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1` and activate the virtual environment with the following line: `source venv/bin/activate` 
# CREDITS
Tensorflow's research models for a frozen Object Detection model (SSD Mobilenet V1 COCO, [retrieved from previous commit](https://github.com/tensorflow/models/blob/d41fc8f4437e585656ac50a2d73bcc8146e05579/research/object_detection/g3doc/detection_model_zoo.md))
Tensorflow's research models for their `object_detection` library found under `research/` in [models](https://github.com/tensorflow/models/tree/master/research/object_detection).
Tensorflow's research models for the labels provided by the [`object_detection` library](https://github.com/tensorflow/models/blob/master/research/object_detection/data/mscoco_label_map.pbtxt).
