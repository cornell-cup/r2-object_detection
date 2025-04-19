# File allowing users to test video functionality of object detection model

from ultralytics import YOLO
from client.config import *

model = YOLO('yolo11m.pt') 

results = model(0, show=True, stream=True)

for result in results:
    boxes = result.boxes
    if boxes is not None:
        confs = boxes.conf.cpu().numpy()       # Confidence scores
        classes = boxes.cls.cpu().numpy()      # Class indices
        names = result.names                   # Class index -> name mapping

        # Create a list of (name, confidence) pairs
        name_conf_pairs = [(names[int(cls)], float(conf)) for cls, conf in zip(classes, confs)]

        # Sort by confidence descending
        sorted_pairs = sorted(name_conf_pairs, key=lambda x: x[1], reverse=True)

        # Just names, sorted by confidence
        sorted_names = [name for name, _ in sorted_pairs]

        print("Sorted class names by confidence:", sorted_names)

