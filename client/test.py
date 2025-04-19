from ultralytics import YOLO
import cv2
import argparse
from client.config import *

parser = argparse.ArgumentParser(description="Run YOLO detection on a source.")
parser.add_argument('--video', action='store_true', help='Use video instead of image')
parser.add_argument('--image', action='store_true', help='File path of the image you want to use')

args = parser.parse_args()

path = args.video

model = YOLO('yolo11m.pt')

if path:
     print(DEFAULT_CAMERA)
     results = model(1, show=True, stream=True)
else:
    image_path = input("What is the file path you would like: ")
    results = model(image_path, show=True, stream=True)

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

