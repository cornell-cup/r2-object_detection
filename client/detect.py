import numpy as np, os, cv2, face_recognition # Default Python Libraries

from client.config import * # Default Configurations

from typing import Mapping, Tuple, List, MutableMapping # Type Hinting

from ultralytics import YOLO # Object Detection Model

class Detect:
    def __init__(self):
        self.model = YOLO('yolo11m.pt')

    def most_prominent(path: str = None):
        if path:
            results = self.model(path, show=True, stream=True)
            return results
        results = self.model(DEFAULT_CAMERA, show=True, stream=True)
        return results