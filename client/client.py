import numpy as np, time, cv2, os # Default Python Libraries
from ultralytics import YOLO

from client.config import * # Default Configuration Values

from typing import List, Mapping, Tuple, Set # Type Hinting

class Client:
    def __init__(self: any, disp: bool = DEFAULT_DISP, prnt: bool = DEFAULT_PRINT) -> 'Client':
        """
        Initializes an instance of client with a lot of default values and configurations.

        PARAMETERS
        ----------
        path         - The path to the directory containing the images to load.
        open         - Whether or not to open the camera.
        load         - Whether or not to load images from the specified path.
        disp         - Whether or not to display the image with bounding boxes.
        prnt         - Whether or not to print the results of the facial recognition.
        cache        - Whether or not to load encodings from the specified cache directory.
        cache_dir    - The directory to load encodings from.
        mappings     - The mappings to update with the added filenames.
        camera       - The camera to use.
        scale_factor - The scale factor to use when resizing images.
        """

        self.disp: bool = disp
        self.prnt: bool = prnt
        self.model = YOLO('yolo11m.pt')
        self.image = None

        self.task_map = {
            'file' : (lambda paths: self.detect_file(path=paths[0], display=disp)),
            'f' : (lambda paths: self.detect_file(path=paths[0], display=disp)),  # Pass in a file path and detect objects in the image

            'main_object' : (lambda _: self.detect_main(display=disp)),
            'm' : (lambda _ : self.detect_main(display=disp)), # Ask Scheduler for an image and return the name of the most prominent object

            'all' : (lambda _: self.detect_all(display=disp)),
            'a' : (lambda _: self.detect_all(display=disp)), # Ask Scheduler for an image and return the names of all detected objects

            'count' : (lambda _: self.detect_count(display=disp)),
            'c' : (lambda _: self.detect_count(display=disp)) # Ask Scheduler for an image and return the num
        }


    def interpret_task(self: any, task: str) -> any:
        """
        Returns the function corresponding to the task name given.

        PARAMETERS
        ----------
        task - The task to interpret.

        RETURNS
        -------
        any - The function corresponding to the task name given.
        """

        self.task_map.setdefault(task, lambda _: print("Unrecognized command, please try again"))
        return self.task_map[task]

    def detect_file(self, path: str = None, display: bool = False):
        if not path:
            return False
        results = self.model(path, show=display, stream=True)
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

            if self.prnt: print("Sorted class names by confidence:", sorted_names)
            return sorted_names

    def detect_all(self, display: bool = False):
        results = self.model(self.image, show=display, stream=True)
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

            if self.prnt: print("Sorted class names by confidence:", sorted_names)
            return sorted_names

    def detect_main(self, display: bool = True):
        results = self.model(self.image, show=display, stream=True)
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

            if self.prnt: print("Most prominent object:", sorted_names[0])
            return sorted_names[0]

    def detect_count(self, display: bool = True):
        results = self.model(self.image, show=display, stream=True)
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

            if self.prnt: print("Most prominent object:", sorted_names[0])
            return sorted_names[0]