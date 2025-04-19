import time # Default Python Libraries
from ultralytics import YOLO

from client.config import * # Default Configuration Values
from client.camera import Camera

class Client:
    def __init__(self: any, disp: bool = DEFAULT_DISP, prnt: bool = DEFAULT_PRINT, 
                 open: bool = DEFAULT_OPEN, camera: str = DEFAULT_CAMERA) -> 'Client':
        """
        Initializes an instance of client with a lot of default values and configurations.

        PARAMETERS
        ----------
        open         - Whether or not to open the camera.
        disp         - Whether or not to display the image with bounding boxes.
        prnt         - Whether or not to print the results of the facial recognition.
        camera       - The camera to use.
        """

        self.disp: bool = disp
        self.prnt: bool = prnt
        self.open: bool = open
        self.camera = Camera(camera) if self.open else None 
        self.model = YOLO('yolo11m.pt') # YOLO object detection model initialization
        self.image = None

        self.task_map = {
            'file' : (lambda paths: self.detect_file(path=paths[0], display=disp)),
            'f' : (lambda paths: self.detect_file(path=paths[0], display=disp)),  # Pass in a file path and detect objects in the image

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
        if not path: return False
        results = self.model(path, show=display, stream=True)
        total_names = []

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                confs = boxes.conf.cpu().numpy()       # Confidence scores
                classes = boxes.cls.cpu().numpy()      # Class indices
                names = result.names                   # Class index -> name mapping

                # Create a sorted list of (name, confidence) pairs
                name_conf_pairs = [(names[int(cls)], float(conf)) for cls, conf in zip(classes, confs)]
                sorted_pairs = sorted(name_conf_pairs, key=lambda x: x[1], reverse=True)
                sorted_names = [name for name, _ in sorted_pairs]

                for name in sorted_names:
                    total_names.append(name)
        
        if self.prnt: 
            print("Sorted names:", total_names)
            time.sleep(3)
        return total_names

    def detect_all(self, display: bool = False):
        image = self.camera.adjust_read() if self.open else self.image
        results = self.model(image, show=display, stream=True)
        total_names = []

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                confs = boxes.conf.cpu().numpy()       # Confidence scores
                classes = boxes.cls.cpu().numpy()      # Class indices
                names = result.names                   # Class index -> name mapping

                # Create a sorted list of (name, confidence) pairs
                name_conf_pairs = [(names[int(cls)], float(conf)) for cls, conf in zip(classes, confs)]
                sorted_pairs = sorted(name_conf_pairs, key=lambda x: x[1], reverse=True)
                sorted_names = [name for name, _ in sorted_pairs]

                for name in sorted_names:
                    total_names.append(name)
            
            if self.prnt: 
                print("Sorted names:", total_names)
                time.sleep(3)
            return total_names

    def detect_count(self, display: bool = True):
        image = self.camera.adjust_read() if self.open else self.image
        results = self.model(image, show=display, stream=True)
        total_names = []

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                confs = boxes.conf.cpu().numpy()       # Confidence scores
                classes = boxes.cls.cpu().numpy()      # Class indices
                names = result.names                   # Class index -> name mapping

                # Create a sorted list of (name, confidence) pairs
                name_conf_pairs = [(names[int(cls)], float(conf)) for cls, conf in zip(classes, confs)]
                sorted_pairs = sorted(name_conf_pairs, key=lambda x: x[1], reverse=True)
                sorted_names = [name for name, _ in sorted_pairs]

                for name in sorted_names:
                    total_names.append(name)

        if self.prnt: 
            print("Number of objects:", len(total_names))
            time.sleep(3)
        return len(total_names)