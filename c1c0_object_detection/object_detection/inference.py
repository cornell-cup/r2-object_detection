import jetson.inference
import jetson.utils
import cv2

class Inference:
    """ Uses jetson's inference model to detect objects in a scene

    Attributes:
        net
        display
    """

    def __init__(self):
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
        self.display = jetson.utils.videoOutput("my_video.mp4") # 'my_video.mp4' for file
        self.coco_dataset = []
        with open('./c1c0_object_detection/object_detection/coco.txt', 'r') as coco_file: 
            self.coco_dataset=[object.strip('\n') for object in coco_file.readlines()]

    # TODO: currently just takes the first object detected
    def detect_object(self, img, obj_of_interest, display=False):
        """
        Runs inference on an image to search for a specific object
        Args:
            img is a numpy color image
            obj_of_interest is the name of the object to search for

        Returns:
            isFound is whether the specified object was found
            bbox is the coordinates of image where the object was detected
            color_img_cuda - an image with all detections found
            TODO: what if same object type was detected multiple times?
        """
        WIDTH, HEIGHT = img.shape[1], img.shape[0]
        detections = []
        color_img_cuda = jetson.utils.cudaFromNumpy(img)
        detections = self.net.Detect(color_img_cuda)
        detection = None

        target_id = self.coco_dataset.index(obj_of_interest)
        found_target=False
        for obj in detections:
            if obj.ClassID == target_id:
                found_target = True
                detection = obj
                break
        if not detections or not found_target:
            return False, None, None, None, None

        top, bottom, left, right = detection.Top, detection.Bottom, detection.Left,detection.Right
        top, bottom, left, right = round(top), round(bottom), round(left), round(right)
        detection_img = jetson.utils.cudaToNumpy(color_img_cuda, WIDTH, HEIGHT, 4)
        
        if display:
            self.display_detections(detection_img, WIDTH, HEIGHT)
        
        return True, top, bottom, left, right
    
    def display_detections(self, color_img_cuda, WIDTH, HEIGHT):
        # display detections
        print ("displaying detections... " )

        cv2img = cv2.cvtColor(color_img_cuda, cv2.COLOR_RGBA2BGR)
        cv2.imshow('detections', cv2img)
