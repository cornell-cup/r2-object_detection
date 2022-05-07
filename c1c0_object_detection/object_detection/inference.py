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

        if not detections:
            print('Nothing detected')
            return False, None

        detection = detections[0]
        top, bottom, left, right = detection.Top, detection.Bottom, detection.Left,detection.Right
        top, bottom, left, right = round(top), round(bottom), round(left), round(right)
        detection_img = jetson.utils.cudaToNumpy(color_img_cuda, WIDTH, HEIGHT, 4)
        
        if display:
            self.display_detections(detection_img, WIDTH, HEIGHT)
        
        return True, top, bottom, left, right
    
    def display_detections(color_img_cuda, WIDTH, HEIGHT):
        # display detections
        print ("displaying detections... " )
        cv2imgRGBA = jetson.utils.cudaToNumpy(color_img_cuda, WIDTH, HEIGHT, 4)
        cv2img = cv2.cvtColor(cv2imgRGBA, cv2.COLOR_RGBA2BGR)
        cv2.imshow('detections', cv2img)