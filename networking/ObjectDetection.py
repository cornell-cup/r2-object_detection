import cv2
from typing import List
import numpy as np

class ObjectDetectionObject:
    """
    Instance Attributes:
    operations - name of the process completed/to be completed
    data - input/output of the given function

    """

    def __init__(self, operations, data):
        # if we have more things we want to compute, we can name what process
        # to be run on the data
        # Sender -- [functions], [corresponding data] -->
        # <-- [function], [response data] -- Responder (runs function on data)
        # Concerns:
        # - Have to know what function name should correspond to what actual
        # function
        # - sender has to know what type of data the responder expects for the function
        self.operations : List[str] = operations
        self.data : List = data

    # def package_object(self):
    #     """
    #     Example Output
    #     {
    #         "function" : "Object Inference",
    #         "data" = [RGB_img]
    #     }
    #     """
    #     pass
    def operations(self):
        return self.operations

    def data(self):
        return self.data

    def __str__(self) -> str:
        s = ""
        for i in range(len(self.operations)):
            s = s.join([self.operations[i], ": ", str(self.data[i]), "\n"])
        # op name, type of corr data
        return s
