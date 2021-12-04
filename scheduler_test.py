import client
import socket
import os 
import time
import sys

import jetson.inference
import jetson.utils
import math
import cv2
import numpy as np

from src.camera import Camera
from src.projections import *
from networking.Client import Client

# Data communication process
if len(sys.argv)>1:
    argument = sys.argv[1]
    print(argument)
else:
    argument = "get data"
proc1 = client.Client("object-detection")
proc1.handshake()
proc1.communicate(argument)
time.sleep(3)
proc1.close()
