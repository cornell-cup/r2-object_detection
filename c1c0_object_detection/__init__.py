"""
Example Usage:
from c1c0_object_detection.object_detection.camera import Camera
from c1c0_object_detection.object_detection.inference import Inference
from c1c0_object_detection.object_detection.grasping import Grasping
import c1c0_object_detection.networking as networking
import c1c0_object_detection.arm as arm
camera = Camera(640, 480)
inf = Inference()
grasping = Grasping()
networking_client = networking.Client()
arm_client = arm.Client()
- camera.get_frames() -> color_frame, depth_frame ->
- camera.get_imgs_from_frames(color_frame, depth_frame) -> color_img, depth_img, dgr ->
- inf.detect_object(color_img, obj_of_interest) -> True, top, bot, left, right ->
- grasping.locate_object(dgr, (top, bot, left, right), depth_frame) -> True, True, coord1, coord2 ->
- networking_client.send_data([coord1, coord2]) # but must be of form [[[x], [y], [z]], ...] -> arm_config
- arm_client.send_data(arm_config) -> (actuate, any additional sensing?) ->
In between each step, scheduler can decide what to do next (eg. if bad camera read,
read again, or ask path planning to move closer)
"""

from . import kinematics, networking, object_detection