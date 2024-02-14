import c1c0_object_detection.arm.publish_arm_updates as arm

import time

arm.init_serial()

converted_array = [0, 0, 0, 320, 0, 0]
arm.publish_updates(converted_array, 1)

arm.close_serial()

