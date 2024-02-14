import c1c0_object_detection.arm.publish_arm_updates as arm

import c1c0_object_detection.kinematics.linear_pathing as alr

import c1c0_object_detection.kinematics.arm_plot as arm_plot

import matplotlib.pyplot as plt

import time

ax = plt.axes(projection='3d')
arm_plot.configure_graph(ax)

startpos = [0, 0, 0, 0, 0, 0]
#startpos = [90, 90, 30, 45, 45, 120]
#startpos = [1.57, 1.57, 0.52, 0.785, 0.785, 2.09]


print ("initializing arm")
arm.init_serial()
print ("Arm initialized. Now reading encoder values")
startpos = arm.read_encoder_values()
print ("read encoder values")
#endpos = [20, 90, 30, 45, 45, 120]
obstacles = []
print ("executing rrt")
arm_config, success = alr.linear_path_to_point(startpos, .1,.1,.25, obstacles, 2)
if not success:
    print ("plan failed")
else:
    print ("plan found")

ax = plt.axes(projection='3d')
arm_plot.configure_graph(ax)
arm_plot.plot_arm_configs(ax, arm_config, [])
plt.show()

for config in arm_config:
    converted_array = alr.radians_to_degrees(config) # uncomment following for rev [::-1]
    print("WRITING ARM CONFIG", converted_array)
    arm.publish_updates(converted_array, 1)
    print("arm config serial written")
    time.sleep(1)
arm.close_serial()

