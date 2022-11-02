import c1c0_object_detection.arm.publish_arm_updates as arm
import c1c0_object_detection.kinematics.linear_pathing as alr


#startpos = [90, 90, 30, 45, 45, 120]
startpos = [1.57, 1.57, 0.52, 0.785, 0.785, 2.09]


print ("initializing arm")
arm.init_serial()
print ("Arm initialized. Now reading encoder values")
startpos = arm.read_encoder_values()
print ("read encoder values")
endpos = [20, 90, 30, 45, 45, 120]
iterations = 10
obstacles = []
print ("executing rrt")
arm_config, success = alr.linear_path_to_point(startpos, 1,1,1, obstacles, 5)
if not success:
    print ("plan failed")
else:
    print ("plan found")
for config in arm_config:
	#converted_array = alr.radians_to_degrees(config) # uncomment following for rev [::-1]
	converted_array = [20, 90, 30, 45, 45, 120]
	print("WRITING ARM CONFIG", converted_array)
	arm.publish_updates(converted_array, 1)
	print("arm config serial written")
	break
arm.close_serial()

