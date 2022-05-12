import c1c0_object_detection.arm.publish_arm_updates as arm
import c1c0_object_detection.kinematics.linear_rrt as alr

arm.init_serial()
startpos = arm.read_encoder_values()
endpos = [0, 45, 0, 0, 0, 0]
iterations = 10
obstacles = []
arm_config = alr.generate_linear_path(startpos, endpos, iterations)
for config in arm_config:
	converted_array = alr.radians_to_degrees(config) # uncomment following for rev [::-1]
	print("WRITING ARM CONFIG", converted_array)
	arm.publish_updates(converted_array, 1)
	print("arm config serial written")
	break
arm.close_serial()
