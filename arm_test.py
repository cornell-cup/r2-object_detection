from kinematics import assuming_linearity_rrt as alr
import arm.publish_arm_updates as arm 

startpos = arm.read_encoder_values()
endpos = []
iterations = 10
obstacles = []
alr.linear_rrt(startpos, endpos, iterations, obstacles, True)