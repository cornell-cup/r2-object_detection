from kinematics import linear_pathing as alr
import arm.publish_arm_updates as arm 

startpos = arm.read_encoder_values()
endpos = []
iterations = 10
obstacles = []
alr.linear_rrt(startpos, endpos, obstacles, iterations, True)