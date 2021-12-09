#from kinematics import assuming_linearity_rrt as alr
import publish_arm_updates as arm 

arm.init_serial('/dev/ttyTHS1', 9600)
startpos = arm.read_encoder_values()
endpos = [0, 50, 12, 20, 20, 20]
#iterations = 10
#obstacles = []
#alr.linear_rrt(startpos, endpos, obstacles, iterations, True)

arm.publish_updates([endpos], 1)
