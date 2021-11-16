"""
Test that displays an arm configuration using kinpy.
Made by Simon Kapen in Fall 2021.
"""

import kinpy as kp
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d

chain = kp.build_chain_from_urdf(open("models/SimpleArmModelforURDF.urdf").read())

# Generate locations of each joint. Input angles here.
th = {'Rev2': 0, 'Rev3': 0, 'Rev4': 0, 'Rev5': 0, 'Rev6': math.pi/2}
ret = chain.forward_kinematics(th)


print(chain.get_joint_parameter_names())
print(chain)
print(ret)
print(ret['base_link'])


ax = plt.axes(projection='3d')


# Draw lines in between each joint
# Only takes the first 4 joints because the last one is opening/closing the claw, which this part of the code
# isn't concerned with
v1 = []
v2 = []
v3 = []
v4 = []
for i in range(3):
    v1.append([ret['link2_1'].pos[i], ret['link3_1'].pos[i]])
    v2.append([ret['link3_1'].pos[i], ret['link4_1'].pos[i]])
    v3.append([ret['link4_1'].pos[i], ret['link5_1'].pos[i]])
    v4.append([ret['link5_1'].pos[i], ret['hand_1'].pos[i]])


ax.plot(v1[0], v1[1], zs=v1[2], color='red')
ax.plot(v2[0], v2[1], zs=v2[2], color='green')
ax.plot(v3[0], v3[1], zs=v3[2], color='blue')
ax.plot(v4[0], v4[1], zs=v4[2], color='orange')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_xlim3d(-.3, .3)
ax.set_ylim3d(-.3, .3)
ax.set_zlim3d(-.3, .3)

plt.show()






