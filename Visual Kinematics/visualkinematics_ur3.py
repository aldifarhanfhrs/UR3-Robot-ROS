#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi

dh_params = np.array([  [0.1519,   0.,       pi/2,   0.],
                        [0.,      -0.24365,  0.,     0.],
                        [0.,      -0.21325,  0.,     0.],
                        [0.11235,  0.,       pi/2,   0.],
                        [0.08535,  0.,       -pi/2,  0.],
                        [0.0819,   0.,       0.,     0.]])
robot = RobotSerial(dh_params)

#theta = np.array([1.5 * pi, -pi, 0., 0., 0.5 * pi, 0.])
theta = np.array([(82*pi/180), (-59*pi/180), (59*pi/180), (187*pi/180), (-78*pi/180), (132*pi/180)])

f = robot.forward(theta)
robot.end_frame

print("-------forward-------")
print()
print("end frame t_4_4:")
print(f.t_4_4)
print("end frame xyz (position):")
print(f.t_3_1.reshape([3, ]))
print("end frame abc (rotation):")
print(f.euler_3)
print()
print("end frame rotational matrix:")
print(f.r_3_3)
print("end frame angle-axis(position):")
print(f.r_3)
print("end frame quaternion(orientation):")
print(f.q_4)

"""robot.show()
"""
"""xyz = np.array([[0.28127], [0.], [1.13182]])
abc = np.array([0.5 * pi, 0., pi])
end = Frame.from_euler_3(abc, xyz)
robot.inverse(end)

print("inverse is successful: {0}".format(robot.is_reachable_inverse))
print("axis values: \n{0}".format(robot.axis_values))"""
robot.show()