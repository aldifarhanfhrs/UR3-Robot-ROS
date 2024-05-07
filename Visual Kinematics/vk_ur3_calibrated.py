#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi
# d a alpha theta
dh_params = np.array([  [0.151870042425877,     -1.97103039747193e-05,       1.56939875554383,         5.80905700456741e-05],
                        [43.5763915196755,      -0.145291128343051,         -0.00450381354878373,     -0.933592307346293],
                        [-33.1459946000884,     -0.212884941597887,          0.00136956882024573,      1.00056403121008],
                        [-10.3179607200152,      1.97098925790105e-05,       1.57164738308229,        -0.0670626070627924 ],
                        [0.0854083420925052,    -2.40603567711256e-05,      -1.57133890016307,        -8.65344839424548e-05],
                        [0.0824923562001749,     0.,                         0.,                      -4.39717947174623e-05]])
robot = RobotSerial(dh_params)

theta = np.array([1.5 * pi, -pi, 0., 0., 0.5 * pi, 0.])
f = robot.forward(theta)
robot.end_frame

print("-------forward-------")
print("end frame t_4_4:")
print(f.t_4_4)
print("end frame xyz:")
print(f.t_3_1.reshape([3, ]))
print("end frame abc:")
print(f.euler_3)
print("end frame rotational matrix:")
print(f.r_3_3)
print("end frame quaternion:")
print(f.q_4)
print("end frame angle-axis:")
print(f.r_3)

"""robot.show()
"""
xyz = np.array([[0.28127], [0.], [1.13182]])
abc = np.array([0.5 * pi, 0., pi])
end = Frame.from_euler_3(abc, xyz)
robot.inverse(end)

print("inverse is successful: {0}".format(robot.is_reachable_inverse))
print("axis values: \n{0}".format(robot.axis_values))
robot.show()