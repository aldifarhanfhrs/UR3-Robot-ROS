#!/usr/bin/env python3
import numpy as np
import tf.transformations as tf
from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion
from math import radians

from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *

#-------------------------------------------------------------------------------------------------
# Inisialisasi Parameter Denavit-Hartenberg (Teoritikal) (Hitung Manual)
# d (unit: meter)
global d1, a2, a3, a7, d4, d5, d6

d1 = 0.1519
a2 = -0.24365
a3 = -0.21325
#a7 = 0.075
d4 = 0.11235
d5 = 0.08535
d6 = 0.0819

d = np.array([0.1519, 0, 0, 0.11235, 0.08535, 0.0819])  # ur3 mm
a = np.array([0, -0.24365, -0.21325, 0, 0, 0])  # ur3 mm
alpha = np.array([pi / 2, 0, 0, pi / 2, -pi / 2, 0])  # ur3

# Convert dari UR Format ke ROS Pose Format

def ur2ros(ur_pose):
    """Convert UR format pose to ROS Pose format."""
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = ur_pose[0]
    ros_pose.position.y = ur_pose[1]
    ros_pose.position.z = ur_pose[2]

    # Orientation in UR format
    rx, ry, rz = ur_pose[3], ur_pose[4], ur_pose[5]

    # Convert orientation from Euler angles to quaternion
    cy = cos(rz * 0.5)
    sy = sin(rz * 0.5)
    cr = cos(rx * 0.5)
    sr = sin(rx * 0.5)
    cp = cos(ry * 0.5)
    sp = sin(ry * 0.5)

    ros_pose.orientation.x = cy * cr * sp - sy * sr * cp
    ros_pose.orientation.y = cy * sr * cp + sy * cr * sp
    ros_pose.orientation.z = sy * cr * cp - cy * sr * sp
    ros_pose.orientation.w = cy * cr * cp + sy * sr * sp
    
    return ros_pose

#-------------------------------------------------------------------------------------------------
# Transformasi Pose dari ROS Pose format ke np.array format

def ros2np(ros_pose):
    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y,
                                    ros_pose.orientation.z, ros_pose.orientation.w])
    
    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose

#-------------------------------------------------------------------------------------------------
# Transformasi Pose dari np.array format ke ROS Pose format

def np2ros(np_pose):
    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation 
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose

#Fungsi Matriks Transformasi Homogen
def HTM(i, theta):
    Rot_z = np.matrix(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = cos(theta[i])
    Rot_z[0, 1] = -sin(theta[i])
    Rot_z[1, 0] = sin(theta[i])

    Trans_z = np.matrix(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.matrix(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.matrix(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = cos(alpha[i])
    Rot_x[1, 2] = -sin(alpha[i])
    Rot_x[2, 1] = sin(alpha[i])

    A_i = Rot_z * Trans_z * Trans_x * Rot_x	    
    return A_i

# Forward Kinematics
def fwd_kin(theta, i_unit='r', o_unit='n'):
    T_06 = np.matrix(np.identity(4))

    if i_unit == 'd':
        theta = [radians(i) for i in theta]
    
    for i in range(6):
        T_06 *= HTM(i, theta)

    if o_unit == 'n':
        return T_06
    elif o_unit == 'p':
        return np2ros(T_06)
    else:
        print("Forward Kinematics Result:")
        print(T_06)  

joint_values = [radians(5), radians(-57), radians(45), radians(15), radians(5), radians(5)]
#Hasil Forward Kinematics Hitung Manual
result_fk = fwd_kin(joint_values, i_unit='r', o_unit='p')
result_htm = fwd_kin(joint_values, i_unit='r', o_unit='n')   # Menggunakan 'p' untuk output format ROS Pose

print("ROS Pose Forward Kinematics (Hitung Manual):")
print(result_fk)
print()
print('**************** INPUT INVERSE KINEMATICS')
print('Hasil Forward Kinematics (Matriks Transformasi Homogen) ')
print(result_htm)

# Modifikasi fungsi AH
def AH(n, th, c):
    T_a = np.identity(4)
    T_a[0, 3] = a[n - 1]
    T_a[1, 3] = 0
    T_a[2, 3] = 0
    
    T_d = np.identity(4)
    T_d[2, 3] = d[n - 1]
    
    Rzt = np.array([[np.cos(th[n - 1, c]), -np.sin(th[n - 1, c]), 0, 0],
                     [np.sin(th[n - 1, c]), np.cos(th[n - 1, c]), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    Rxa = np.array([[1, 0, 0, 0],
                     [0, np.cos(alpha[n - 1]), -np.sin(alpha[n - 1]), 0],
                     [0, np.sin(alpha[n - 1]), np.cos(alpha[n - 1]), 0],
                     [0, 0, 0, 1]])

    A_i = np.dot(np.dot(np.dot(T_d, Rzt), T_a), Rxa)

    return A_i

# Modifikasi fungsi invKine
def invKine(desired_pos):  
    th = np.zeros((6, 8))
    P_05 = (desired_pos @ np.array([0, 0, -d6, 1])) - np.array([0, 0, 0, 1])  # Removed .T
    
    # **** theta1 ****
    psi = np.arctan2(P_05[2], P_05[1])  # Adjusted indexing
    phi = np.arccos(d4 / np.sqrt(P_05[1] ** 2 + P_05[2] ** 2))  # Adjusted indexing and expression
    th[0, 0:4] = np.pi / 2 + psi + phi
    th[0, 4:8] = np.pi / 2 + psi - phi
    
    # **** theta5 ****
    cl = [0, 4]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = T_10 @ desired_pos
        arg_cos = (T_16[2, 3] - d4) / d6
        if -1 <= arg_cos <= 1:
            th[4, c:c + 2] = np.arccos(arg_cos)
            th[4, c + 2:c + 4] = -np.arccos(arg_cos)
        else:
            print(f"Invalid argument for arccosine: {arg_cos}. Skipping calculation.")
            th[4, c:c + 4] = np.nan  # Assign NaN if the argument is invalid
    
    # **** theta6 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = np.linalg.inv(T_10 @ desired_pos)
        th[5, c:c + 2] = np.arctan2(-T_16[1, 2], T_16[0, 2])  # Adjusted indexing and expression
    
    # **** theta3 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = AH(6, th, c)
        T_54 = AH(5, th, c)
        T_14 = (T_10 @ desired_pos) @ np.linalg.inv(T_54 @ T_65)
        P_13 = T_14 @ np.array([0, -d4, 0, 1]) - np.array([0, 0, 0, 1])  # Removed .T
        t3 = np.arccos((np.linalg.norm(P_13) ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3))  # Adjusted expression
        th[2, c] = t3
        th[2, c + 1] = -t3
    
    # **** theta2 and theta 4 ****
    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = np.linalg.inv(AH(6, th, c))
        T_54 = np.linalg.inv(AH(5, th, c))
        T_14 = (T_10 @ desired_pos) @ T_65 @ T_54
        P_13 = T_14 @ np.array([0, -d4, 0, 1]) - np.array([0, 0, 0, 1])  # Removed .T
        th[1, c] = -np.arctan2(P_13[1], -P_13[0]) + np.arcsin(a3 * np.sin(th[2, c]) / np.linalg.norm(P_13))  # Adjusted expression
        T_32 = np.linalg.inv(AH(3, th, c))
        T_21 = np.linalg.inv(AH(2, th, c))
        T_34 = T_32 @ T_21 @ T_14
        th[3, c] = np.arctan2(T_34[1, 0], T_34[0, 0])  # Adjusted indexing and expression
    
    return th

# Import library yang diperlukan
import numpy as np

# Input posisi yang diinginkan (matriks transformasi T_06)
desired_pos = np.array(result_htm)

# Panggil fungsi inverse kinematics
result_ik = invKine(desired_pos)

# Tampilkan hasil
print("Solusi inverse kinematics (radian):")
print(result_ik)

result_ik_degrees = np.degrees(result_ik)

# Tampilkan hasil
print("Solusi inverse kinematics (derajat):")
print(result_ik_degrees)
print()
result_ik_transposed = np.transpose(result_ik_degrees)

# Tampilkan hasil
print("Solusi inverse kinematics (dalam derajat):")
print(result_ik_transposed)
print()

# Inisialisasi list untuk menyimpan pose kartesian
print('Hasil IK Transpose (List)')
result_ik_transposed_list = result_ik_transposed.tolist()
"""print(result_ik_transposed_list)"""
# Result in Degree
print('8 Solusi IK (Derajat)')
print("[")
for sublist in result_ik_transposed_list:
    print("[", end=" ")
    for i, value in enumerate(sublist):
        # Print float values with higher precision
        if isinstance(value, float):
            if i == len(sublist) - 1:
                print("{:.15f}".format(value), end=" ")
            else:
                print("{:.15f}".format(value), end=", ")
        else:
            print(value, end=", ")
    print("],")  # Add a closing square bracket and comma for each sublist
print("]")

print("ROS Pose hasil forward kinematics dari nilai sendi hasil inverse kinematics:")
for index, solution in enumerate(result_ik_transposed_list, start=1):
    result_cartesian = fwd_kin(solution, i_unit='d', o_unit='p')
    print(f"Solution {index}:")
    print(result_cartesian)
    print()