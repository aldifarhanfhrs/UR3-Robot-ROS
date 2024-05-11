#!/usr/bin/python3

## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University

# ***** lib
import numpy as np
from numpy import linalg
import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

global mat
mat = np.matrix

# ****** Coefficients ******
global d1, a2, a3, a7, d4, d5, d6
d1 = 0.1519
a2 = -0.24365
a3 = -0.21325
a7 = 0.075
d4 = 0.11235
d5 = 0.08535
d6 = 0.0819

d = mat([0.1519, 0., 0., 0.11235, 0.08535, 0.0819])  # ur10 mm
a = mat([0, -0.612, -0.5723, 0, 0, 0])  # ur10 mm
alph = mat([pi / 2, 0., 0., pi / 2, -pi / 2, 0])  # ur10

# ************************************************** FORWARD KINEMATICS

def AH(n, th, c):

    T_a = mat(np.identity(4), copy=False)
    T_a[0, 3] = a[0, n - 1]
    T_d = mat(np.identity(4), copy=False)
    T_d[2, 3] = d[0, n - 1]

    Rzt = mat([[cos(th[n - 1, c]), -sin(th[n - 1, c]), 0, 0],
               [sin(th[n - 1, c]), cos(th[n - 1, c]), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]], copy=False)

    Rxa = mat([[1, 0, 0, 0],
               [0, cos(alph[0, n - 1]), -sin(alph[0, n - 1]), 0],
               [0, sin(alph[0, n - 1]), cos(alph[0, n - 1]), 0],
               [0, 0, 0, 1]], copy=False)

    A_i = T_d * Rzt * T_a * Rxa

    return A_i

def HTrans(th, c):
    A_1 = AH(1, th, c)
    A_2 = AH(2, th, c)
    A_3 = AH(3, th, c)
    A_4 = AH(4, th, c)
    A_5 = AH(5, th, c)
    A_6 = AH(6, th, c)

    T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

    return T_06

# ************************************************** INVERSE KINEMATICS

def invKine(desired_pos):  # T60
    th = mat(np.zeros((6, 8)))
    P_05 = (desired_pos * mat([0, 0, -d6, 1]).T - mat([0, 0, 0, 1]).T)

    # **** theta1 ****

    psi = atan2(P_05[2 - 1, 0], P_05[1 - 1, 0])
    phi = acos(d4 / sqrt(P_05[2 - 1, 0] * P_05[2 - 1, 0] + P_05[1 - 1, 0] * P_05[1 - 1, 0]))
    th[0, 0:4] = pi / 2 + psi + phi
    th[0, 4:8] = pi / 2 + psi - phi
    th = th.real

    # **** theta5 ****

    cl = [0, 4]  # wrist up or down
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_16 = T_10 * desired_pos
        th[4, c:c + 2] = +acos((T_16[2, 3] - d4) / d6)
        th[4, c + 2:c + 4] = -acos((T_16[2, 3] - d4) / d6)

    th = th.real

    # **** theta6 ****

    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_16 = linalg.inv(T_10 * desired_pos)
        th[5, c:c + 2] = atan2((-T_16[1, 2] / sin(th[4, c])), (T_16[0, 2] / sin(th[4, c])))

    th = th.real

    # **** theta3 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_65 = AH(6, th, c)
        T_54 = AH(5, th, c)
        T_14 = (T_10 * desired_pos) * linalg.inv(T_54 * T_65)
        P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0, 0, 0, 1]).T
        t3 = cmath.acos(
            (linalg.norm(P_13) ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3))  # norm ?
        th[2, c] = t3.real
        th[2, c + 1] = -t3.real

    # **** theta2 and theta 4 ****

    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = linalg.inv(AH(1, th, c))
        T_65 = linalg.inv(AH(6, th, c))
        T_54 = linalg.inv(AH(5, th, c))
        T_14 = (T_10 * desired_pos) * T_65 * T_54
        P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0, 0, 0, 1]).T

        # theta 2
        th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3 * sin(th[2, c]) / linalg.norm(P_13))
        # theta 4
        T_32 = linalg.inv(AH(3, th, c))
        T_21 = linalg.inv(AH(2, th, c))
        T_34 = T_32 * T_21 * T_14
        th[3, c] = atan2(T_34[1, 0], T_34[0, 0])
    th = th.real

    return th

# Import library yang diperlukan
import numpy as np

# Input posisi yang diinginkan (matriks transformasi T_06)
desired_pos = np.array([[9.90295797e-01, -1.38975612e-01,  1.18989396e-04, -3.25740991e-01],
                            [-5.16086992e-04, -4.53365110e-03, -9.99989590e-01, -2.23177803e-01],
                            [ 1.38974705e-01,  9.90285426e-01, -4.56137914e-03,  3.14972644e-01],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])

# Panggil fungsi inverse kinematics
result = invKine(desired_pos)

# Tampilkan hasil
print("Solusi inverse kinematics:")
print(result)

result_degrees = np.degrees(result)

# Tampilkan hasil
print("Solusi inverse kinematics (dalam derajat):")
print(result_degrees)
print()

result_transposed = np.transpose(result_degrees)

# Tampilkan hasil
print("Solusi inverse kinematics (dalam derajat):")
print(result_transposed)