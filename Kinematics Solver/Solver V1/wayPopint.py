#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import radians

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move', anonymous=True)

# Instantiate a `MoveGroupCommander` object for the UR3 manipulator
group = moveit_commander.MoveGroupCommander("manipulator")  # ur3 moveit group name: manipulator

# Define the joint angle positions in degrees (only one set of angles)
joint_angles_degrees = [-80.0, -120.0, -41.0, 175.0, -82.0, -0.0]

# Convert joint angles from degrees to radians
joint_angles_radians = [radians(angle) for angle in joint_angles_degrees]

# Set the target joint values
group.set_joint_value_target(joint_angles_radians)

# Plan and execute the motion
plan = group.plan()
group.go(wait=True)

# Shutdown moveit_commander
moveit_commander.roscpp_shutdown()