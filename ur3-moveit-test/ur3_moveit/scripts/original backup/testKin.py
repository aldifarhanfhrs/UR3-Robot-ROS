#!/usr/bin/env python

import sys
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion

pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move', anonymous=True)
group = moveit_commander.MoveGroupCommander("manipulator")

# Set posisi dan orientasi yang diinginkan
pose_goal.position.x = 0.25614099999999995
pose_goal.position.y = -1.1843
pose_goal.position.z = 0.011600000000000004
pose_goal.orientation.x = 0.5
pose_goal.orientation.y = 0.5
pose_goal.orientation.z = 0.5
pose_goal.orientation.w = 0.5000000000000001

while not rospy.is_shutdown():
    group.set_pose_target(pose_goal)
    group.go(True)
    rospy.sleep(2)

moveit_commander.roscpp_shutdown()
