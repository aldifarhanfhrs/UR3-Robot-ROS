#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move', anonymous=True)
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Set posisi dan orientasi yang diinginkan
    #pose_goal = Pose()
    pose_goal2 = Pose()
    pose_goal2.position.x = -0.12195617701229756
    pose_goal2.position.y = 0.15122569945602055
    pose_goal2.position.z = 0.6941294674072664
    pose_goal2.orientation.x = -0.6664466720611539
    pose_goal2.orientation.y = -0.2360011441781301
    pose_goal2.orientation.z = 0.25919321858973027
    pose_goal2.orientation.w = 0.6580054473044448

    group.set_pose_target(pose_goal2)
    group.go(True)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
