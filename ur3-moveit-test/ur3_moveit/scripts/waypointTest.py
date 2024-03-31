#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
import time

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move', anonymous=True)
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Set waypoint pertama
    pose_goal1 = Pose()
    pose_goal1.position.x = 0.2978372748968737
    pose_goal1.position.y = 0.10814687320481103
    pose_goal1.position.z = 0.3188491251205956
    pose_goal1.orientation.x = 0.9536672223675944
    pose_goal1.orientation.y = 0.3006102721738748
    pose_goal1.orientation.z = 0.01148201798147305
    pose_goal1.orientation.w = 0.0045228871641013225

    group.set_pose_target(pose_goal1)
    group.go(True)

    # Delay 3 detik
    time.sleep(4)

    # Set waypoint kedua
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
