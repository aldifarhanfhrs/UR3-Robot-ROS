#!/usr/bin/env python

import sys
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import Pose
import time

# Inisialisasi node ROS
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move', anonymous=True)

# Inisialisasi MoveGroupCommander untuk grup manipulator
group = moveit_commander.MoveGroupCommander("manipulator")

# Loop melalui setiap solusi pose dari hasil forward kinematics
for pose_index in range(1, 9):  # Ada 8 solusi pose
    # Buat Pose baru
    pose_goal = Pose()
    
    # Set posisi dan orientasi dari result_cartesian
    if pose_index == 1:
        pose_goal.position.x = -0.32574099140030965
        pose_goal.position.y = -0.22317780274121768
        pose_goal.position.z = 0.31497264420402243
        pose_goal.orientation.x = 0.7069990853988244
        pose_goal.orientation.y = -0.04932527561612567
        pose_goal.orientation.z = 0.04918453819682154
        pose_goal.orientation.w = 0.703775668542885
    elif pose_index == 2:
        pose_goal.position.x = -0.3257409914003096
        pose_goal.position.y = -0.22317780274121768
        pose_goal.position.z = 0.3149726442040225
        pose_goal.orientation.x = 0.7069990853988243
        pose_goal.orientation.y = -0.049325275616125694
        pose_goal.orientation.z = 0.04918453819682157
        pose_goal.orientation.w = 0.7037756685428849
    elif pose_index == 3:
        pose_goal.position.x = -0.32574099140030965
        pose_goal.position.y = -0.22317780274121768
        pose_goal.position.z = 0.3149726442040225
        pose_goal.orientation.x = 0.7069990853988243
        pose_goal.orientation.y = -0.049325275616125694
        pose_goal.orientation.z = 0.049184538196821566
        pose_goal.orientation.w = 0.7037756685428849
    elif pose_index == 4:
        pose_goal.position.x = -0.32574099140030965
        pose_goal.position.y = -0.22317780274121768
        pose_goal.position.z = 0.3149726442040225
        pose_goal.orientation.x = 0.7069990853988243
        pose_goal.orientation.y = -0.04932527561612563
        pose_goal.orientation.z = 0.04918453819682154
        pose_goal.orientation.w = 0.7037756685428849
    elif pose_index == 5:
        pose_goal.position.x = -0.3257409914003136
        pose_goal.position.y = -0.2231778027412182
        pose_goal.position.z = 0.31497264420402216
        pose_goal.orientation.x = 0.7069990853988243
        pose_goal.orientation.y = -0.04932527561613134
        pose_goal.orientation.z = 0.049184538196815994
        pose_goal.orientation.w = 0.7037756685428849
    elif pose_index == 6:
        pose_goal.position.x = -0.32574099140031354
        pose_goal.position.y = -0.22317780274121812
        pose_goal.position.z = 0.3149726442040224
        pose_goal.orientation.x = 0.7069990853988242
        pose_goal.orientation.y = -0.04932527561613121
        pose_goal.orientation.z = 0.04918453819681586
        pose_goal.orientation.w = 0.703775668542885
    elif pose_index == 7:
        pose_goal.position.x = -0.32574099140031354
        pose_goal.position.y = -0.22317780274121812
        pose_goal.position.z = 0.31497264420402216
        pose_goal.orientation.x = 0.7069990853988243
        pose_goal.orientation.y = -0.04932527561613125
        pose_goal.orientation.z = 0.04918453819681595
        pose_goal.orientation.w = 0.7037756685428849
    elif pose_index == 8:
        pose_goal.position.x = -0.32574099140031343
        pose_goal.position.y = -0.22317780274121807
        pose_goal.position.z = 0.31497264420402216
        pose_goal.orientation.x = 0.7069990853988243
        pose_goal.orientation.y = -0.04932527561613132
        pose_goal.orientation.z = 0.049184538196816015
        pose_goal.orientation.w = 0.7037756685428849
    
    # Set target pose menggunakan nilai dari result_cartesian
    group.set_pose_target(pose_goal)
    
    # Gerakkan robot ke target pose
    group.go(True)
    
    # Tunggu 2 detik sebelum melanjutkan ke pose berikutnya
    time.sleep(2)

# Shutdown node ROS setelah selesai
moveit_commander.roscpp_shutdown()
