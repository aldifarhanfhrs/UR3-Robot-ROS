import rospy
from turtlesim.msg import Pose

def poseCallback(pose_message):
    print("Pose Callback")
    print('X = {}'.format(pose_message.x))
    print('Y = {}'.format(pose_message.y))
    print('Yaw = {}'.format(pose_message.theta))

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_motion_pose', anonymous= True)
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo('Node Terminated by Aldi')
