import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander


class ObjectDetectionAndControl:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # Inisialisasi MoveIt
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Deteksi sudut kemiringan balok menggunakan OpenCV
            # Misalnya, kita akan mencari sudut kemiringan menggunakan Hough Lines
            # Anda dapat mengganti ini dengan metode deteksi lain yang sesuai
            
            # Misalnya, kita menggunakan contoh sederhana: mencari garis terpanjang
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
            max_length = 0
            max_line = None
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if length > max_length:
                    max_length = length
                    max_line = line[0]
            
            # Jika garis terpanjang ditemukan, hitung sudut kemiringannya
            if max_line is not None:
                x1, y1, x2, y2 = max_line
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                
                # Hitung posisi sudut kemiringan dan ubah menjadi posisi end effector UR
                end_effector_position = self.calculate_end_effector_position(angle)
                
                # Rencanakan dan eksekusi pergerakan end effector menggunakan MoveIt
                self.move_end_effector(end_effector_position)

        except CvBridgeError as e:
            print(e)

    def calculate_end_effector_position(self, angle):
        # Misalnya, kita mengubah sudut kemiringan menjadi posisi end effector yang sederhana
        # Anda perlu menyesuaikan ini sesuai dengan geometri robot UR3 Anda
        end_effector_position = Pose()
        end_effector_position.position.x = angle / 180.0  # Misalnya, menggunakan sudut sebagai koordinat x
        end_effector_position.position.y = 0.0  # Misalnya, tetap pada sumbu y
        end_effector_position.position.z = 0.1  # Misalnya, tinggi tetap konstan
        return end_effector_position

    def move_end_effector(self, end_effector_position):
        # Rencanakan pergerakan end effector
        self.group.set_pose_target(end_effector_position)
        plan = self.group.plan()

        # Eksekusi pergerakan
        self.group.go(wait=True)


def main():
    rospy.init_node('object_detection_and_control_node')
    obj_detection_control = ObjectDetectionAndControl()
    rospy.spin()

if __name__ == '__main__':
    main()
