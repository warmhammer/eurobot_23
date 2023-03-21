import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy
import cv2
import math
import sys
import os
from os.path import abspath
sys.path.append(abspath(os.path.dirname(__file__) + '/../'))
from include.aruco_detector import Detector


rospy.init_node('detector_node')

bridge = CvBridge()

this_dir = os.path.dirname(__file__)
camera_info_path = abspath(this_dir + '/../../camera_calibration/camera_info/camera_info.npz')
detector = Detector(0.1, cv2.aruco.DICT_4X4_100, camera_info_path)


def image_callback(img_msg):

    # rospy.loginfo(img_msg.header)

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    markers_poses, res_frame = detector.detect_markers(cv_image)

    print(f'=== markers poses ===')
    for id, pose in markers_poses.items():
        pose.Roll = math.degrees(pose.Roll)
        pose.Pitch = math.degrees(pose.Pitch)
        pose.Yaw = math.degrees(pose.Yaw)
        print('\nid: %d' % id, '\nx:\t%0.3f \ny:\t%0.3f \nz:\t%0.3f' %(pose.X, pose.Y, pose.Z),
              '\nroll:\t%d, \npitch:\t%d, \nyaw:\t%d\n' %(pose.Roll, pose.Pitch, pose.Yaw), sep='\n')
    
    cv2.imshow('Image', res_frame)
    cv2.waitKey(1)


sub_image = rospy.Subscriber("/my_robot/camera1/image_raw", Image, image_callback)

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

while not rospy.is_shutdown():
    rospy.spin()
