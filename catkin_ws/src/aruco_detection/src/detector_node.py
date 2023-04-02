import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy
import cv2
from tf.transformations import quaternion_from_matrix
from tf.transformations import translation_from_matrix
from tf import TransformBroadcaster
import math
import sys
import os
from os.path import abspath
sys.path.append(abspath(os.path.dirname(__file__) + '/../'))
from include.aruco_detector import Detector


rospy.init_node('detector_node')
tb = TransformBroadcaster()
bridge = CvBridge()

this_dir = os.path.dirname(__file__)
camera_info_path = abspath(this_dir + '/../../camera_calibration/camera_info/camera_info.npz')
detector = Detector(0.1, cv2.aruco.DICT_4X4_100, camera_info_path)


def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    rotation_matricies, res_frame = detector.detect_markers(cv_image)

    for id, r_mtx in rotation_matricies.items():
        
        tb.sendTransform(translation_from_matrix(r_mtx), 
                         quaternion_from_matrix(r_mtx), 
                         rospy.Time.now(), 
                         f'marker_id_{id}', '/camera')
            
    cv2.imshow('Image', res_frame)
    cv2.waitKey(1)


sub_image = rospy.Subscriber("/my_robot/camera1/image_raw", Image, image_callback)

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

while not rospy.is_shutdown():
    rospy.spin()
