import cv2
import rospy
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_from_matrix
from tf.transformations import translation_from_matrix
import sys
import os
from os.path import abspath
from os import path
sys.path.append(abspath(path.dirname(__file__) + '/../'))
from include.aruco_detector import Detector


rospy.init_node("aruco_test")
tb = TransformBroadcaster()

this_dir = os.path.dirname(__file__)
camera_info_path = abspath(this_dir + '/../../camera_calibration/camera_info/camera_info.npz')
detector = Detector(0.1, cv2.aruco.DICT_4X4_100, camera_info_path)


params = {'cameraIndex': 0, 'frameWidth': 640, 'frameHeight': 480}

cap = cv2.VideoCapture(params['cameraIndex'])
cap.set(cv2.CAP_PROP_FRAME_WIDTH, params['frameWidth'])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, params['frameHeight'])

if not cap.isOpened():
    print('Can not open camera by index')
    exit(0)



rate = rospy.Rate(30)

while cap.isOpened() and not rospy.is_shutdown():
    ret, frame = cap.read()

    if ret:
        
        key = cv2.waitKey(1)

        rotation_matricies, res_frame = detector.detect_markers(frame)

        for id, r_mtx in rotation_matricies.items():
            tb.sendTransform(translation_from_matrix(r_mtx), 
                             quaternion_from_matrix(r_mtx), 
                             rospy.Time.now(), f"marker_id_{id}", "/camera")

            rate.sleep()

        cv2.imshow('img', res_frame)
        if key == ord('q') or key == 27:
            break

cv2.destroyAllWindows()
cap.release()
