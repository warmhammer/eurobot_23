import sys
import os
from os.path import abspath
from os import path
sys.path.append(abspath(path.dirname(__file__) + '/../'))
from include.aruco_detector import Detector
from include.aruco_detector import ObjectPose
import cv2
import math


params = {'cameraIndex': 0, 'frameWidth': 320, 'frameHeight': 240}

cap = cv2.VideoCapture(params['cameraIndex'])
cap.set(cv2.CAP_PROP_FRAME_WIDTH, params['frameWidth'])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, params['frameHeight'])

if not cap.isOpened():
    print('Can not open camera by index')
    exit(0)

this_dir = os.path.dirname(__file__)
camera_info_path = abspath(this_dir + '/../../camera_calibration/camera_info/camera_info.npz')
detector = Detector(0.1, cv2.aruco.DICT_4X4_100, camera_info_path)

while cap.isOpened():
    ret, frame = cap.read()

    if ret:
        
        key = cv2.waitKey(1)

        markers_poses, res_frame = detector.detect_markers(frame)

        print(f'=== markers poses ===')
        for id, pose in markers_poses.items():
            pose.Roll = math.degrees(pose.Roll)
            pose.Pitch = math.degrees(pose.Pitch)
            pose.Yaw = math.degrees(pose.Yaw)
            print('\nid: %d' % id, '\nx:\t%0.3f \ny:\t%0.3f \nz:\t%0.3f' %(pose.X, pose.Y, pose.Z),
                '\nroll:\t%d, \npitch:\t%d, \nyaw:\t%d\n' %(pose.Roll, pose.Pitch, pose.Yaw), sep='\n')

        cv2.imshow('img', res_frame)
        if key == ord('q') or key == 27:
            break

cv2.destroyAllWindows()
cap.release()
