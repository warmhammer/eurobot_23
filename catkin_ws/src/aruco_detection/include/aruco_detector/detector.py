import cv2
import numpy as np
from cv2 import aruco
import math
from .object_pose import ObjectPose

class Detector:

    def __init__(self, marker_size, aruco_dict_type, camera_info_path):
        self.__cameraInfoPath = camera_info_path
        with np.load(camera_info_path) as file:
            camera_matrix, dist_coeffs = [file[i] for i in ('cameraMatrix', 'distCoeffs')]

        self._markerSize = marker_size
        self.__cameraMatrix = camera_matrix
        self.__distCoeff = dist_coeffs
        self.__arucoDictType = aruco_dict_type


    def __camera_to_world_coords(self, rotation_vector, translation_vector):
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        x, y, z = np.dot(-(np.linalg.inv(rotation_matrix)),
                                        translation_vector)
        roll, pitch, yaw = self.__rotation_matrix_to_euler_angles(rotation_matrix)
        pose = ObjectPose(x, y, z, roll, pitch, yaw)

        return pose
    

    def __rotation_matrix_to_euler_angles(self, rotation_mtx):
        sy = math.sqrt(rotation_mtx[0, 0] * rotation_mtx[0, 0] + rotation_mtx[1, 0] * rotation_mtx[1, 0])

        singular = sy < 1e-2

        if not singular:
            roll = math.atan2(rotation_mtx[2, 1], rotation_mtx[2, 2])
            pitch = math.atan2(-rotation_mtx[2, 0], sy)
            yaw = math.atan2(rotation_mtx[1, 0], rotation_mtx[0, 0])
        else:
            roll = math.atan2(-rotation_mtx[1, 2], rotation_mtx[1, 1])
            pitch = math.atan2(-rotation_mtx[2, 0], sy)
            yaw = 0

        return roll, pitch, yaw


    def detect_markers(self, frame): 
            
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(self.__arucoDictType)
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters, 
                                                  cameraMatrix=self.__cameraMatrix, distCoeff=self.__distCoeff)

            markers_poses: dict[int, ObjectPose] = {}

            if(ids is not None):

                for i in range(len(ids)):
                    rotation_vector, translation_vector, _ = aruco.estimatePoseSingleMarkers(corners[i], self._markerSize,
                                                                            self.__cameraMatrix, self.__distCoeff)
                    id = ids[i][0]
                    markers_poses[id] = self.__camera_to_world_coords(rotation_vector[0][0], translation_vector[0][0])

                frame = aruco.drawDetectedMarkers(frame, corners)

            return markers_poses, frame