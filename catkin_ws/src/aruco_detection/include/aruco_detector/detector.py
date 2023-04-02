import cv2
import numpy as np
from cv2 import aruco
import math
from typing import Dict, Tuple, Any, Optional


class Detector:

    def __init__(self, marker_size, aruco_dict_type, camera_info_path):
        self.__cameraInfoPath = camera_info_path
        with np.load(camera_info_path) as file:
            camera_matrix, dist_coeffs = [file[i] for i in ('cameraMatrix', 'distCoeffs')]

        self._markerSize = marker_size
        self.__cameraMatrix = camera_matrix
        self.__distCoeff = dist_coeffs
        self.__arucoDictType = aruco_dict_type


    def detect_markers(self, frame) -> Tuple[Dict[int, Any], Optional[Any]]:

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(self.__arucoDictType)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters,
                                              cameraMatrix=self.__cameraMatrix, distCoeff=self.__distCoeff)

        rotation_matricies = {}

        if ids is not None:

            for i in range(len(ids)):
                rotation_vector, translation_vector, _ = aruco.estimatePoseSingleMarkers(corners[i], self._markerSize,
                                                                                         self.__cameraMatrix,
                                                                                         self.__distCoeff)
                rotation_matrix, _ = cv2.Rodrigues(rotation_vector[0][0])
                rotation_matrix[2, :2] = rotation_matrix[2, :2] * -1
                rotation_matrix[:2, 2] = rotation_matrix[:2, 2] * -1
                translation_vector = translation_vector[0][0]
                translation_vector[:2] = translation_vector[:2] * -1

                rotation_matrix_4x4 = np.eye(4)
                rotation_matrix_4x4[:3, :3] = rotation_matrix
                rotation_matrix_4x4[:3, 3] = translation_vector

                id = ids[i][0]
                rotation_matricies[id] = rotation_matrix_4x4

            frame = aruco.drawDetectedMarkers(frame, corners)

        return rotation_matricies, frame
