import cv2
import numpy as np
from cv2 import aruco


class Detector:
    def __init__(self, marker_size, aruco_dict_type, camera_matrix, dist_coeffs, draw_markers=False):
        self._markerSize = marker_size
        self.__cameraMatrix = camera_matrix
        self.__distCoeff = dist_coeffs
        self.__arucoDict = aruco.Dictionary_get(aruco_dict_type)
        self.__drawMarkers = draw_markers
        self.__objectPoints = np.array([[marker_size / 2, -marker_size / 2, 0],
                                        [-marker_size / 2, -marker_size / 2, 0],
                                        [-marker_size / 2, marker_size / 2, 0],
                                        [marker_size / 2, marker_size / 2, 0]], dtype=np.float32)

    def detect_markers(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray_frame, self.__arucoDict, parameters=parameters,
                                              cameraMatrix=self.__cameraMatrix, distCoeff=self.__distCoeff)
        if self.__drawMarkers == True:
            frame = aruco.drawDetectedMarkers(frame, corners)
        rotation_matrices = {}
        if ids is not None:
            for i in range(len(ids)):
                id = ids[i][0]
                _, rvec, tvec = cv2.solvePnP(self.__objectPoints, corners[i], self.__cameraMatrix, self.__distCoeff)
                rmat, _ = cv2.Rodrigues(rvec)
                rmat_4x4 = np.eye(4)
                rmat_4x4[:3, :3] = rmat
                rmat_4x4[2, :2] *= -1
                rmat_4x4[:2, 2] *= -1
                tvec = np.squeeze(tvec)
                tvec[:2] *= -1
                rmat_4x4[:3, 3] = tvec
                rotation_matrices[id] = rmat_4x4
        return rotation_matrices, frame
