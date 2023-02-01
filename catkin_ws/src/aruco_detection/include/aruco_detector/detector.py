import cv2
import numpy


class Detector:

    def __init__(self, marker_size, aruco_dict_type, camera_info_path):
        self.__cameraInfoPath = camera_info_path
        with numpy.load(camera_info_path) as file:
            camera_mtx, dist, _, _ = [file[i] for i in ('cameraMatrix', 'distCoeff', 'rVectors', 'tVectors')]

        self._markerSize = marker_size
        self.__cameraMatrix = camera_mtx
        self.__distCoeff = dist
        self.__arucoDictType = aruco_dict_type

    def detect(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(self.__arucoDictType)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_frame, aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=self.__cameraMatrix,
                                                                    distCoeff=self.__distCoeff)
        result = dict()

        if len(corners) > 0:
            for i in range(0, len(ids)):
                result[str(ids[i])] = cv2.aruco.estimatePoseSingleMarkers(corners[i], self._markerSize,
                                                                          self.__cameraMatrix, self.__distCoeff)
        return result
