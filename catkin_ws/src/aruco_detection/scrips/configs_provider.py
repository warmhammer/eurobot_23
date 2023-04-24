from os.path import abspath
from os.path import dirname
import numpy as np
import yaml


class ConfigsProvider:
    def __init__(self):
        this_dir = dirname(abspath(__file__))
        capturer_config_path = abspath(this_dir + '/../config/capturer_config.yaml')
        camera_info_path = abspath(this_dir + '/../../camera_calibration/camera_info/camera_info.npz')
        markers_config_path = abspath(this_dir + '/../config/known_markers.yaml')

        with open(capturer_config_path, "r") as file:
            capturer_config_path = yaml.safe_load(file)
        self.__camera_index = capturer_config_path["camera_index"]
        self.__frame_width = capturer_config_path["frame_width"]
        self.__frame_height = capturer_config_path["frame_height"]

        with open(markers_config_path, "r") as file:
            self.__known_markers = yaml.safe_load(file)

        with np.load(camera_info_path) as file:
            self.__camera_matrix, self.__dist_coeffs = [file[i] for i in ('cameraMatrix', 'distCoeffs')]

    def get_capturer_props(self):
        return self.__camera_index, self.__frame_width, self.__frame_height

    def get_camera_info(self):
        return self.__camera_matrix, self.__dist_coeffs
    
    def get_markers_config(self):
        return self.__known_markers
    