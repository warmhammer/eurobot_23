import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import time
import tf.transformations as tft
from tf import TransformBroadcaster
PKG_PATH = rospkg.RosPack().get_path('aruco_detection')
sys.path.append(PKG_PATH)
from scrips.camera_pose_common import get_table_pose_and_rotation_matrix
from scrips.configs_provider import ConfigsProvider


class ImageReceiver:
    def __init__(self, configs):
        self.__received_img = None
        self._configs = configs

    def receive_from_camera(self):
        cap_props = self._configs.get_capturer_props()
        self.__received_img = self.__get_frame_from_camera()
        return self.__received_img

    def img_callback(self, img_msg, cv_bridge):
        if self.__received_img is None:
            cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
            self.__received_img = cv_image

    def receive_from_topic(self, wait_time):
        self.__received_img = None
        start_time = time.time()
        rate = rospy.Rate(30)
        while self.__received_img is None and time.time() - start_time < wait_time:
            rate.sleep()
        return self.__received_img

    def __get_frame_from_camera(self):
        try:
            ci, fw, fh = self._configs.get_capturer_props()
            cap = cv2.VideoCapture(ci)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, fw)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, fh)
            if not cap.isOpened():
                raise cv2.error()
            while cap.isOpened():
                ret, frame = cap.read()
                if ret == True:
                    cap.release()
                    return frame
        except cv2.error as e:
            rospy.logerr(f"Can not open camera. Exception text:\n{e}")
        finally:
            rospy.set_param('camera_pose_ok', False)
            exit()


def determine_and_broadcast_camera_pose(table_pose, table_rmtx, t_broadcaster):
    r_table_to_cam = tft.inverse_matrix(table_rmtx)
    T_table_to_world = np.dot(tft.translation_matrix(table_pose[:3]),
                              tft.euler_matrix(*table_pose[3:]))
    T_cam_world = np.dot(T_table_to_world, r_table_to_cam)
    rospy.set_param('camera_pose_ok', True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        t_broadcaster.sendTransform(tft.translation_from_matrix(T_cam_world),
                                    tft.quaternion_from_matrix(T_cam_world),
                                    rospy.Time.now(), "camera", "/world")
        rate.sleep()


def main():
    rospy.init_node("camera_pose_node")
    configs = ConfigsProvider()
    cv_bridge = CvBridge()
    tb = TransformBroadcaster()
    rate = rospy.Rate(30)
    receiver = ImageReceiver(configs)
    use_camera = rospy.get_param('~use_camera', True)
    if not use_camera:
        topic_name = rospy.get_param("~receiving_img_topic", "/my_robot/camera1/image_raw")
        rospy.Subscriber(topic_name, Image, lambda msg: receiver.img_callback(msg, cv_bridge))
    camera_info = configs.get_camera_info()
    static_markers = configs.get_markers_config()["static_markers"]
    for _ in range(3):
        if use_camera:
            frame = receiver.receive_from_camera()
        else:
            wait_time = 1  # seconds
            frame = receiver.receive_from_topic(wait_time)
        success, table_pose, rmtx = get_table_pose_and_rotation_matrix(frame, static_markers, *camera_info)
        if success:
            break
        rate.sleep()
    if not success:
        rospy.logerr("Camera position was not found")
        rospy.set_param('camera_pose_ok', False)
        exit()
    determine_and_broadcast_camera_pose(table_pose, rmtx, tb)


if __name__ == "__main__":
    main()
