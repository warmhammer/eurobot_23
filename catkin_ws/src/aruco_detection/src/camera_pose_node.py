import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys
import time
import yaml
from os.path import abspath
import tf.transformations as tft
from tf import TransformBroadcaster
PKG_PATH = rospkg.RosPack().get_path('aruco_detection')
sys.path.append(PKG_PATH)
from include.camera_pose_common import get_table_pose_and_rotation_matrix


markers_config_path = abspath(PKG_PATH + '/config/known_markers.yaml')
camera_config_path = abspath(PKG_PATH + '/config/camera_properties.yaml')
camera_info_path = abspath(PKG_PATH + '/../camera_calibration/camera_info/camera_info.npz')

with open(markers_config_path, "r") as file:
    known_markers = yaml.safe_load(file)
static_markers = known_markers['static_markers']

with open(camera_config_path, "r") as file:
    camera_prop = yaml.safe_load(file)
camera_index = camera_prop["camera_index"]
frame_width = camera_prop["frame_width"]
frame_height = camera_prop["frame_height"]

with np.load(camera_info_path) as file:
    camera_matrix, dist_coeffs = [file[i] for i in ('cameraMatrix', 'distCoeffs')]

bridge = CvBridge()
tb = TransformBroadcaster()
recived_img = None


def get_frame_from_camera():
    try:
        cap = cv2.VideoCapture(camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
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

def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error. Exception text:\n{e}")

    global recived_img
    if recived_img is None:
        recived_img = cv_image

def main():
    rospy.init_node("camera_pose_node")

    global recived_img
    use_camera = rospy.get_param('~use_camera', 'true')
    rate = rospy.Rate(30)

    if use_camera == True:
        recived_img = get_frame_from_camera()

    else:
        img_sub = rospy.Subscriber("/my_robot/camera1/image_raw", Image, image_callback)
        start_time = time.time()

        while recived_img is None and time.time() - start_time < 1:
            rate.sleep()

    frame = recived_img

    for i in range(3):
        ret, table_pose, rmtx = get_table_pose_and_rotation_matrix(frame, static_markers, camera_matrix, dist_coeffs)

        if ret == True:
            break

        if use_camera == True:
            frame = get_frame_from_camera()

        else:
            recived_img = None
            start_time = time.time()

            while recived_img is None and time.time() - start_time < 1:
                rate.sleep()

            if recived_img is None:
                break

            frame = recived_img
        rate.sleep()

    if ret != True:
        rospy.logerr("Camera position was not found")
        rospy.set_param('camera_pose_ok', False)
        exit()

    img_sub.unregister()

    r_table_to_cam = tft.inverse_matrix(rmtx)

    T_table_to_world = np.dot(tft.translation_matrix(table_pose[:3]), 
                            tft.euler_matrix(table_pose[3], table_pose[4], table_pose[5]))

    T_cam_world = np.dot(T_table_to_world, r_table_to_cam)

    rospy.set_param('camera_pose_ok', True)

    while not rospy.is_shutdown():
        tb.sendTransform(tft.translation_from_matrix(T_cam_world), 
                        tft.quaternion_from_matrix(T_cam_world), 
                        rospy.Time.now(), "camera", "/world")
        rate.sleep()
if __name__ == "__main__":
    main()
