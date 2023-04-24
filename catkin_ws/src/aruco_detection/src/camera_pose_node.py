import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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


recived_img = None


def get_frame_from_camera(camera_index, frame_width, frame_height):
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

def image_callback(img_msg, cv_bridge):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error. Exception text:\n{e}")

    global recived_img
    if recived_img is None:
        recived_img = cv_image

def main():
    rospy.init_node("camera_pose_node")
    configs = ConfigsProvider()
    cv_bridge = CvBridge()
    tb = TransformBroadcaster()
    global recived_img
    use_camera = rospy.get_param('~use_camera', 'true')
    rate = rospy.Rate(30)

    camera_index, frame_width, frame_height = configs.get_capturer_props()
    if use_camera == True:
        recived_img = get_frame_from_camera(camera_index, frame_width, frame_height)

    else:
        img_sub = rospy.Subscriber("/my_robot/camera1/image_raw", Image, lambda msg: image_callback(msg, cv_bridge))
        start_time = time.time()

        while recived_img is None and time.time() - start_time < 1:
            rate.sleep()

    frame = recived_img

    camera_matrix, dist_coeffs = configs.get_camera_info()
    static_markers = configs.get_markers_config()["static_markers"]
    for i in range(3):
        ret, table_pose, rmtx = get_table_pose_and_rotation_matrix(frame, static_markers, camera_matrix, dist_coeffs)

        if ret == True:
            break

        if use_camera == True:
            frame = get_frame_from_camera(camera_index, frame_width, frame_height)

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
