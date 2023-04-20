import yaml
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf.transformations as tft
from tf import TransformBroadcaster
import sys
import time
from os.path import abspath
PKG_PATH = rospkg.RosPack().get_path('aruco_detection')
sys.path.append(PKG_PATH)
from include.aruco_detector import Detector


DEBUG_WINDOW = False
camera_config_path = abspath(PKG_PATH + '/config/camera_properties.yaml')
camera_info_path = abspath(PKG_PATH + '/../camera_calibration/camera_info/camera_info.npz')
bridge = CvBridge()
tb = TransformBroadcaster()
markers_config_path = abspath(PKG_PATH + '/config/known_markers.yaml')
with open(markers_config_path, "r") as file:
    markers_config = yaml.safe_load(file)
detector = Detector(markers_config["marker_size"], cv2.aruco.DICT_4X4_100, camera_info_path, True)


def configure_video_capturer(camera_config_path):
    with open(camera_config_path, "r") as file:
        camera_prop = yaml.safe_load(file)
    camera_index = camera_prop["camera_index"]
    frame_width = camera_prop["frame_width"]
    frame_height = camera_prop["frame_height"]

    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    if not cap.isOpened():
        raise ValueError(f"Can not open camera by index {camera_index}")

    return cap

def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    rotation_matricies, res_frame = detector.detect_markers(cv_image)

    for id, r_mtx in rotation_matricies.items():
        
        tb.sendTransform(tft.translation_from_matrix(r_mtx), 
                         tft.quaternion_from_matrix(r_mtx), 
                         rospy.Time.now(), 
                         f'marker_id_{id}', '/camera')
    if DEBUG_WINDOW:        
        cv2.imshow('Image', res_frame)
        cv2.waitKey(1)

def main():
    rospy.init_node('detector_node')
    global DEBUG_WINDOW
    DEBUG_WINDOW = rospy.get_param('visualize', False)
    rate = rospy.Rate(5)

    ok = rospy.get_param('camera_pose_ok')
    start_time = time.time()

    while ok != True and time.time() - start_time < 10:
        rospy.logwarn("Waiting for camera position...")
        ok = rospy.get_param('camera_pose_ok')
        rate.sleep()

    if ok != True:
        rospy.logerr("Determine camera position first!")
        exit()

    use_camera = rospy.get_param('~use_camera', True)

    if use_camera == True:
        cap = configure_video_capturer(camera_config_path)
        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret == True:
                rotation_matricies, res_frame = detector.detect_markers(frame)

                for id, r_mtx in rotation_matricies.items():
                    tb.sendTransform(tft.translation_from_matrix(r_mtx), 
                                    tft.quaternion_from_matrix(r_mtx), 
                                    rospy.Time.now(), f"marker_id_{id}", "/camera")
                if DEBUG_WINDOW:
                    cv2.imshow('Image', res_frame)
                    cv2.waitKey(1)

        cv2.destroyAllWindows()
        cap.release()

    elif use_camera == False:
        rospy.Subscriber("/my_robot/camera1/image_raw", Image, image_callback)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
