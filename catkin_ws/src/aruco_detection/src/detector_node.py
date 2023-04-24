import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf.transformations as tft
from tf import TransformBroadcaster
import sys
import time
PKG_PATH = rospkg.RosPack().get_path('aruco_detection')
sys.path.append(PKG_PATH)
from scrips.aruco_detector import Detector
from scrips.configs_provider import ConfigsProvider


def configure_video_capturer(camera_index, frame_width, frame_height):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    if not cap.isOpened():
        raise ValueError(f"Can not open camera by index {camera_index}")

    return cap

def image_callback(img_msg, cv_bridge, detector, t_broadcaster, show_img=False):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    rotation_matricies, res_frame = detector.detect_markers(cv_image)

    for id, r_mtx in rotation_matricies.items():
        
        t_broadcaster.sendTransform(tft.translation_from_matrix(r_mtx),
                                    tft.quaternion_from_matrix(r_mtx),
                                    rospy.Time.now(),
                                    f'marker_id_{id}', '/camera')
    if show_img:        
        cv2.imshow('Image', res_frame)
        cv2.waitKey(1)

def main():
    rospy.init_node('detector_node')
    configs = ConfigsProvider()
    known_markers = configs.get_markers_config()
    marker_size = known_markers["marker_size"]
    camera_mtx, dist_coeffs = configs.get_camera_info()
    detector = Detector(marker_size, cv2.aruco.DICT_4X4_100, camera_mtx, dist_coeffs, True)
    bridge = CvBridge()
    tb = TransformBroadcaster()
    rate = rospy.Rate(5)

    ok = rospy.get_param('camera_pose_ok', False)
    start_time = time.time()

    while ok != True and time.time() - start_time < 10:
        rospy.logwarn("Waiting for camera position...")
        ok = rospy.get_param('camera_pose_ok', False)
        rate.sleep()

    if ok != True:
        rospy.logerr("Determine camera position first!")
        exit()

    use_camera = rospy.get_param('~use_camera', True)
    show_img = rospy.get_param('~visualize', False)

    if use_camera == True:
        ci, fw, fh = configs.get_capturer_props()
        cap = configure_video_capturer(ci, fw, fh)
        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret == True:
                rotation_matricies, res_frame = detector.detect_markers(frame)

                for id, r_mtx in rotation_matricies.items():
                    tb.sendTransform(tft.translation_from_matrix(r_mtx),
                                     tft.quaternion_from_matrix(r_mtx),
                                     rospy.Time.now(), f"marker_id_{id}", "/camera")
                if show_img:
                    cv2.imshow('Image', res_frame)
                    cv2.waitKey(1)

        cv2.destroyAllWindows()
        cap.release()

    elif use_camera == False:
        rospy.Subscriber("/my_robot/camera1/image_raw", Image,
                         lambda msg: image_callback(msg, bridge, detector, tb, show_img))

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
