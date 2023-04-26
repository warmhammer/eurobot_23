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
    try:
        cap = cv2.VideoCapture(camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        if not cap.isOpened():
            raise cv2.error()
        return cap
    except cv2.error as e:
        rospy.logerr(f"Can not open camera. Exception text:\n{e}")
        exit()


def image_callback(img_msg, cv_bridge, detector, t_broadcaster, publisher):
    cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    detect_and_broadcast(cv_bridge, detector, t_broadcaster, publisher, cv_image)


def detect_and_broadcast(cv_bridge, detector, t_broadcaster, publisher, cv_image):
    rotation_matrices, res_frame = detector.detect_markers(cv_image)
    for id, r_mtx in rotation_matrices.items():
        t_broadcaster.sendTransform(tft.translation_from_matrix(r_mtx),
                                    tft.quaternion_from_matrix(r_mtx),
                                    rospy.Time.now(),
                                    f'marker_id_{id}', '/camera')
    img_msg = cv_bridge.cv2_to_imgmsg(res_frame, "bgr8")
    if publisher.get_num_connections() > 0:
        publisher.publish(img_msg)


def wait_for_camera_pose(wait_time):
    rate = rospy.Rate(5)
    ok = rospy.get_param('camera_pose_ok', False)
    start_time = time.time()
    while not ok and time.time() - start_time < wait_time:
        rospy.logwarn("Waiting for camera position...")
        ok = rospy.get_param('camera_pose_ok', False)
        rate.sleep()
    if not ok:
        rospy.logerr("Determine camera position first!")
        exit()


def detecting_using_camera(publisher, configs, detector, cv_bridge, t_broadcaster):
    cap_props = configs.get_capturer_props()
    cap = configure_video_capturer(*cap_props)
    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret == True:
            detect_and_broadcast(cv_bridge, detector, t_broadcaster, publisher, frame)
        else:
            rospy.logwarn("Failed to get frame from camera")
    cap.release()


def main():
    rospy.init_node('detector_node')
    wait_time = 10  # seconds
    wait_for_camera_pose(wait_time)
    topic_name = rospy.get_param("~publishing_img_topic", "/detector_node/detector")
    publisher = rospy.Publisher(topic_name, Image, queue_size=10)
    configs = ConfigsProvider()
    known_markers = configs.get_markers_config()
    marker_size = known_markers["marker_size"]
    camera_mtx, dist_coeffs = configs.get_camera_info()
    detector = Detector(marker_size, cv2.aruco.DICT_4X4_100, camera_mtx, dist_coeffs, True)
    bridge = CvBridge()
    tb = TransformBroadcaster()

    use_camera = rospy.get_param('~use_camera', True)
    if use_camera:
        detecting_using_camera(publisher, configs, detector, bridge, tb)
    else:
        topic_name = rospy.get_param("~receiving_img_topic", "/my_robot/camera1/image_raw")
        rospy.Subscriber(topic_name, Image,
                         lambda msg: image_callback(msg, bridge, detector, tb, publisher))
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
