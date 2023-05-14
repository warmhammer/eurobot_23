import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tf.transformations as tft
from tf import TransformBroadcaster
import sys
import time
PKG_PATH = rospkg.RosPack().get_path('aruco_detection')
sys.path.append(PKG_PATH)
from scrips.aruco_detector import Detector
from scrips.configs_provider import ConfigsProvider
from scrips.cube_pose_common import get_cube_to_marker_mtxs


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


def image_callback(img_msg, cv_bridge, detector, t_broadcaster, publisher, cube_to_marker_mtxs):
    cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    detect_and_broadcast(cv_bridge, detector, t_broadcaster, publisher, cv_image, cube_to_marker_mtxs)


def detect_and_broadcast(cv_bridge, detector, t_broadcaster, publisher, cv_image, cube_to_marker_mtxs):
    dolly_mtxs = cube_to_marker_mtxs["dolly"]
    enemy_mtxs = cube_to_marker_mtxs["enemy"]
    T_camera_to_world = rospy.get_param("T_camera_to_world")
    T_camera_to_world = np.array(T_camera_to_world, dtype=np.float32)
    rotation_matrices, res_frame = detector.detect_markers(cv_image)
    for id, T_marker_to_camera in rotation_matrices.items():
        T_marker_to_world = T_camera_to_world @ T_marker_to_camera
        T_marker_to_world[2, 3] = 0
        t_broadcaster.sendTransform(tft.translation_from_matrix(T_marker_to_world),
                                    tft.quaternion_from_matrix(T_marker_to_world),
                                    rospy.Time.now(),
                                    f'marker_id_{id}', '/world')
        if id in dolly_mtxs:
            T_cube_to_marker = dolly_mtxs[id]
            T_cube_to_world = T_marker_to_world @ T_cube_to_marker
            t_broadcaster.sendTransform(tft.translation_from_matrix(T_cube_to_world),
                                    tft.quaternion_from_matrix(T_cube_to_world),
                                    rospy.Time.now(),
                                    'dolly', '/world')
        if id in enemy_mtxs:
            T_cube_to_marker = enemy_mtxs[id]
            T_cube_to_world = T_marker_to_world @ T_cube_to_marker
            t_broadcaster.sendTransform(tft.translation_from_matrix(T_cube_to_world),
                                    tft.quaternion_from_matrix(T_cube_to_world),
                                    rospy.Time.now(),
                                    'enemy', '/world')

    if publisher.get_num_connections() > 0:
        img_msg = cv_bridge.cv2_to_imgmsg(res_frame, "bgr8")
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
    cube_markers = known_markers["cube_markers"]
    cube_to_marker_mtxs = get_cube_to_marker_mtxs(cube_markers)
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
                         lambda msg: image_callback(msg, bridge, detector, tb, publisher, cube_to_marker_mtxs))
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
