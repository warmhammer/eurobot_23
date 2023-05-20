import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from os import path
from os.path import abspath


counter = 1
def image_callback(img_msg, cv_bridge, output_path):
    global counter
    cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    cv2.imshow("image", cv_image)
    key = cv2.waitKey(1)
    # save frame to file when pressing spacebar or 's' key
    if key == ord(' ') or key == ord('s'):
        file_path = f"{output_path}/img{counter}.png"
        cv2.imwrite(file_path, cv_image)
        counter += 1
    # exit when pressing 'q' or Esc
    elif key == ord('q') or key == 27:
        rospy.signal_shutdown("exit when pressing 'q' or Esc")


rospy.init_node("take_images")
cv_bridge = CvBridge()
topic_name = rospy.get_param("~receiving_img_topic", "/usb_cam/image_raw")
this_dir = path.dirname(abspath(__file__))
output_path = rospy.get_param("~output_path", f"{this_dir}/../images")
output_path = abspath(output_path)
if not path.isdir(output_path):
    os.makedirs(output_path)
else:
    for f in os.listdir(output_path):
        os.remove(os.path.join(output_path, f))

sub = rospy.Subscriber(topic_name, Image, lambda msg: image_callback(msg, cv_bridge, output_path))

rate = rospy.Rate(60)

while not rospy.is_shutdown():
    rate.sleep()