import rospy
from std_msgs import msg
import numpy as np

# w = 2.0
w = 0
A = 4
A0 = 0 # 7

def sin_callback_left(event: rospy.timer.TimerEvent):
    cur_time = event.current_real.to_sec()

    value = A * np.sin(w * cur_time) + A0
    # value = 0
    message = msg.Float32(value)

    cmd_pub_left.publish(message)

def sin_callback_right(event: rospy.timer.TimerEvent):
    cur_time = event.current_real.to_sec()

    value = A * np.sin(w * cur_time) + A0
    # value = 0
    message = msg.Float32(value)

    cmd_pub_right.publish(message)

if __name__ == '__main__':
    rospy.init_node('laser_converter')
    rospy.Rate(100)

    cmd_pub_left = rospy.Publisher('/dolly/left_wheel/pwd32', msg.Float32, queue_size=1)
    rospy.Timer(rospy.Duration(0.01), sin_callback_left)

    cmd_pub_right = rospy.Publisher('/dolly/right_wheel/pwd32', msg.Float32, queue_size=1)
    rospy.Timer(rospy.Duration(0.01), sin_callback_right)


    rospy.spin()
