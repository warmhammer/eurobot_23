#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import socket

def main(host="192.168.1.10", port=65432):
    publisher = rospy.Publisher('basket/points', Int16, queue_size=1)
    rospy.init_node('basket_listener')

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        
        while True:
            conn, _ = s.accept()

            with conn:
                while True:
                    data = conn.recv(1024).decode("utf-8")

                    if not data:
                        break

                    publisher.publish(int(data))

if __name__ == '__main__':
    main()