#!/usr/bin/env python3.8
import time

import rospy
from std_srvs.srv import Empty
from dynamic_reconfigure.srv import dynamic_reconfigure

service_name = '/move_base/clear_costmaps'

if __name__ == "__main__":
    rospy.init_node('clear_costmap_node')

    rospy.wait_for_service(service_name)

    clear_costmap = rospy.ServiceProxy(service_name, Empty)

    while True:
        time.sleep(1)

        clear_costmap()