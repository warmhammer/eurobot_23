#!/usr/bin/env python3.8
from math import cos, sin, pi

import rospy

from std_msgs.msg import UInt16MultiArray, Header
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


class Laser:
    def __init__(self, x: float, y: float, rot: float, tilt: float):
        self._x = x
        self._y = y
        self._rot = rot * pi / 180
        self._tilt = tilt * pi / 180

    def get_point(self, dist: int) -> Point32:
        x = self._x + dist * cos(self._tilt) * cos(self._rot)
        y = self._y + dist * cos(self._tilt) * sin(self._rot)

        x /= 1000   # mm into metres
        y /= 1000 

        return Point32(x, y, 0)


class LaserSet:
    def __init__(self, lasers: Laser, frame_id: str, sub_topic: str, pub_topic: str):
        self._lasers = lasers
        self.frame_id = frame_id

        rospy.Subscriber(sub_topic, UInt16MultiArray, self.callback)
        self.pub = rospy.Publisher(pub_topic, PointCloud, queue_size=1)

    def callback(self, dists: UInt16MultiArray):
        point_cloud = PointCloud()

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        point_cloud.header = header

        for i, laser in enumerate(self._lasers):
            if dists.data[i] < 1000:
                point_cloud.points.append(laser.get_point(dists.data[i]))

        self.pub.publish(point_cloud)

# def callback_(dists: UInt16MultiArray):
#     rospy.logwarn('true')


if __name__ == '__main__':
    rospy.init_node('laser_converter')

    laser_set = LaserSet(
        [
            Laser(0, 0, 0, 0),
            Laser(0, 0, 60, 0),
            Laser(0, 0, 120, 0),

            Laser(0, 0, 180, 0),
            Laser(0, 0, 240, 0),
            Laser(0, 0, 300, 0)
        ],
        'sensors',
        # 'base_footprint',
        'range_sensors_topic',
        'laser_scan'
    )
    
    # rospy.Subscriber('/range_sensors_topic', UInt16MultiArray, callback_)

    # rospy.logwarn('true')

    rospy.spin()