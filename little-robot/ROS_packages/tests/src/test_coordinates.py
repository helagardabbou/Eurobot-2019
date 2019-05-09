#!/usr/bin/env python

"""
    send the required coordinates (1Hz) of CURRY
    not the best version
"""

import rospy
from geometry_msgs.msg import Point


def coords_talker(x, y, z):
    pub = rospy.Publisher('required_coords', Point, queue_size=10)
    rospy.init_node('coordinates', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(Point(x, y, z))
        rate.sleep()

try:
    coords_talker(10, 0, 0)
except rospy.ROSInterruptException:
    pass
