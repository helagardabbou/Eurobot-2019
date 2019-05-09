#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('mouse_controller', anonymous=True)
    rospy.Subscriber('mouse_position', Point, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
