#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

count = 0

def callback(data):
     rospy.loginfo("I heard %s", data.data)

def listener():
    rospy.init_node('led1', anonymous=True)
    rospy.Subscriber('led_value', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
