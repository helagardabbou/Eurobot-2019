#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

count = 0

def talker():
    pub = rospy.Publisher('led_value', Int32, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        global count
        count += 1
        pub.publish(count)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
