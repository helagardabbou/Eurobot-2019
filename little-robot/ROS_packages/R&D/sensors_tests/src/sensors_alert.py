#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range


def callback(data):
    rospy.loginfo("%s: %s", data.header.frame_id, data.range)
    if data.range < 0.05:
        rospy.loginfo("STOP: %s is detecting something.", data.header.frame_id)
    else:
        rospy.loginfo("%s: continue...", data.header.frame_id)

def sensors_alert():
    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber('ultrasound_front_left', Range, callback)
    rospy.Subscriber('ultrasound_front_right', Range, callback)
    rospy.Subscriber('ultrasound_rear_left', Range, callback)
    rospy.Subscriber('ultrasound_rear_right', Range, callback)
    rospy.Subscriber('ultrasound_left', Range, callback)
    rospy.Subscriber('ultrasound_right', Range, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        sensors_alert()
    except rospy.ROSInterruptException:
        pass
