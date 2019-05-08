#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Empty

rospy.init_node('move')
rospy.Subscriber('/cmd_feedback', Empty, feedback_cb)
pub_lin = rospy.Publisher('cmd_lin', Float32, queue_size=50)
pub_rot = rospy.Publisher('cmd_rot', Float32, queue_size=50)
pub_arm = rospy.Publisher('cmd_arm', String, queue_size=50)
pub_stop = rospy.Publisher('cmd_stop', String, queue_size=50)

action = 0

def feedback_cb():
    action += 1
    move()

def lin(dist):
    pub_lin.Publish(dist)

def rot(angle):
    pub_rot.Publish(angle)

def arm(side):
    pub_tige.Publish(side)

def move():
    actions = [lin(0.5), rot(-90), lin(0.2), arm("left")]
    actions[action]


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
