#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty, Int16



# Set the global variables
step = 1



# Definition of the publishers
bob_pub = rospy.Publisher('show_bob', Empty, queue_size=10)
nav_pub = rospy.Publisher('required_coords', Point, queue_size=10)
rotate_pub = rospy.Publisher('rotate_instructions', Int16, queue_size=10)
take_pub = rospy.Publisher('take_puck', Empty, queue_size=10)
flush_pub = rospy.Publisher('flush_pucks', Empty, queue_size=10)



# The first action to start the callback
def show_bob():
    global bob_pub
    bob_pub.publish(Empty())
    rospy.loginfo("showing Bob")



# The function to move CURRY
def go_to(x, y):
    global nav_pub
    nav_pub.publish(Point(x, y, 0))



# The function to rotate CURRY
def rotate(degrees):
    global rotate_pub
    rotate_pub.publish(degrees)



# The function to take a puck
def take_puck():
    global take_pub
    take_pub.publish(Empty())



# The function to flush the pucks
def flush_pucks():
    global flush_pub
    flush_pub.publish(Empty())



# The callback function
def step_ok_callback(data):
    global step
    if data.data == True:

        if step == 1:
            take_puck()

        if step == 2:
            take_puck()

        if step == 3:
            take_puck()

        if step == 4:
            flush_pucks()

        if step == 5:
            rospy.loginfo("Finished !")

        if step > 5:
            rospy.loginfo("Anormally finished")
        step += 1
    else:
        rospy.loginfo("Waiting ...")





if __name__ == '__main__':
    try:
        rospy.init_node("raspberry", anonymous=True)
        rospy.Subscriber('curry_arrived', Bool, step_ok_callback)
        rate=rospy.Rate(1)
        show_bob()

        while not rospy.is_shutdown():
            rospy.loginfo(step)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
