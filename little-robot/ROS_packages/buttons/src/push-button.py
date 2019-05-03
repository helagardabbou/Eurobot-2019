#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool



GPIO.setwarnings(False) # Ignore warning
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

# Publishers
pub0 = rospy.Publisher('button0', Bool, queue_size=10)
pub1 = rospy.Publisher('button1', Bool, queue_size=10)
pub2 = rospy.Publisher('button2', Bool, queue_size=10)
pub3 = rospy.Publisher('button3', Bool, queue_size=10)
pub4 = rospy.Publisher('button4', Bool, queue_size=10)
pub5 = rospy.Publisher('button5', Bool, queue_size=10)

# button0 - front left - 29
# button1 - front right - 31
# button2 - left - 32
# button3 - right - 33
# button4 - rear left - 35
# button5 - rear right - 36
pins = [29, 40, 32, 33, 35, 36]
for pin in pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)



def button0_talker():
    global pub0
    if GPIO.input(pins[0]) == GPIO.HIGH:
        pub0.publish(True)
    else:
        pub0.publish(False)



def button1_talker():
    global pub1
    if GPIO.input(pins[1]) == GPIO.HIGH:
        pub1.publish(True)
    else:
        pub1.publish(False)



def button2_talker():
    global pub2
    if GPIO.input(pins[2]) == GPIO.HIGH:
        pub2.publish(True)
    else:
        pub2.publish(False)



def button3_talker():
    global pub3
    if GPIO.input(pins[3]) == GPIO.HIGH:
        pub3.publish(True)
    else:
        pub3.publish(False)



def button4_talker():
    global pub4
    if GPIO.input(pins[4]) == GPIO.HIGH:
        pub4.publish(True)
    else:
        pub4.publish(False)



def button5_talker():
    global pub5
    if GPIO.input(pins[5]) == GPIO.HIGH:
        pub5.publish(True)
    else:
        pub5.publish(False)



if __name__ == '__main__':
    try:
        rospy.init_node('buttons', anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            button0_talker()
            button1_talker()
            button2_talker()
            button3_talker()
            button4_talker()
            button5_talker()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
