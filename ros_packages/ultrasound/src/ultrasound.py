#!/usr/bin/python
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String

go_condition = [True, True, True, True, True, True, True, True]

def callback(data, sensor_num):
    pub = rospy.Publisher('cmd_stop', String, queue_size=10)

    if (data.range < 10 and data.range > data.min_range):
        go_condition[sensor_num] = False
    else:
        go_condition[sensor_num] = True

    if (all(go_condition) == True):
        pub.publish("start")
    else:
        pub.publish("stop")

def stop_cb(msg):
    print(msg.data)
    if (msg.data == "stop"):
        go_condition[0] = False
    elif (msg.data== "start"):
        go_condition[0] = True

def listener():
    rospy.init_node('ultrasound', anonymous=True)
    rospy.Subscriber('run', String, stop_cb)
    rospy.Subscriber('ultrasound_1', Range, callback, 1)
    rospy.Subscriber('ultrasound_2', Range, callback, 2)
    rospy.Subscriber('ultrasound_3', Range, callback, 3)
    #rospy.Subscriber('ultrasound_4', Range, callback, 4) # disactivated because unnecessary
    #rospy.Subscriber('ultrasound_5', Range, callback, 5)
    #rospy.Subscriber('ultrasound_6', Range, callback, 6)
    #rospy.Subscriber('ultrasound_7', Range, callback, 7)
    rospy.spin()

if __name__ == '__main__':
    listener()
