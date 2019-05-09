# Documentation of ROS

This file contains all the requirements to integrate ROS in our codes.

## Steps to create a package
1. To create a package with "catkin_create_pkg package_name rospy" in the "catkin_ws/src" folder
2. To create code(s) and make it executable with "(sudo) chmod +x code.py" in the "catkin_ws/src/package/src" folder
3. To build the packages with "catkin_make" in the "catkin_ws" folder
4. To install the build packages with "catkin_make install" in the "catkin_ws" folder

## Steps to launch a package
1. To start a core with roscore in the "catkin_ws" folder
3. To setup the source with "source ~/catkin_ws/devel/setup.bash" in each terminal
2. To run the codes with "rosrun PACKAGE EXECUTABLE [ARGS]" (one code/terminal) (Arduino: "rosrun rosserial_python serial_node.py /dev/ttyACM0")
3. use "ls /dev/tty*" to know wich USB to use for the arduino

## In the codes
1. Python
- To add "#!/usr/bin/env python" at the first line of the file
- To import the "rospy" module in python
- To import the good type of message (example: "from std_msgs.msg import Int32")
- To adapt the python code to use ROS node(s), topic(s), ...
2. Arduino
- To import ros with #include "<ros.h>"
- To import the good message types (example: #include <std_msgs/String.h>)
- To continue...
- For the complete informations, you can go to "http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup"
- You can find all the informations about this on "https://ecam-eurobot.github.io/Tutorials/software/ros/rosserial.html"
3. Tests
- To put the code on the arduino
- roscore
- rosrun rosserial_python serial_node.py /dev/ttyACM0
- rostopic echo /ultrasound
