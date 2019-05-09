#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "encoder_publisher");
  ros::NodeHandle n;
  ros::Publisher enc_pub = n.advertise<geometry_msgs::Vector3>("encoder_ticks", 50);
  geometry_msgs::Vector3 enc;

  enc.x = 0;
  enc.y = 0;

  ros::Rate r(10.0);
  while(n.ok()){
    enc.x += 5;
    enc.y += 8;

    enc_pub.publish(enc);
    ros::spinOnce();
    r.sleep();
  }
}
