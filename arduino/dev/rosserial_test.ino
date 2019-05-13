#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;
geometry_msgs::Vector3 encoder_msg;
ros::Publisher pub_encoder("encoder_ticks", &encoder_msg);

volatile long _LeftEncoderTicks = 10;
volatile long _RightEncoderTicks = 20;

void setup() {
  nh.initNode();
  nh.advertise(pub_encoder);
}

void loop() {
  encoder_msg.x = _LeftEncoderTicks;
  encoder_msg.y = _RightEncoderTicks;

  pub_encoder.publish(&encoder_msg);

  nh.spinOnce();
}
