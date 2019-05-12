#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

#define TRIGGER_PINR  5   //back
#define ECHO_PINR    4

#define TRIGGER_PINL  7   //front
#define ECHO_PINL     6

#define TRIGGER_PINB  12   //left
#define ECHO_PINB     13

#define TRIGGER_PINF  11   //right
#define ECHO_PINF     10

#define MAX_DISTANCE 300 // Maximum distance we want to ping

NewPing sonarL(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // back us
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE); // front us
NewPing sonarB(TRIGGER_PINB, ECHO_PINB, MAX_DISTANCE); // left us
NewPing sonarF(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE); // right us

ros::NodeHandle  nh;

sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
sensor_msgs::Range range_msg_4;
ros::Publisher pub_range1("ultrasound_1", &range_msg_1);
ros::Publisher pub_range2("ultrasound_2", &range_msg_2);
ros::Publisher pub_range3("ultrasound_3", &range_msg_3);
ros::Publisher pub_range4("ultrasound_4", &range_msg_4);

char frameid[] = "base_link";

long duration;
 float tmp;

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);

  range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_1.header.frame_id =  "ultrasound_1";
  range_msg_1.field_of_view = 0.3665;  // fake
  range_msg_1.min_range = 0.0;
  range_msg_1.max_range = MAX_DISTANCE;

  range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_2.header.frame_id =  "ultrasound_2";
  range_msg_2.field_of_view = 0.3665;  // fake
  range_msg_2.min_range = 0.0;
  range_msg_2.max_range = MAX_DISTANCE; 

  range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_3.header.frame_id =  "ultrasound_2";
  range_msg_3.field_of_view = 0.3665;  // fake
  range_msg_3.min_range = 0.0;
  range_msg_3.max_range = MAX_DISTANCE;  

  range_msg_4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_4.header.frame_id =  "ultrasound_4";
  range_msg_4.field_of_view = 0.3665;  // fake
  range_msg_4.min_range = 0.0;
  range_msg_4.max_range = MAX_DISTANCE;
}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    tmp=sonarL.ping_cm();
    range_msg_1.range = tmp;
    range_msg_1.header.stamp = nh.now();
    pub_range1.publish(&range_msg_1);

    tmp=sonarR.ping_cm();
    range_msg_2.range = tmp;
    range_msg_2.header.stamp = nh.now();
    pub_range2.publish(&range_msg_2);

     tmp=sonarB.ping_cm();
    range_msg_3.range = tmp;
    range_msg_3.header.stamp = nh.now();
    pub_range2.publish(&range_msg_3);

    tmp=sonarF.ping_cm();
    range_msg_4.range = tmp;
    range_msg_4.header.stamp = nh.now();
    pub_range4.publish(&range_msg_4);

    range_time =  millis() + 50;
  }
  nh.spinOnce();
}
