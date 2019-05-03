#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

#define TRIGGER_PIN_1  26   // FL: Front Left
#define ECHO_PIN_1    28   

#define TRIGGER_PIN_2  22   // FR: Front Right
#define ECHO_PIN_2    24

#define TRIGGER_PIN_3  42   // RL: Rear Left
#define ECHO_PIN_3    44 

#define TRIGGER_PIN_4  38   // RR: Rear Right
#define ECHO_PIN_4    40 

#define TRIGGER_PIN_5  34   // L: Left
#define ECHO_PIN_5     36

#define TRIGGER_PIN_6  30   // R: Right
#define ECHO_PIN_6    32 

#define MAX_DISTANCE 300 // Maximum distance we want to ping  

NewPing sonar_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // FL
NewPing sonar_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // FR
NewPing sonar_3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); // RL
NewPing sonar_4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE); // RR
NewPing sonar_5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE); // L
NewPing sonar_6(TRIGGER_PIN_6, ECHO_PIN_6, MAX_DISTANCE); // R

ros::NodeHandle  nh;

sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
sensor_msgs::Range range_msg_4;
sensor_msgs::Range range_msg_5;
sensor_msgs::Range range_msg_6;
ros::Publisher pub_range_1("ultrasound_front_left", &range_msg_1);
ros::Publisher pub_range_2("ultrasound_front_right", &range_msg_2);
ros::Publisher pub_range_3("ultrasound_rear_left", &range_msg_3);
ros::Publisher pub_range_4("ultrasound_rear_right", &range_msg_4);
ros::Publisher pub_range_5("ultrasound_left", &range_msg_5);
ros::Publisher pub_range_6("ultrasound_right", &range_msg_6);
 

char frameid[] = "base_link";

long duration;
 float tmp;
 float old_tmp_1;
 float old_tmp_2;
 float old_tmp_3;
 float old_tmp_4;
 float old_tmp_5;
 float old_tmp_6;
 

void setup()
{
  nh.initNode();
  nh.advertise(pub_range_1);
  nh.advertise(pub_range_2);
  nh.advertise(pub_range_3);
  nh.advertise(pub_range_4);
  nh.advertise(pub_range_5);
  nh.advertise(pub_range_6);

  range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_1.header.frame_id =  "ultrasound_front_left";
  range_msg_1.field_of_view = 0.3665;  // fake
  range_msg_1.min_range = 0.0;
  range_msg_1.max_range = MAX_DISTANCE;
  
  range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_2.header.frame_id =  "ultrasound_front_right";
  range_msg_2.field_of_view = 0.3665;  // fake
  range_msg_2.min_range = 0.0;
  range_msg_2.max_range = MAX_DISTANCE; 
   
  range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_3.header.frame_id =  "ultrasound_rear_left";
  range_msg_3.field_of_view = 0.3665;  // fake
  range_msg_3.min_range = 0.0;
  range_msg_3.max_range = MAX_DISTANCE;  
  
  range_msg_4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_4.header.frame_id =  "ultrasound_rear_right";
  range_msg_4.field_of_view = 0.3665;  // fake
  range_msg_4.min_range = 0.0;
  range_msg_4.max_range = MAX_DISTANCE; 
  
  range_msg_5.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_5.header.frame_id =  "ultrasound_left";
  range_msg_5.field_of_view = 0.3665;  // fake
  range_msg_5.min_range = 0.0;
  range_msg_5.max_range = MAX_DISTANCE; 
  
  range_msg_6.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_6.header.frame_id =  "ultrasound_right";
  range_msg_6.field_of_view = 0.3665;  // fake
  range_msg_6.min_range = 0.0;
  range_msg_6.max_range = MAX_DISTANCE;
}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    tmp=sonar_1.ping_cm();
    tmp=filtre(tmp,old_tmp_1);
    range_msg_1.range = tmp;
    range_msg_1.header.stamp = nh.now();
    pub_range_1.publish(&range_msg_1);
    old_tmp_1=tmp;

    tmp=sonar_2.ping_cm();
    tmp=filtre(tmp,old_tmp_2);
    range_msg_2.range = tmp;
    range_msg_2.header.stamp = nh.now();
    pub_range_2.publish(&range_msg_2);
    old_tmp_2=tmp;
    
    tmp=sonar_3.ping_cm();
    tmp=filtre(tmp,old_tmp_3);
    range_msg_3.range = tmp;
    range_msg_3.header.stamp = nh.now();
    pub_range_3.publish(&range_msg_3);
    old_tmp_3=tmp;

    tmp=sonar_4.ping_cm();
    tmp=filtre(tmp,old_tmp_4);
    range_msg_4.range = tmp;
    range_msg_4.header.stamp = nh.now();
    pub_range_4.publish(&range_msg_4);
    old_tmp_4=tmp;
    
    tmp=sonar_5.ping_cm();
    tmp=filtre(tmp,old_tmp_5);
    range_msg_5.range = tmp;
    range_msg_5.header.stamp = nh.now();
    pub_range_5.publish(&range_msg_5);
    old_tmp_5=tmp;

    tmp=sonar_6.ping_cm();
    tmp=filtre(tmp,old_tmp_6);
    range_msg_6.range = tmp;
    range_msg_6.header.stamp = nh.now();
    pub_range_6.publish(&range_msg_6);
    old_tmp_6=tmp;

    range_time =  millis() + 200;
  }
  nh.spinOnce();
}
float filtre(float tmp, float old_tmp){
  
  if ( tmp > 200 || tmp < 2){
    tmp=old_tmp;
    return tmp;
  }
  else{
    return tmp;
  }

}
