#include <ros.h>
#include <geometry_msgs/Point.h>   // float x, float y, float z
#include <std_msgs/Bool.h>

volatile float dicky[3] = {0, 0, 0};
int counter = 0;

ros::NodeHandle nh;

geometry_msgs::Point str_msg;
std_msgs::Bool cuming;
ros::Publisher chatter("test_coords", &str_msg);
ros::Publisher cumer("curry_arrived", &cuming);

void messageCb( const geometry_msgs::Point& toggle_msg){
  dicky[0] = toggle_msg.x;
  dicky[1] = toggle_msg.y;
  dicky[2] = toggle_msg.z;
}

ros::Subscriber<geometry_msgs::Point> sub("required_coords", &messageCb );

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.advertise(cumer);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (counter % 3 == 0) {
    cuming.data = true;
  }
  else {
    cuming.data = false;
  }
    
  str_msg.x = dicky[0];
  str_msg.y = dicky[1];
  str_msg.z = dicky[2];
  
  chatter.publish( &str_msg );
  cumer.publish( &cuming );
  counter++;
  nh.spinOnce();
  delay(1000);
}
