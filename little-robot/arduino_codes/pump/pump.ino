#include <SoftwareSerial.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#define AX_START 255
#define AX_GOAL_LENGTH 5
#define AX_WRITE_DATA 3
#define AX_GOAL_POSITION_L 30
#define BaudRate (57600ul)

#define ID (4u) /*see on servo stickers*/

#define VALVE 7

#define PUMP 4

#define RAMP 9

int val;
int initial_pos = 512;
int in_pos = 205;
int out_pos = 819;

volatile int storage = 0;
volatile int etape;

SoftwareSerial mySerial(10, 11); // RX, TX
Servo Ramp_servo;

ros::NodeHandle nh;

std_msgs::Bool arrival_msg;

ros::Publisher pub("curry_arrived", &arrival_msg );

// === Information ===
/*
 * A4,A5: I2C for RGB captor
 * 
 * D4: Pump
 * D5,D6,D7: Valve
 * 
 * D10,D11: Serial for motor
 * 
 * pump 1/0: HIGH/LOW
 * valve 1/0: open/close
 * 
 * rotation motor 300° on 1024 bits
 * rotation of 90° = 1024/300*90 = 307
 */
 
// === Functions ===

// pump-valve

void pump_on(){
  digitalWrite(PUMP, HIGH);
}

void pump_off(){
  digitalWrite(PUMP, LOW);
}

void valve_open(){
  digitalWrite(VALVE, LOW);
}

void valve_close(){
  digitalWrite(VALVE, HIGH);
}

// arm

int arm_move(unsigned char id, int Position) // cf library AX12A
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

    const unsigned int l = 9;
    unsigned char packet[l];

    unsigned char Checksum = (~(id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = id;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;

    return (mySerial.write(packet, l));
}

// ramp

/* SET VALUE depending of tests */
void ramp_close(){
  Ramp_servo.write(0);
}

void ramp_open(){
  Ramp_servo.write(512);
}
/* * * * * */

// all


void show_bob(){
  arm_move(ID, in_pos);
  delay(1000);
  // 1s
  arrival_msg.data = true;
  pub.publish( &arrival_msg );
  delay(10);

}

void take_puck(){
 
  if (storage < 2){
    pump_on();
    delay(1000);
    arm_move(ID, in_pos);
    delay(1000);
    valve_open();
    delay(1000);
    arm_move(ID, out_pos);
    delay(2000);
    pump_off();
    delay(1000);
    arm_move(ID, in_pos);
    delay(2000);
    //6s
    storage=storage+1;
    arrival_msg.data = true;
    pub.publish( &arrival_msg );
    delay(10);

    
  }
  else if (storage == 2){
    pump_on();
    delay(2000);
    arm_move(ID, in_pos);
    delay(2000);
    valve_open();
    delay(1000);
    arm_move(ID, initial_pos);
    delay(2000);
    //6s
    storage=storage+1;
    arrival_msg.data = true;
    pub.publish( &arrival_msg );
    delay(10);  

  }
  else {
    arrival_msg.data = false;
    pub.publish( &arrival_msg );
    delay(10);
    
  }
}

void flush_puck(){
    pump_on();
    delay(1000);
    ramp_open();
    delay(2000);
    arm_move(ID, out_pos);
    delay(2000);
    pump_off();
    delay(2000);
    arm_move(ID, in_pos);
    delay(2000);
    ramp_close();
    delay(2000);
    //11s
    storage = 0;
    arrival_msg.data = true;
    pub.publish( &arrival_msg );
    delay(10);

}

// ROS
void msgTake( const std_msgs::Empty& toggle_msg){
  if(etape==2||etape==3||etape==4){
    etape=etape+1;
    take_puck();
    
  }
}

void msgFlush( const std_msgs::Empty& toggle_msg){
  if(etape==5){
    etape=1;
    flush_puck();
    
  }
}

void msgStart( const std_msgs::Empty& toggle_msg){
  if(etape==1){
    etape=etape+1;
    show_bob();
    
  }
}

// === Main ===

ros::Subscriber<std_msgs::Empty> sub_take("take_puck", &msgTake );
ros::Subscriber<std_msgs::Empty> sub_flush("flush_pucks", &msgFlush );
ros::Subscriber<std_msgs::Empty> sub_start("show_bob", &msgStart );

void setup() {
  // put your setup code here, to run once:

  // Ros config
  Serial.begin(57600); // opens serial port, sets data rate to 57600 bps for ros
  nh.initNode();
  nh.subscribe(sub_take);
  nh.subscribe(sub_flush);
  nh.subscribe(sub_start);
  nh.advertise(pub);

  // Valve config
  pinMode(VALVE, OUTPUT);

  // Pump config
  pinMode(PUMP, OUTPUT);

  // Motor config
  mySerial.begin(57600);
  delay(1000);
  arm_move(ID, initial_pos);
  delay(1000);
  // Ramp config
  Ramp_servo.attach(RAMP);
  pump_off();
  storage=0;
  etape=1;
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(10);
}
