 #include <PID_v1.h>
 #include <ecamlib.h>
#include <Encoder.h>
#include <FlexiTimer2.h>
#include <time.h>
#include <math.h> 
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
// statement of functions // 
void quad();
void robot_stop();
void isrt();
void navigation();
void rotation(float angle);
void run_robot( float x_coord,float y_coord);
void calcul_vitesse(float x_coord,float y_coord);


volatile float required_coords[3] = {0, 0, 0};




// Wiring 
// FL motor
// HBridge ENA : 3; IN1 : 28; IN2 : 26 
// Motor YELLOW : 42; WHITE : 20
 // FR motor
// HBridge ENA : 5; IN3 : 22; IN4 : 24 
// Motor YELLOW : 40; WHITE : 2
 // BL motor
// HBridge ENA : 11; IN3 : 29; IN4 : 27 
// Motor YELLOW : 38; WHITE : 21
 // BR motor
// HBridge ENA : 6; IN1 : 25; IN2 : 23 
// Motor YELLOW : 36; WHITE : 19
 // Front Left motor pins 
#define PWM_FL 3
#define IN1_FL 52
#define IN2_FL 50
 // Front Right motor pins
#define PWM_FR 5
#define IN1_FR 46
#define IN2_FR 48
//IN3=29 et IN4=27
 // Back Left motor pins
#define PWM_BL 11
#define IN1_BL 28
#define IN2_BL 26
 // Back Right motor pins
#define PWM_BR 6
#define IN1_BR 24
#define IN2_BR 22 

 // statement of the encoder // 
 
 Encoder motor_encoder_FL = Encoder(20, 42);
 Encoder motor_encoder_FR = Encoder(2, 40);
 Encoder motor_encoder_BL = Encoder(21, 38);
 Encoder motor_encoder_BR = Encoder(19, 36);
 
 //configuration of the encoder//
 
 const int ENCODER_TICKS_PER_REV = 3200;
 
 // configuration of the PID //
 
 const int CADENCE_MS = 10;
 volatile float dt = CADENCE_MS / 1000.;
 float gain_p = 80.0;
 float gain_i = 5.0;
 float gain_d = 0.0;  
 double setpoint_FL, input_FL, output_FL;
 PID PID_FL(&input_FL, &output_FL, &setpoint_FL,gain_p,gain_i,gain_d, DIRECT);
 double setpoint_FR, input_FR, output_FR;
 PID PID_FR(&input_FR, &output_FR, &setpoint_FR,gain_p,gain_i,gain_d, DIRECT);
 double setpoint_BL, input_BL, output_BL;
 PID PID_BL(&input_BL, &output_BL, &setpoint_BL,gain_p,gain_i,gain_d, DIRECT);
 double setpoint_BR, input_BR, output_BR;
 PID PID_BR(&input_BR, &output_BR, &setpoint_BR,gain_p,gain_i,gain_d, DIRECT);
 
 // Angular velocity//
 
 volatile float angular_speed_FL = 0;
 volatile float angular_speed_FR = 0;
 volatile float angular_speed_BL = 0;
 volatile float angular_speed_BR = 0;


 

 
 // Variable to keep track of the old encoder value
 volatile long old_encoder_FL = 0;
 volatile long old_encoder_FR = 0;
 volatile long old_encoder_BL = 0;
 volatile long old_encoder_BR = 0;
 volatile bool ack;


 // requested speed //
 volatile float velocity_FL = 0;
 volatile float velocity_FR = 0;
 volatile float velocity_BL = 0;
 volatile float velocity_BR = 0;
 
 volatile float velocity_FL_old = 0;
 volatile float velocity_FR_old = 0;
 volatile float velocity_BL_old = 0;
 volatile float velocity_BR_old = 0;
// radius of the wheel //
 float radius = 0.03;
 // angle of rotation // 
 volatile float angle;
 // requested distance //
 volatile float distance=0;
 // Real-time distance //
 volatile float distance_in=0;
 
 volatile bool isrt_end = false;
 volatile bool arrive = false;
 volatile bool US_active =false;
 volatile bool mdr =false;

 ros::NodeHandle nh;


std_msgs::Bool arrival_msg;
volatile float required_x;
volatile float required_y;
volatile float vitesse_abs=0.1;
volatile int quadran=2;

void requiredCoords( const geometry_msgs::Point& cmd_msg){
  
  required_x = cmd_msg.x;
  required_y = cmd_msg.y;
  calcul_vitesse(cmd_msg.x,cmd_msg.y);
  //quad();
}


// ToDo: a function to compare the real coords and the required ones

void stopUSFrontLeft( const sensor_msgs::Range& stop_us_fl_msg){
  if (10<stop_us_fl_msg.range < 20.00) {
    //robot_stop();
  }
}

void stopUSFrontRight( const sensor_msgs::Range& stop_us_fr_msg){
  if (10<stop_us_fr_msg.range < 20.00) {
    //robot_stop();
  }
}

void stopUSLeft( const sensor_msgs::Range& stop_us_l_msg){
  if (10<stop_us_l_msg.range < 20.00) {
    //robot_stop();
  }
}

void stopUSRight( const sensor_msgs::Range& stop_us_r_msg){
  if (10<stop_us_r_msg.range < 20.00) {
    //robot_stop();
  }
}

void stopUSRearLeft( const sensor_msgs::Range& stop_us_rl_msg){
  if (stop_us_rl_msg.range < 20.00) {
      US_active =true;
  }
  else{
      US_active =false;
  }
}

void stopUSRearRight( const sensor_msgs::Range& stop_us_rr_msg){
  if (stop_us_rr_msg.range < 20.00) {
      US_active =true;
  }
  else{
      US_active =false;
  }
}



ros::Subscriber<geometry_msgs::Point> sub1("required_coords", &requiredCoords );
ros::Subscriber<sensor_msgs::Range> sub2("ultrasound_front_left", &stopUSFrontLeft );
ros::Subscriber<sensor_msgs::Range> sub3("ultrasound_front_right", &stopUSFrontRight );
ros::Subscriber<sensor_msgs::Range> sub4("ultrasound_left", &stopUSLeft );
ros::Subscriber<sensor_msgs::Range> sub5("ultrasound_right", &stopUSRight );
ros::Subscriber<sensor_msgs::Range> sub6("ultrasound_rear_left", &stopUSRearLeft );
ros::Subscriber<sensor_msgs::Range> sub7("ultrasound_rear_right", &stopUSRearRight );



ros::Publisher pub("curry_arrived", &arrival_msg );
 
 void setup() {

     nh.initNode();
     nh.subscribe(sub1);
     nh.subscribe(sub2);
     nh.subscribe(sub3);
     nh.subscribe(sub4);
     nh.subscribe(sub5);
     nh.subscribe(sub6);
     nh.subscribe(sub7);
     nh.advertise(pub);
     
     Serial.begin(57600);
 
     pinMode(PWM_FL,OUTPUT);
     pinMode(IN1_FL,OUTPUT); 
     pinMode(IN2_FL,OUTPUT); 
     pinMode(PWM_FR,OUTPUT);
     pinMode(IN1_FR,OUTPUT); 
     pinMode(IN2_FR,OUTPUT);  
     pinMode(PWM_BL,OUTPUT);
     pinMode(IN1_BL,OUTPUT); 
     pinMode(IN2_BL,OUTPUT);
     pinMode(PWM_BR,OUTPUT);
     pinMode(IN1_BR,OUTPUT); 
     pinMode(IN2_BR,OUTPUT);  
 
     FlexiTimer2::set(CADENCE_MS, isrt); // Periodic execution of isrt() function
     FlexiTimer2::start();
 
     PID_FL.SetMode(AUTOMATIC);
     PID_FL.SetSampleTime(CADENCE_MS);
     PID_FL.SetOutputLimits(-255,255);
     PID_FR.SetMode(AUTOMATIC);
     PID_FR.SetSampleTime(CADENCE_MS);
     PID_FR.SetOutputLimits(-255,255);
     PID_BR.SetMode(AUTOMATIC);
     PID_BR.SetSampleTime(CADENCE_MS);
     PID_BR.SetOutputLimits(-255,255);
     PID_BL.SetMode(AUTOMATIC);
     PID_BL.SetSampleTime(CADENCE_MS);   
     PID_BL.SetOutputLimits(-255,255);
     distance_in=0;
     distance=0;
     isrt_end=false;
     arrive=false;
     US_active =false;
     mdr=false;
         
 }

 void loop() {
      if(isrt_end){
        if((distance_in > distance)&&!arrive){
              arrival_msg.data = true;
              pub.publish( &arrival_msg );
              robot_stop_dist();
              arrive=true;
        }
        else{
          if(US_active && !mdr){
            robot_stop();
            mdr=true;
          }
        
          else if(!US_active  && mdr){
            mdr=false;
            vitesse_abs=0.1;
            velocity_BR=velocity_BR_old;
            velocity_FR=velocity_FR_old;
            velocity_BL=velocity_BL_old;
            velocity_FL=velocity_FL_old;
          }
          else if (US_active  && mdr){
            robot_stop();
          }
        }
        
        isrt_end = false;
        
      }
      run_robot();
      nh.spinOnce();
      
 }
void run_robot(){
    if(abs(angular_speed_FL) < abs(velocity_FL)*0.3 || abs(angular_speed_FR) < abs(velocity_FR)*0.3||abs(angular_speed_BL) < abs(velocity_BL)*0.3||abs(angular_speed_BR) < abs(velocity_BR)*0.3){
          PID_FL.SetOutputLimits(-255*0.3,255*0.3);
          PID_FR.SetOutputLimits(-255*0.3,255*0.3);
          PID_BL.SetOutputLimits(-255*0.3,255*0.3);
          PID_BR.SetOutputLimits(-255*0.3,255*0.3);
      }
      else{
        PID_FL.SetOutputLimits(-255,255);
        PID_FR.SetOutputLimits(-255,255);
        PID_BL.SetOutputLimits(-255,255);
        PID_BR.SetOutputLimits(-255,255);
      }
      navigation();

}
void robot_stop_dist(){
  vitesse_abs=0;
  velocity_FL = 0;
  velocity_FR = 0;
  velocity_BL = 0;
  velocity_BR = 0;
  
  velocity_BR_old=velocity_BR;
  velocity_FR_old=velocity_FR;
  velocity_FL_old=velocity_FL;
  velocity_BL_old=velocity_BL;
  
}
void robot_stop(){
  vitesse_abs=0;
  velocity_FL = 0;
  velocity_FR = 0;
  velocity_BL = 0;
  velocity_BR = 0;
  
  
}
void calcul_vitesse(float x_coord,float y_coord){
  arrive=false;
  vitesse_abs=0.1;
  distance_in=0;
  distance = sqrt(square(x_coord)+square(y_coord));
  angle = atan2(x_coord,y_coord);
  float vx = vitesse_abs*sin(angle);
  float vy = vitesse_abs*cos(angle);
  velocity_FL = (1/radius)*(vx-vy);
  velocity_FR = (1/radius)*(vx+vy);
  velocity_BL = (1/radius)*(vx+vy);
  velocity_BR =(1/radius)*(vx-vy);
  
  velocity_BR_old=velocity_BR;
  velocity_FR_old=velocity_FR;
  velocity_FL_old=velocity_FL;
  velocity_BL_old=velocity_BL;
}


void rotation(float angle){
  arrive=false;
  vitesse_abs=0.1*3.14*2;
  distance_in=0;
  if (angle > 0){
    velocity_FL = (1/radius)*(-(0.25+0.18))*vitesse_abs;
    velocity_FR = (1/radius)*(0.25+0.18)*vitesse_abs;
    velocity_BL = (1/radius)*(-(0.25+0.18))*vitesse_abs;
    velocity_BR = (1/radius)*(0.25+0.18)*vitesse_abs;
  }
  else{
    vitesse_abs=-vitesse_abs;
    velocity_FL = (1/radius)*(-(0.25+0.18))*vitesse_abs;
    velocity_FR = (1/radius)*(0.25+0.18)*vitesse_abs;
    velocity_BL = (1/radius)*(-(0.25+0.18))*vitesse_abs;
    velocity_BR = (1/radius)*(0.25+0.18)*vitesse_abs;
  }
  float rad=abs(angle)*(3.14/180);
  float temp = vitesse_abs/rad;
  distance = 0.1*temp;
  
}
void quad(){
  float degree = (1/3.14)*angle *180;
  if ( 0 < degree && degree <= 60){
    quadran =1;
  }
  if ( 60 < degree && degree <= 120){
    quadran =2;
  }
  
  if( -60 < degree && degree <= 0){
    quadran = 6;
  }
  if ( -120 < degree && degree <= -60){
    quadran =5;
  }
  if( 120 < degree && degree <= 180){
    quadran=3;
    
  }
  if(-179 < degree && degree <= -120){
    quadran=4;
  }
  

}
 
// this function is registered as an event, see setup()
// Speed measurement 
void isrt(){
    // Motor FL
    // FL motor

    distance_in+=(vitesse_abs*dt)*100;
    int delta_encoder_FL = motor_encoder_FL.read() - old_encoder_FL;
    old_encoder_FL = motor_encoder_FL.read();
    angular_speed_FL = ( (2.0 * 3.141592 * (double)delta_encoder_FL) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
     // Motor FR
    // FR motor
    int delta_encoder_FR = motor_encoder_FR.read() - old_encoder_FR;
    old_encoder_FR = motor_encoder_FR.read();
    angular_speed_FR = -( (2.0 * 3.141592 * (double)delta_encoder_FR) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
     // Motor BL
    // BL motor
    int delta_encoder_BL = motor_encoder_BL.read() - old_encoder_BL;
    old_encoder_BL = motor_encoder_BL.read();
    angular_speed_BL = ( (2.0 * 3.141592 * (double)delta_encoder_BL) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
     // Motor BR
    // BR motor
    int delta_encoder_BR = motor_encoder_BR.read() - old_encoder_BR;
    old_encoder_BR = motor_encoder_BR.read();
    angular_speed_BR = -( (2.0 * 3.141592 * (double)delta_encoder_BR) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
    
    isrt_end=true;
    

    

 }
  
  void navigation(){
     
     setpoint_FL = velocity_FL;
     setpoint_FR = velocity_FR;
     setpoint_BR = velocity_BR;
     setpoint_BL = velocity_BL;
     
     input_FL = angular_speed_FL;
     input_FR = angular_speed_FR;
     input_BL = angular_speed_BL;
     input_BR = angular_speed_BR;
     
     PID_FL.Compute();
     PID_FR.Compute();
     PID_BL.Compute();
     PID_BR.Compute();


     

    
       // Handle direction with H bridge pin writing 
     if (output_FL > 0 ){
      digitalWrite(IN1_FL, HIGH);
      digitalWrite(IN2_FL, LOW);
     }
     else {
       digitalWrite(IN1_FL, LOW);
       digitalWrite(IN2_FL, HIGH);
     }
 
     if (output_FR > 0 ){
       digitalWrite(IN1_FR, LOW);
       digitalWrite(IN2_FR, HIGH);
     }
     else {
       digitalWrite(IN1_FR, HIGH);
       digitalWrite(IN2_FR, LOW);
     }
 
     if (output_BL > 0 ){
       digitalWrite(IN1_BL, HIGH);
       digitalWrite(IN2_BL, LOW);
     }
     else {
       digitalWrite(IN1_BL, LOW);
       digitalWrite(IN2_BL, HIGH);
     }
 
     if (output_BR >  0 ){
       digitalWrite(IN1_BR, LOW);
       digitalWrite(IN2_BR, HIGH);
     }
     else {
       digitalWrite(IN1_BR, HIGH);
       digitalWrite(IN2_BR, LOW);
     }
     
     analogWrite(PWM_FL, abs(output_FL));
     analogWrite(PWM_FR, abs(output_FR));
     analogWrite(PWM_BL, abs(output_BL));
     analogWrite(PWM_BR, abs(output_BR));  
     
 }


 
