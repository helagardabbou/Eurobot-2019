 #include <PID_v1.h>
 #include <ecamlib.h>
#include <Encoder.h>
#include <FlexiTimer2.h>
#include <time.h>
#include <math.h> 
#include <ros.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle nh;

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
 
 Encoder motor_encoder_FL = Encoder(20, 42);
 Encoder motor_encoder_FR = Encoder(2, 40);
 Encoder motor_encoder_BL = Encoder(21, 38);
 Encoder motor_encoder_BR = Encoder(19, 36);
 const int ENCODER_TICKS_PER_REV = 3200;
 const int CADENCE_MS = 10;
volatile double dt = CADENCE_MS / 1000.;
  // Angular velocity
 volatile double angular_speed_FL = 0;
 volatile double angular_speed_FR = 0;
 volatile double angular_speed_BL = 0;
 volatile double angular_speed_BR = 0;
 

 volatile double vitesse_abs=0.1;
 volatile double distance=0;
 volatile double distance_in=0;
 volatile float coord_x=-10;
 volatile float coord_y=0;
 // PID 
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
 
 // Variable to keep track of the old encoder value
 volatile long old_encoder_FL = 0;
 volatile long old_encoder_FR = 0;
 volatile long old_encoder_BL = 0;
 volatile long old_encoder_BR = 0;


 
 float velocity_FL = 3;
 float velocity_FR = 3;
 float velocity_BL = 3;
 float velocity_BR = 3;

 double radius = 0.03;
 
 void setup() {

     Serial.begin(9600);
 
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
         
 }

 void loop() {
    delay(10);
    if (distance_in>distance){
        robot_stop();
        calcul_vitesse(coord_x,coord_y);
        
      }
      else{
        delay(10);
        run_robot();
        Serial.println(distance_in);
      }  
  
  
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
void robot_stop(){
  vitesse_abs=0;
  analogWrite(PWM_FL, abs(0));
  analogWrite(PWM_FR, abs(0));
  analogWrite(PWM_BL, abs(0));
  analogWrite(PWM_BR, abs(0));
  velocity_FL = 0;
  velocity_FR = 0;
  velocity_BL = 0;
  velocity_BR = 0;
  delay(1000);
  
}
void calcul_vitesse(double x_coord,double y_coord){
  vitesse_abs=0.1;
  distance = sqrt(square(x_coord)+square(y_coord));
  distance_in=0;
  double angle = atan2(x_coord,y_coord);
  double vx = vitesse_abs*sin(angle);
  double vy = vitesse_abs*cos(angle);
  velocity_FL = (1/radius)*(vx-vy);
  velocity_FR = (1/radius)*(vx+vy);
  velocity_BL = (1/radius)*(vx+vy);
  velocity_BR =(1/radius)*(vx-vy);
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


 
