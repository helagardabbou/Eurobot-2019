
/* Arduino Mega 2560 interrupt pins
Interrupt number: 0    1    2    3    4    5
physical pin:     2    3    21   20   19   18
*/
#include <FastPID.h>
#include "Arduino.h"
#include <Servo.h>
/* port mapping
       7  6  5  4  3  2  1  0
       -----------------------
PORTA: 29 28 27 26 25 24 23 22
PORTB: 13 12 11 10 50 51 52 53
PORTC: 30 31 32 33 34 35 36 37
PORTD: 38 -- -- -- 18 19 20 21
PORTE: -- -- 3  2  5  -- 1  0
PORTG: -- -- 4  -- -- 39 40 41
PORTH: -- 8  9  7  6  -- 16 17
PORTJ: -- -- -- -- -- -- 14 15
PORTL: 42 43 44 45 16 47 48 49
*/

// Macros for easier port access
#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))

// Quadrature Encoders
#define c_LeftEncoderPinA 2 // PORTE4
#define c_LeftEncoderPinB 3 // PORTE5
#define c_RightEncoderPinA 19  // PORTD2
#define c_RightEncoderPinB 18 // PORTD3

#define leftLed 4 // PORTH6
#define rightLed 5 // PORTH5

//Create Object fir servo motors
Servo monServomoteur;
Servo monServomoteur2;


volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev =0;
volatile bool _LeftEncoderBPrev =0;
volatile long _LeftEncoderTicks = 0;

volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile bool _RightEncoderAPrev =0;
volatile bool _RightEncoderBPrev =0;
volatile long _RightEncoderTicks = 0;

volatile  byte leftLedState = LOW;
volatile  byte rightLedState = LOW;

// Init left motors
const byte pwm_motorLeft = 9;
const byte dir_motorLeft = 11;

// Init right motor
const byte pwm_motorRight = 10;
const byte dir_motorRight = 7;


//Init PID
#define PIN_setpoint_lin_LIN  A2

float KpLin=0.2, KiLin=0, KdLin=0, Hz=20;
float KpRot=0.25, KiRot=0, KdRot=0;
int output_bits = 9;
bool output_signed = true;

FastPID myPIDLin(KpLin, KiLin, KdLin, Hz, output_bits, output_signed);
FastPID myPIDRot(KpRot, KiRot, KdRot, Hz, output_bits, output_signed);
FastPID myPID(KpRot, KiRot, KdRot, Hz, output_bits, output_signed);

//Init variables 
int vitesse;
int PWM_lin;
int PWM_rot;
int PWM_M_Left;
int PWM_M_Right;
int PWM_M_Max_Left;
int PWM_M_Max_Right;
int ts;
int tss; 


void setup()
{
  Serial.begin(9600);
  
  pinMode(c_LeftEncoderPinA, INPUT);     // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);     // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinB), HandleLeftMotorInterrupt, CHANGE);

  pinMode(c_RightEncoderPinA, INPUT);     // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinB, INPUT);     // sets pin B as input
  digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinA), HandleRightMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinB), HandleRightMotorInterrupt, CHANGE);

  pinMode(leftLed, OUTPUT);
  digitalWrite(leftLed, LOW);
  pinMode(rightLed, OUTPUT);
  digitalWrite(rightLed, LOW);

  pinMode(pwm_motorLeft, OUTPUT);
  pinMode(dir_motorLeft, OUTPUT);
  pinMode(pwm_motorRight, OUTPUT);
  pinMode(dir_motorRight, OUTPUT);
  
  
  //Check PID
  if (myPID.err()) {
    Serial.println("There is a configuration error!");
    for (;;) {}
  }

  TCCR2B = TCCR2B & 0b11111000 | 0x01; //Frequence of the Arduino Mega

  // Attache the servo motor to the pin D13
  monServomoteur.attach(13);
  monServomoteur2.attach(12);
}

void loop(){
  Rotation(91);
  stopwheels();
  delay(5000);


  //For the debbuging leds
  digitalWrite(leftLed, leftLedState);
  digitalWrite(rightLed, rightLedState);
}

void tige_gauche_activation(){
    // Fait bouger le bras de 0° à 180°
  //celui de GAUCHE
  monServomoteur2.write(90);
  delay(15);
  for (int position = 10; position <= 130; position++) {
    monServomoteur2.write(position);
    delay(15);
  }
}

void tige_droite_activation(){
  monServomoteur.write(90);
  delay(15);
  for (int position = 130; position >= 0; position--) {
    monServomoteur.write(position);
    delay(15);
  }
}


void stopwheels(){
  analogWrite(pwm_motorRight, 0);
  analogWrite(pwm_motorLeft, 0);
  
}


void Linear(double setpoint_lindis ){
  double LeftEncoderRel = _LeftEncoderTicks; //Relative value of the left encoder 
  double RightEncoderRel = _RightEncoderTicks; //Relative value of the right encoder 
  double setpoint_rot =0; //The robot can't turn
  //double actposition = (double)((_LeftEncoderTicks + _RightEncoderTicks))/2.0;
  
  double setpoint_lin= ((setpoint_lindis * 4000 )/(2* 3.14* 0.04)) ; //Conversion of the distance to ticks
  double error_lin =  setpoint_lin - 0;
  double error_rot =  setpoint_rot -0;
  double feedback_lin = 0;
  double feedback_rot = 0;
  while( abs(error_lin) > 180){
    error_lin = setpoint_lin - (double)((((_LeftEncoderTicks -LeftEncoderRel) + (_RightEncoderTicks-RightEncoderRel)))/2.0);
    error_rot = setpoint_rot - (double)((((_LeftEncoderTicks-LeftEncoderRel) - (_RightEncoderTicks-RightEncoderRel)))/2.0);
   feedback_lin = (double)((((_LeftEncoderTicks-LeftEncoderRel) + (_RightEncoderTicks-RightEncoderRel)))/2.0);
   feedback_rot =(double)((((_LeftEncoderTicks-LeftEncoderRel) - (_RightEncoderTicks-RightEncoderRel)))/2.0);
    ts = micros();
    PWM_lin = myPIDLin.step(setpoint_lin, feedback_lin);
    PWM_rot = myPIDRot.step(setpoint_rot, feedback_rot);
    tss = micros();
    PWM_M_Left = PWM_lin + PWM_rot;
    PWM_M_Right= PWM_lin - PWM_rot;
    
    if(PWM_M_Left > 0) digitalWrite(dir_motorLeft, HIGH);
    else digitalWrite(dir_motorLeft, LOW);
    if(PWM_M_Right > 0) digitalWrite(dir_motorRight, HIGH);
    else digitalWrite(dir_motorRight, LOW);
    
    if (abs(PWM_M_Left)-abs(PWM_M_Right) > 10){
      if(abs(PWM_M_Left)<abs(PWM_M_Right)){
          PWM_M_Max_Left = 170;
          PWM_M_Max_Right = 240;
        } 
      else {
          PWM_M_Max_Left = 240;
          PWM_M_Max_Right = 170;
            }
    }
    else{
          PWM_M_Max_Left = 170;
          PWM_M_Max_Right = 170;
    }
        
        Serial.print(" LeftEncoderRel: "); 
        Serial.print(LeftEncoderRel);
        Serial.print(" RightEncoderRel: "); 
        Serial.print(RightEncoderRel);
        Serial.print(" LeftEncoder: "); 
        Serial.print(_LeftEncoderTicks);
        Serial.print(" RightEncoder: "); 
        Serial.print(_RightEncoderTicks);
        Serial.print(" PWMLeft: "); 
        Serial.print(PWM_M_Left);
        Serial.print(" PWMRightt: "); 
        Serial.println(PWM_M_Right);
  
    if(abs(PWM_M_Left) > PWM_M_Max_Left) PWM_M_Left = PWM_M_Max_Left;
    if(abs(PWM_M_Right) > PWM_M_Max_Right) PWM_M_Right = PWM_M_Max_Right;

      analogWrite(pwm_motorRight, abs(PWM_M_Right));
      analogWrite(pwm_motorLeft, abs(PWM_M_Left));

        Serial.print(" Setpoint: ");
        Serial.print(setpoint_lin);//4000 Counts Per Revolution   
        Serial.print(" Error_Lin: ");
        Serial.print(error_lin);//4000 Counts Per Revolution    
        Serial.print("  RevolutionsLeft: ");
        Serial.print(_LeftEncoderTicks);//4000 Counts Per Revolution
        Serial.print("  RevolutionsRight: ");
        Serial.print(_RightEncoderTicks);//4000 Counts Per Revolution
        Serial.print(" PWMLeft: "); 
        Serial.print(PWM_M_Left);
        Serial.print(" PWMRightt: "); 
        Serial.println(PWM_M_Right);
    
  }
  stopwheels();
}


void Rotation( double angleConsigne){
  double LeftEncoderRel = _LeftEncoderTicks; //Relative value of the left encoder 
  double RightEncoderRel = _RightEncoderTicks; //Relative value of the right encoder 
  //double actposition = (double)((_LeftEncoderTicks + _RightEncoderTicks))/2.0;
  
  double distance_rot = (angleConsigne /360)*3.14*2*0.1775; //Conversion of the angle into distance 
  double setpoint_rot= ((distance_rot * 4000 )/(2* 3.14* 0.04)) ; //Conversion of the distance to ticks
  double setpoint_lin =0;
  double error_lin =  setpoint_lin - 0;
  double error_rot =  setpoint_rot -0;
  double feedback_lin = 0;
  double feedback_rot = 0;
  while( abs(error_rot) > 180){
    error_lin = setpoint_lin - (double)(((_LeftEncoderTicks-LeftEncoderRel) + (_RightEncoderTicks-RightEncoderRel)))/2.0;
    error_rot = setpoint_rot - (double)(((_LeftEncoderTicks-LeftEncoderRel) - (_RightEncoderTicks-RightEncoderRel)))/2.0;
   feedback_lin = (double)(((_LeftEncoderTicks-LeftEncoderRel) + (_RightEncoderTicks-RightEncoderRel)))/2.0;
   feedback_rot =(double)(((_LeftEncoderTicks-LeftEncoderRel) - (_RightEncoderTicks-RightEncoderRel)))/2.0;
    ts = micros();
    PWM_lin = myPIDLin.step(setpoint_lin, feedback_lin);
    PWM_rot = myPIDRot.step(setpoint_rot, feedback_rot);
    tss = micros();
    PWM_M_Left = PWM_lin + PWM_rot;
    PWM_M_Right =PWM_lin - PWM_rot;
    
    if(PWM_M_Left > 0) digitalWrite(dir_motorLeft, HIGH);
    else digitalWrite(dir_motorLeft, LOW);
    if(PWM_M_Right > 0) digitalWrite(dir_motorRight, HIGH);
    else digitalWrite(dir_motorRight, LOW);
    
    if (abs(PWM_M_Left)-abs(PWM_M_Right) > 10){
      if(abs(PWM_M_Left)<abs(PWM_M_Right)){
          PWM_M_Max_Left = 170;
          PWM_M_Max_Right = 240;
        } 
      else {
          PWM_M_Max_Left = 240;
          PWM_M_Max_Right = 170;
            }
    }
    else{
          PWM_M_Max_Left = 170;
          PWM_M_Max_Right = 170;
    }
    if(abs(PWM_M_Left) > PWM_M_Max_Left) PWM_M_Left = PWM_M_Max_Left;
    if(abs(PWM_M_Right) > PWM_M_Max_Right) PWM_M_Right = PWM_M_Max_Right;

      analogWrite(pwm_motorRight, abs(PWM_M_Right));
      analogWrite(pwm_motorLeft, abs(PWM_M_Left));
      
  Serial.print(" Error_rot: ");
  Serial.print(error_rot);//4000 Counts Per Revolution    
  Serial.print("  RevolutionsLeft: ");
  Serial.print(_LeftEncoderTicks);//4000 Counts Per Revolution
  Serial.print("  RevolutionsRight: ");
  Serial.print(_RightEncoderTicks);//4000 Counts Per Revolution
  Serial.print(" PWMLeft: "); 
  Serial.print(PWM_M_Left);
  Serial.print(" PWMRightt: "); 
  Serial.println(PWM_M_Right);
    
  }
  stopwheels();
}


void HandleLeftMotorInterrupt(){
  _LeftEncoderBSet = ((PINE & B00100000)>>5); //digitalRead(3) PE5
  _LeftEncoderASet = ((PINE & B00010000)>>4); //digitalRead(2) PE4
  
  _LeftEncoderTicks+=ParseLeftEncoder();
 
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

void HandleRightMotorInterrupt(){
  _RightEncoderBSet = ((PIND & B00000100)>>2); // digitalRead(20) PD2
  _RightEncoderASet = ((PIND & B00001000)>>3); // digitalRead(21) PD3
  
  _RightEncoderTicks+=ParseRightEncoder();
  
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}

int ParseLeftEncoder(){
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }
  return 0;
}

int ParseRightEncoder(){
  if(_RightEncoderAPrev && _RightEncoderBPrev){
    leftLedState = HIGH;
    rightLedState = HIGH;
    if(!_RightEncoderASet && _RightEncoderBSet) return 1;
    if(_RightEncoderASet && !_RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && _RightEncoderBPrev){
    leftLedState = LOW;
    rightLedState = HIGH;
    if(!_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && !_RightEncoderBPrev){
    leftLedState = LOW;
    rightLedState = LOW;
    if(_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(!_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(_RightEncoderAPrev && !_RightEncoderBPrev){
    leftLedState = HIGH;
    rightLedState = LOW;
    if(_RightEncoderASet && _RightEncoderBSet) return 1;
    if(!_RightEncoderASet && !_RightEncoderBSet) return -1;
  }
  return 0;
}
