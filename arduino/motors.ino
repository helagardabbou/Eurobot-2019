/* Arduino Mega 2560 interrupt pins
   Interrupt number: 0    1    2    3    4    5
   physical pin:     2    3    21   20   19   18

             port mapping
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

/************************/
/* Libraries to include */
/************************/

#include <FastPID.h>
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

// Macros for easier faster port writing
// Not used in this code anymore, but could be useful to write to leds faster (with some clever code)
#define SET(x,y) (x|=(1<<y))    // usage: SET(PORTA,7) is the same as digitalWrite(29,HIGH)
#define CLR(x,y) (x&=(~(1<<y))) // usage: CLR(PORTE,1) is the same as digitalWrite(1,LOW)

/****************************************/
/* Define constants such as pin numbers */
/***************************************/

// Quadrature Encoders
#define c_LeftEncoderPinA 2 // PORTE4
#define c_LeftEncoderPinB 3 // PORTE5
#define c_RightEncoderPinA 19  // PORTD2
#define c_RightEncoderPinB 18 // PORTD3

// LEDs
#define c_LeftLedPin 4 // PORTH6
#define c_RightLedPin 5 // PORTH5

// Servo
#define c_LeftServoPin 22
#define c_RightServoPin 24

// Motors
#define c_LeftMotorPwmPin 9
#define c_LeftMotorDirPin 11
#define c_RightMotorPwmPin 10
#define c_RightMotorDirPin 7

// PID parameters
#define output_bits 9
#define output_signed true

/*********************/
/* Declare variables */
/*********************/

// Left encoder
volatile bool _LeftEncoderACur;
volatile bool _LeftEncoderBCur;
volatile bool _LeftEncoderAPrev = 0;
volatile bool _LeftEncoderBPrev = 0;
volatile long _LeftEncoderTicks = 0;

// Right encoder
volatile bool _RightEncoderACur;
volatile bool _RightEncoderBCur;
volatile bool _RightEncoderAPrev = 0;
volatile bool _RightEncoderBPrev = 0;
volatile long _RightEncoderTicks = 0;

// LED states
// Used so we can write to the pins in the main loop and keep interruptions as simple and short as possible
volatile  byte c_LeftLedPinState = LOW;
volatile  byte c_RightLedPinState = LOW;

// PWM
int PWM_lin;
int PWM_rot;
int PWM_M_Left;
int PWM_M_Right;
int PWM_M_Max_Left;
int PWM_M_Max_Right;

// set PID constants
float KpLin=0.2, KiLin=0, KdLin=0, Hz=20;
float KpRot=0.25, KiRot=0, KdRot=0;

// ROS messages
float dist_msg;
float turn_msg;
String arm_msg;
bool stop_condition = false;
std_msgs::Empty feedback_msg;
geometry_msgs::Vector3 encoder_msg;

/*******************/
/* Object creation */
/*******************/

// PID
FastPID diff_PID_lin(KpLin, KiLin, KdLin, Hz, output_bits, output_signed);
FastPID diff_PID_rot(KpRot, KiRot, KdRot, Hz, output_bits, output_signed);
FastPID diff_PID(KpRot, KiRot, KdRot, Hz, output_bits, output_signed);

// Mechanical arms
Servo right_arm;
Servo left_arm;

ros::NodeHandle nh;

// callback functions are executed each time nh.sipnOnce() is run and there is a new message on the topic
void lin_cb(const std_msgs::Float32& msg) {
    dist_msg = msg.data;
    Linear(dist_msg);
    pub_feedback.publish(&feedback_msg); // publish empty feedback message so we know the action has been completed
}

void rot_cb(const std_msgs::Float32& msg) {
    turn_msg = msg.data;
    Rotation(turn_msg);
    pub_feedback.publish(&feedback_msg);
}

void arm_cb(const std_msgs::String& msg) {
    arm_msg = msg.data;
    if(strcmp(arm_msg.c_str(), "left") == 0) {
        activate_left_arm();
        pub_feedback.publish(&feedback_msg);
    }
    else if(strcmp(arm_msg.c_str(), "right") == 0) {
        activate_right_arm();
        pub_feedback.publish(&feedback_msg);
    }
}

void stop_cb(const std_msgs::String& msg) {
    if(strcmp(msg.data, "stop") == 0) {
        stop_condition = true;
        stopwheels();
    }
    else
        stop_condition = false;
}

// ROS listeners
ros::Subscriber<std_msgs::Float32> sub_lin("cmd_lin", &lin_cb );
ros::Subscriber<std_msgs::Float32> sub_rot("cmd_rot", &rot_cb );
ros::Subscriber<std_msgs::String> sub_arm("cmd_arm", &arm_cb );
ros::Subscriber<std_msgs::String> sub_stop("cmd_stop", &stop_cb );

// ROS publishers
ros::Publisher pub_encoder("encoder_ticks", &encoder_msg);
ros::Publisher pub_feedback("cmd_feedback", &feedback_msg);

void setup() {
    Serial.begin(57600);

    // init ROS
    nh.initNode();
    nh.subscribe(sub_lin);
    nh.subscribe(sub_rot);
    nh.subscribe(sub_arm);
    nh.subscribe(sub_stop);
    nh.advertise(pub_encoder);
    nh.advertise(pub_feedback);

    /******************************/
    /* Declare inputs and outputs */
    /******************************/

    pinMode(c_LeftEncoderPinA, INPUT);     // sets pin A as input
    digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
    pinMode(c_LeftEncoderPinB, INPUT);     // sets pin B as input
    digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
    attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterrupt, CHANGE);

    pinMode(c_RightEncoderPinA, INPUT);     // sets pin A as input
    digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
    pinMode(c_RightEncoderPinB, INPUT);     // sets pin B as input
    digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
    attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinA), HandleRightMotorInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinB), HandleRightMotorInterrupt, CHANGE);

    pinMode(c_LeftLedPin, OUTPUT);
    digitalWrite(c_LeftLedPin, LOW);
    pinMode(c_RightLedPin, OUTPUT);
    digitalWrite(c_RightLedPin, LOW);

    pinMode(c_LeftMotorPwmPin, OUTPUT);
    pinMode(c_LeftMotorDirPin, OUTPUT);
    pinMode(c_RightMotorPwmPin, OUTPUT);
    pinMode(c_RightMotorDirPin, OUTPUT);

    //Check PID
    if(diff_PID.err()) {
        nh.logerror("There is a configuration error!");
        for (;;) {}
    }

    // Change the Arduino Mega's pwm frequency on pin 9 and 10 to 31.37255kHz instead of the default 490.20Hz.
    TCCR2B = TCCR2B & 0b11111000 | 0x01;

    right_arm.attach(c_RightServoPin);
    left_arm.attach(c_RightServoPin);
}

void loop() {
    // Publish encoder data over ROS
    encoder_msg.x = _LeftEncoderTicks;
    encoder_msg.y = _RightEncoderTicks;
    pub_encoder.publish(&encoder_msg);

    nh.spinOnce();

    digitalWrite(c_LeftLedPin, c_LeftLedPinState);
    digitalWrite(c_RightLedPin, c_RightLedPinState);
}

void activate_left_arm() {
    left_arm.write(90);
    delay(15);
    for (int position = 110; position >= 0; position--) {
        left_arm.write(position);
        delay(15);
    }
}

void activate_right_arm() {
    right_arm.write(90);
    delay(15);
    for (int position = 130; position >= 40; position--) {
        right_arm.write(position);
        delay(15);
    }
}

void Linear(double setpoint_lindis) {
    double setpoint_rot = 0;
    double actposition = (double)((_LeftEncoderTicks + _RightEncoderTicks)) / 2.0;
    double setpoint_lin = actposition + (setpoint_lindis * 4000 )/(2 * 3.14 * 0.04) ;
    double error_lin = setpoint_lin - 0;
    double error_rot = setpoint_rot - 0;
    double feedback_lin = 0;
    double feedback_rot = 0;

    while (abs(error_lin) > 180) {
        nh.spinOnce();
        if(stop_condition == false) {
            error_lin = setpoint_lin - (double)((_LeftEncoderTicks + _RightEncoderTicks))/2.0;
            error_rot = setpoint_rot - (double)((_LeftEncoderTicks - _RightEncoderTicks))/2.0;
            feedback_lin = (double)((_LeftEncoderTicks + _RightEncoderTicks))/2.0;
            feedback_rot =(double)((_LeftEncoderTicks - _RightEncoderTicks))/2.0;
            PWM_lin = diff_PID_lin.step(setpoint_lin, feedback_lin);
            PWM_rot = diff_PID_rot.step(setpoint_rot, feedback_rot);
            PWM_M_Left = PWM_lin + PWM_rot;
            PWM_M_Right = PWM_lin - PWM_rot;

            if(PWM_M_Left > 0) digitalWrite(c_LeftMotorDirPin, HIGH);
            else digitalWrite(c_LeftMotorDirPin, LOW);
            if(PWM_M_Right > 0) digitalWrite(c_RightMotorDirPin, HIGH);
            else digitalWrite(c_RightMotorDirPin, LOW);

            if(abs(PWM_M_Left)-abs(PWM_M_Right) > 10) {
                if(abs(PWM_M_Left)<abs(PWM_M_Right)) {
                    PWM_M_Max_Left = 170;
                    PWM_M_Max_Right = 240;
                }
                else {
                    PWM_M_Max_Left = 240;
                    PWM_M_Max_Right = 170;
                }
            }

            else {
                PWM_M_Max_Left = 170;
                PWM_M_Max_Right = 170;
            }

            if(abs(PWM_M_Left) > PWM_M_Max_Left) PWM_M_Left = PWM_M_Max_Left;
            if(abs(PWM_M_Right) > PWM_M_Max_Right) PWM_M_Right = PWM_M_Max_Right;

            analogWrite(c_RightMotorPwmPin, abs(PWM_M_Right));
            analogWrite(c_LeftMotorPwmPin, abs(PWM_M_Left));
        } // if()
    } // while()
   stopwheels();
}

void Rotation(double angleConsigne) {
    double actposition = (double)((_LeftEncoderTicks + _RightEncoderTicks)) / 2.0;
    double distance_rot = (angleConsigne / 360) * 3.14 * 2 * 0.1775;
    double setpoint_rot= actposition + (distance_rot * 4000 )/(2 * 3.14 * 0.04) ;
    double setpoint_lin = 0;
    double error_lin = setpoint_lin - 0;
    double error_rot = setpoint_rot - 0;
    double feedback_lin = 0;
    double feedback_rot = 0;

    while(abs(error_rot) > 180) {
        nh.spinOnce();
        if(stop_condition == false) {
            error_lin = setpoint_lin - (double)((_LeftEncoderTicks + _RightEncoderTicks))/2.0;
            error_rot = setpoint_rot - (double)((_LeftEncoderTicks - _RightEncoderTicks))/2.0;
            feedback_lin = (double)((_LeftEncoderTicks + _RightEncoderTicks))/2.0;
            feedback_rot =(double)((_LeftEncoderTicks - _RightEncoderTicks))/2.0;
            PWM_lin = diff_PID_lin.step(setpoint_lin, feedback_lin);
            PWM_rot = diff_PID_rot.step(setpoint_rot, feedback_rot);
            PWM_M_Left = PWM_lin + PWM_rot;
            PWM_M_Right =PWM_lin - PWM_rot;

            if(PWM_M_Left > 0) digitalWrite(c_LeftMotorDirPin, HIGH);
            else digitalWrite(c_LeftMotorDirPin, LOW);

            if(PWM_M_Right > 0) digitalWrite(c_RightMotorDirPin, HIGH);
            else digitalWrite(c_RightMotorDirPin, LOW);

            if(abs(PWM_M_Left)-abs(PWM_M_Right) > 10) {
                if(abs(PWM_M_Left)<abs(PWM_M_Right)) {
                    PWM_M_Max_Left = 170;
                    PWM_M_Max_Right = 240;
                }
                else {
                    PWM_M_Max_Left = 240;
                    PWM_M_Max_Right = 170;
                }
            }
            else {
                PWM_M_Max_Left = 170;
                PWM_M_Max_Right = 170;
            }

            if(abs(PWM_M_Left) > PWM_M_Max_Left) PWM_M_Left = PWM_M_Max_Left;
            if(abs(PWM_M_Right) > PWM_M_Max_Right) PWM_M_Right = PWM_M_Max_Right;

            analogWrite(c_RightMotorPwmPin, abs(PWM_M_Right));
            analogWrite(c_LeftMotorPwmPin, abs(PWM_M_Left));
        }
    }
    stopwheels();
}

void stopwheels() {
    analogWrite(c_LeftMotorPwmPin, 0);
    analogWrite(c_RightMotorPwmPin, 0);
}

void HandleLeftMotorInterrupt() {
    _LeftEncoderBCur = ((PINE & B00100000)>>5); //digitalRead(3) PE5
    _LeftEncoderACur = ((PINE & B00010000)>>4); //digitalRead(2) PE4

    _LeftEncoderTicks+=ParseLeftEncoder();

    _LeftEncoderAPrev = _LeftEncoderASet;
    _LeftEncoderBPrev = _LeftEncoderBCur;
}

void HandleRightMotorInterrupt() {
    _RightEncoderBCur = ((PIND & B00000100)>>2); // digitalRead(20) PD2
    _RightEncoderACur = ((PIND & B00001000)>>3); // digitalRead(21) PD3

    _RightEncoderTicks+=ParseRightEncoder();

    _RightEncoderAPrev = _RightEncoderACur;
    _RightEncoderBPrev = _RightEncoderBCur;
}

int ParseLeftEncoder() {
    if(_LeftEncoderAPrev && _LeftEncoderBPrev) {
        if(!_LeftEncoderACur && _LeftEncoderBCur) return 1;
        if(_LeftEncoderACur && !_LeftEncoderBCur) return -1;
    }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev) {
        if(!_LeftEncoderACur && !_LeftEncoderBCur) return 1;
        if(_LeftEncoderACur && _LeftEncoderBCur) return -1;
    }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev) {
        if(_LeftEncoderACur && !_LeftEncoderBCur) return 1;
        if(!_LeftEncoderACur && _LeftEncoderBCur) return -1;
    }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev) {
        if(_LeftEncoderACur && _LeftEncoderBCur) return 1;
        if(!_LeftEncoderACur && !_LeftEncoderBCur) return -1;
    }
    return 0;
}

int ParseRightEncoder() {
    if(_RightEncoderAPrev && _RightEncoderBPrev) {
        c_LeftLedPinState = HIGH;
        c_RightLedPinState = HIGH;
        if(!_RightEncoderACur && _RightEncoderBCur) return 1;
        if(_RightEncoderACur && !_RightEncoderBCur) return -1;
    }else if(!_RightEncoderAPrev && _RightEncoderBPrev) {
        c_LeftLedPinState = LOW;
        c_RightLedPinState = HIGH;
        if(!_RightEncoderACur && !_RightEncoderBCur) return 1;
        if(_RightEncoderACur && _RightEncoderBCur) return -1;
    }else if(!_RightEncoderAPrev && !_RightEncoderBPrev) {
        c_LeftLedPinState = LOW;
        c_RightLedPinState = LOW;
        if(_RightEncoderACur && !_RightEncoderBCur) return 1;
        if(!_RightEncoderACur && _RightEncoderBCur) return -1;
    }else if(_RightEncoderAPrev && !_RightEncoderBPrev) {
        c_LeftLedPinState = HIGH;
        c_RightLedPinState = LOW;
        if(_RightEncoderACur && _RightEncoderBCur) return 1;
        if(!_RightEncoderACur && !_RightEncoderBCur) return -1;
    }
    return 0;
}
