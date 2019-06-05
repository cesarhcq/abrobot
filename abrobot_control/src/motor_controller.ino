// We'll name the Sabertooth object ST.
// For how to configure the Sabertooth, see the DIP Switch Wizard for
// https://www.dimensionengineering.com/datasheets/Sabertooth2x60.pdf
// Be sure to select Simplified Serial Mode for use with this library.
// This sample uses a baud rate of 9600 (page 6 and 16 on Sabertotth2x60 manual).
//
// Connections to make (See Gunther_Assembly_Manual):
//   Arduino TX->1  ->  Sabertooth S1
//   Arduino GND    ->  Sabertooth 0V
//   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//   Arduino PIN 3  ->  Encoder white cable LA 
//   Arduino PIN 4  ->  Encoder white cable LB
//   Arduino PIN 5  ->  Encoder white cable RA
//   Arduino PIN 6  ->  Encoder white cable RB
//   https://learn.parallax.com/tutorials/robot/arlo/arlo-robot-assembly-guide/section-1-motor-mount-and-wheel-kit-assembly/step-6
//
// If you want to use a pin other than TX->1, see the SoftwareSerial example.

#define LOOPTIME        200   // PID loop time(ms)
#define encoder0PinA_Left 2   // encoder A pin Left
#define encoder0PinA_Right 3  // encoder A pin Right
#define encoder0PinB_Left 4   // encoder B pin Left
#define MOTOR_LEFT 2          // motor pin Left
#define MOTOR_RIGHT 1         // motor pin Right

#define PWM_1 9         // motor pin Right
#define PWM_2 10         // motor pin Right

#include "robot_specs.h"
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point32.h>
#include <SabertoothSimplified.h>

SabertoothSimplified ST;

char encoder[] = "/encoder";
unsigned long lastMilli = 0;       // loop timing
double vel_req1 = 0;
double vel_req2 = 0;
double vel_act1 = 0;
double vel_act2 = 0;
int PWM_val1 = 0;
int PWM_val2 = 0;

double pidTerm = 0;
double new_pwm = 0;
double new_cmd = 0;

double Sum_vel_Left = 0;
int cont_Left = 1;

double Sum_vel_Right = 0;
int cont_Right = 1;

//Left wheel encoder
volatile long encoder0Pos_Left = 0;
long encoder0PosAnt_Left = 0;

//Right wheel encoder
volatile long encoder0Pos_Right = 0;
long encoder0PosAnt_Right = 0;


//ROS Function - Angular and linear Velocity Desired
void handle_cmd(const geometry_msgs::Twist& msg){
  
  //V - linear velocity disired
  double x = msg.linear.x;

  //W - angular velocity disired
  double z = msg.angular.z;

  // Robot Differential Drive Kinematic
  vel_req1 = ( (2*x) - (z*L) )/2;  // Left wheel
  vel_req2 = ( (2*x) + (z*L) )/2;  // Right wheel
}

// *********************************************
ros::NodeHandle  nh;
//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &handle_cmd);
//Publisher
geometry_msgs::Vector3Stamped vel_encoder_msg;
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_msg);

// *********************************************

void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  //Serial.begin(9600);
  delay(1000);
  encoder0Pos_Left = 0;
  encoder0Pos_Right = 0;
  encoder0PosAnt_Left = 0;
  encoder0PosAnt_Right = 0;
  vel_req1 = 0;
  vel_req2 = 0;
  vel_act1 = 0;
  vel_act2 = 0;
  PWM_val1 = 0;
  PWM_val2 = 0;

  // filter mean
  Sum_vel_Left = 0;
  cont_Left = 1;

  // filter mean
  Sum_vel_Right = 0;
  cont_Right = 1;

  // ROS Initialization with Publishers and Subscribers 
  nh.initNode();
  nh.subscribe(sub_rasp);
  nh.advertise(pub_encoder);

  pinMode(PWM_1, INPUT);
  pinMode(PWM_2, INPUT);

  pinMode (encoder0PinA_Left, INPUT);
  pinMode (encoder0PinB_Left, INPUT);
  pinMode (encoder0PinA_Right, INPUT);

  digitalWrite(encoder0PinA_Left, HIGH);                // turn on pullup resistor
  digitalWrite(encoder0PinB_Left, HIGH); 
  digitalWrite(encoder0PinA_Right, HIGH);

  attachInterrupt(1, encoder_Left, RISING); // encoder left
  attachInterrupt(0, encoder_Right, RISING); // encoder right

}

void loop()
{
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME){
    getMotorData(time-lastMilli);

    PWM_val1 = updatePid(MOTOR_LEFT, vel_req1, vel_act1);
    PWM_val2 = updatePid(MOTOR_RIGHT, vel_req2, vel_act2);

    //Output Motor Left
    ST.motor(MOTOR_LEFT, PWM_val1);// vl
    //Output Motor Right
    ST.motor(MOTOR_RIGHT, -PWM_val2);// vr

    publishVEL(time-lastMilli);
    lastMilli = time;
  }

}

void getMotorData(unsigned long time)  {
  double dt = time * 0.001;
  double w1 = (encoder_pulse_left * PI / 180);
  double w2 = (encoder_pulse_right * PI / 180);
  double vel_left = (double((encoder0Pos_Left-encoder0PosAnt_Left)*w1*R)/double(dt));
  double vel_right = (double((encoder0Pos_Right-encoder0PosAnt_Right)*w2*R)/double(dt));
  encoder0PosAnt_Left = encoder0Pos_Left;
  encoder0PosAnt_Right = encoder0Pos_Right;

  vel_act1 = filterLeft(vel_left);
  vel_act2 = filterRight(vel_right);
}

double filterLeft(double vel_left)  {

  Sum_vel_Left = Sum_vel_Left + vel_left;
  double filter = Sum_vel_Left / cont_Left;

  //Mean of velocity in 10 interations
  cont_Left++;
  if(cont_Left>7){
    Sum_vel_Left = 0;
    cont_Left = 1;
  }

  return filter;
}

double filterRight(double vel_right)  {

  Sum_vel_Right = Sum_vel_Right + vel_right;
  double filter = Sum_vel_Right / cont_Right;

  //Mean of velocity in 10 interations
  cont_Right++;
  if(cont_Right>7){
    Sum_vel_Right = 0;
    cont_Right = 1;
  }

  return filter;
}

// PID correction - Function

int updatePid(int idMotor, double referenceValue, double encoderValue) {
  float Kp = 0.8;
  float Kd = 0.0;
  float Ki = 0.2;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  // erro = kinetmatic - encoder
  double error = referenceValue-encoderValue;
  if (idMotor == MOTOR_LEFT) {
    
    pidTerm = Kp*error + Ki*int_error1 + Kd*(error-last_error1);
    int_error1 += error;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Ki*int_error2 + Kd*(error-last_error2);
    last_error2 = error;
  }

  if(referenceValue == 0){
    last_error1 = 0;
    last_error2 = 0;
    int_error1 = 0;
    int_error2 = 0;
    pidTerm = 0;
  }

  double constrainMotor = abs(referenceValue)*1.1;

  new_pwm = constrain( ((pidTerm*127)/(0.8)) , -((constrainMotor*127)/(0.8)), ((constrainMotor*127)/(0.8)) );
  new_cmd = constrain( new_pwm , -127, 127 );

  return int(new_cmd);
}

void publishVEL(unsigned long time) {
  vel_encoder_msg.header.stamp = nh.now();
  vel_encoder_msg.header.frame_id = encoder;
  vel_encoder_msg.vector.x = vel_act1;  // encoder left
  vel_encoder_msg.vector.y = vel_act2;  // pid rad/s
  vel_encoder_msg.vector.z = vel_req2;  // reference wr
  //vel_encoder_msg.vector.z = double(time)/1000;
  pub_encoder.publish(&vel_encoder_msg);
  nh.spinOnce();
}

void encoder_Left() {
  if (digitalRead(encoder0PinA_Left) == digitalRead(encoder0PinB_Left)) encoder0Pos_Left++;
  else encoder0Pos_Left--;
}
void encoder_Right() {
  int pwm_value_1 = pulseIn(PWM_1, HIGH);
  int pwm_value_2 = pulseIn(PWM_2, HIGH);
  if ((pwm_value_1 - pwm_value_2) > 0) encoder0Pos_Right++;
  else encoder0Pos_Right--;
}