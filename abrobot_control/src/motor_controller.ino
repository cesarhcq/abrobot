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

<<<<<<< HEAD
geometry_msgs::Twist vel_ref;
geometry_msgs::Vector3Stamped vel_encoder_robo;
geometry_msgs::Point32 vel_kinematic_robo;

//Right wheel
int encoder0PinA_Right = 5;
//int encoder0PinB_Right = 6;
int encoderPinALast_Right= LOW;
int encoder0Pos_Right = 1;
float vel_Right = 0;

//Left wheel
int encoder0PinA_Left = 3;
//int encoder0PinB_Left = 4;
int encoderPinALast_Left = LOW;
int encoder0Pos_Left = 1;
float vel_Left = 0;

//Position encoder Left
int read_Left = LOW;
float Delta_t_Left = 0;
float Sum_t_Left = 0;
float Sum_vel_Left = 0;
float PreviusMillis_Left = 0;
int cont_Left = 0;

//Position encoder Right
int read_Right = LOW;
float Delta_t_Right = 0;
float Sum_t_Right = 0;
float Sum_vel_Right = 0;
float PreviusMillis_Right = 0;
int cont_Right = 0;

//Controller variable
float epx_Right = 0;
float epx_Left = 0;
=======
char encoder[] = "/encoder";
unsigned long lastMilli = 0;       // loop timing
double vel_req1 = 0;
double vel_req2 = 0;
double vel_act1 = 0;
double vel_act2 = 0;
int PWM_val1 = 0;
int PWM_val2 = 0;

double error = 0;
double pidTerm = 0;
double new_cmd = 0;

// double last_error1 = 0;
// double last_error2 = 0;
// double int_error1 = 0;
// double int_error2 = 0;

double Sum_vel_Left = 0;
double Media_Vl_encoder = 0;
int cont_Left = 1;

double Sum_vel_Right = 0;
double Media_Vr_encoder = 0;
int cont_Right = 1;

//Left wheel encoder
volatile long encoder0Pos_Left = 0;
long encoder0PosAnt_Left = 0;

//Right wheel encoder
volatile long encoder0Pos_Right = 0;
long encoder0PosAnt_Right = 0;

>>>>>>> gunther_real

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

<<<<<<< HEAD
// Robot Differential Drive Kinematic
void kinematic(){
=======
// *********************************************

ros::NodeHandle  nh;
//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &handle_cmd);
//Publisher
geometry_msgs::Vector3Stamped vel_encoder_msg;
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_msg);

// *********************************************
>>>>>>> gunther_real

void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  //Serial.begin(9600);
  delay(3000);
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

<<<<<<< HEAD
ros::NodeHandle  nh;
//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &velResp);
//Publisher
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_robo);

// *********************************************
//Left wheel control
void RosController_Wheel_Left() {

  //Call reference speed from kinematic
  float w_left = vel_kinematic_robo.x + (vel_kinematic_robo.x*0.0672);

  //Debug kinematic
  //vel_encoder_robo.z = w_left;

  //Tangential velocity measured by encoder sensor - Vel_Left
  read_Left = digitalRead(encoder0PinA_Left);
  if ((encoderPinALast_Left == LOW) && (read_Left == HIGH)) {
    // if (digitalRead(encoder0PinB_Left) == HIGH) {
      encoder0Pos_Left++;
      // Time between encoder signals
      Delta_t_Left = (millis() - PreviusMillis_Left) * 0.001;
      PreviusMillis_Left = millis();

      //Linear speed with respect to Theta = 10 degrees (encoder sensitivity) of wheel displacement of R = 7.5 cm radius.
      float w = (10 * PI / 180) / (Delta_t_Left);
      vel_Left = w*R;
      Sum_vel_Left = Sum_vel_Left + vel_Left;

      //Mean of velocity in 30 interations
      cont_Left++;
      if(cont_Left>30){
        Sum_vel_Left = vel_Left;
        encoder0Pos_Left = 1;
        cont_Left = 0;
      }

    //}
  }

  encoderPinALast_Left = read_Left;
  //Average speed of a wheel V_linear
  float Media_Vl_encoder = Sum_vel_Left / encoder0Pos_Left;

  //Publisher Encoder Debug
  //vel_encoder_robo.vector.x = vel_Left;

  //convert sinal to volt
  float Vl_gain;

  //V_linear controll erro = (cinematica - encoder)
  float erro = abs(w_left) - Media_Vl_encoder;
  //Proportional gain
  float kp = 0.2;
  //Integrative Gain
  float ki = 0.0008;

  if (abs(erro) > 0.01) {
    //PID control
    float u = (erro * kp) + ((erro + epx_Left) * ki);
    //Integrator Cumulative Error
    epx_Left = epx_Left + erro;

    //Change the sinal of controll
    if(w_left < 0){
      u = u*(-1);
    }

    if(w_left == 0){
      //Reset commands
      Media_Vl_encoder = 0;
      encoder0Pos_Left = 1;
      Sum_vel_Left = 0;
      epx_Left = 0;
    }else{
      //Speed saturation conversion
      Vl_gain = round((127 * u)/0.6);
    }

    //Degug-ROS
    //vel_encoder_robo.y = u;
=======
void loop()
{
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME){
    getMotorData(time-lastMilli);

    PWM_val1 = updatePid(1, vel_req1, vel_act1);

    //PWM_val1 = ((vel_req1*127)/(0.8));
    //PWM_val2 = ((vel_req2*127)/(0.8));
>>>>>>> gunther_real

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

<<<<<<< HEAD
//Left wheel control
void RosController_Wheel_Right() {

  //Call reference speed from kinematic
  float w_right = vel_kinematic_robo.y;

  //Debug kinematic
  //vel_encoder_robo.z = w_right;// - (w_right*0.004);

  //Tangential velocity measured by encoder sensor - Vel_Left
  read_Right = digitalRead(encoder0PinA_Right);
  if ((encoderPinALast_Right == LOW) && (read_Right == HIGH)) {
    // if (digitalRead(encoder0PinB_Left) == HIGH) {
      encoder0Pos_Right++;
      // Time between encoder signals
      Delta_t_Right = (millis() - PreviusMillis_Right) * 0.001;
      PreviusMillis_Right = millis();

      //Linear speed with respect to Theta = 10 degrees (encoder sensitivity) of wheel displacement of R = 7.5 cm radius.
      float w = (10 * PI / 180) / (Delta_t_Right);
      vel_Right = w*R;
      Sum_vel_Right = Sum_vel_Right + vel_Right;

      //Mean of velocity in 30 interations
      cont_Right++;
      if(cont_Right>30){
        Sum_vel_Right = vel_Right;
        encoder0Pos_Right = 1;
        cont_Right = 0;
      }

    //}
  }

  encoderPinALast_Right = read_Right;
  //Average speed of a wheel V_linear
  float Media_Vr_encoder = Sum_vel_Right / encoder0Pos_Right;

  //Publish velocity right
  float w_right_encoder = vel_Right;

  //Publisher Encoder Debug
  //vel_encoder_robo.vector.y = vel_Right;

  //convert sinal to volt
  float Vl_gain;

  //V_linear controll erro = (cinematica - encoder)
  float erro = abs(w_right) - Media_Vr_encoder;
  //Proportional gain
  float kp = 0.2;
  //Integrative Gain
  float ki = 0.0008;

  if (abs(erro) > 0.01) {
    //PID control
    float u = (erro * kp) + ((erro + epx_Right) * ki);
    //Integrator Cumulative Error
    epx_Right = epx_Right + erro;

    //Change the sinal of controll
    if(w_right < 0){
      u = u*(-1);
    }

    if(w_right == 0){
      //Reset commands
      Media_Vr_encoder = 0;
      encoder0Pos_Right = 1;
      Sum_vel_Right = 0;
      epx_Right = 0;
    }else{
      //Speed saturation conversion
      Vl_gain = round((127 * u)/0.6);
      //Vl_gain = constrain(Vl_gain,-127,127);
     // vel_encoder_robo.vector.z = Vl_gain;
    }

    //Degug-ROS
    //vel_encoder_robo.y = u;
=======
double filterLeft(double vel_left)  {

  Sum_vel_Left = Sum_vel_Left + vel_left;
  double Media = Sum_vel_Left / cont_Left;

  //Mean of velocity in 10 interations
  cont_Left++;
  if(cont_Left>7){
    Sum_vel_Left = 0;
    cont_Left = 1;
  }

  return Media;
}
>>>>>>> gunther_real

double filterRight(double vel_right)  {

  Sum_vel_Right = Sum_vel_Right + vel_right;
  double Media = Sum_vel_Right / cont_Right;

  //Mean of velocity in 10 interations
  cont_Right++;
  if(cont_Right>7){
    Sum_vel_Right = 0;
    cont_Right = 1;
  }

  return Media;
}

<<<<<<< HEAD
void publishEncoder() {

  if(vel_kinematic_robo.x < 0) vel_Left = vel_Left*(-1);
  if(vel_kinematic_robo.y < 0) vel_Right = vel_Right*(-1);

  vel_encoder_robo.header.stamp = nh.now();
  // Using kinematic
  vel_encoder_robo.vector.x = vel_kinematic_robo.x;
  vel_encoder_robo.vector.y = vel_kinematic_robo.y;
  // using Encoder
  // vel_encoder_robo.vector.x = vel_Left;
  // vel_encoder_robo.vector.y = vel_Right
  //vel_encoder_robo.vector.z = 0;
  pub_encoder.publish(&vel_encoder_robo);
  nh.spinOnce();
}

// *********************************************

void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  //Serial.begin(9600);
  delay(5000);
  pinMode (encoder0PinA_Right, INPUT);
  //pinMode (encoder0PinB_Right, INPUT);
  pinMode (encoder0PinA_Left, INPUT);
  //pinMode (encoder0PinB_Left, INPUT);
  
// ROS Initialization with Publishers and Subscribers 
  nh.initNode();
  nh.subscribe(sub_rasp);
  //nh.advertise(pub_kinematic);
  nh.advertise(pub_encoder);
=======
// PID correction - Function

int updatePid(int id, double targetValue, double currentValue) {
  float Kp = 0.8;
  float Kd = 0.0;
  float Ki = 0.1;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  // erro = kinetmatic - encoder
  error = vel_req1-currentValue;
  if (id == 1) {
    
    pidTerm = Kp*error + Ki*int_error1 + Kd*(error-last_error1);
    int_error1 += error;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }

  new_cmd = constrain( ((pidTerm*127)/(0.8)) , -127, 127 );
  //new_cmd = round((pidTerm*127)/(0.8));
>>>>>>> gunther_real

  //new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  //new_cmd = 4095.0*new_pwm/MAX_RPM;

  return int(new_cmd);
}

<<<<<<< HEAD
void loop()
{
  nh.spinOnce();
  kinematic();
  RosController_Wheel_Left();
  RosController_Wheel_Right();
  publishEncoder();
  delay(1);
=======
void publishVEL(unsigned long time) {
  vel_encoder_msg.header.stamp = nh.now();
  vel_encoder_msg.header.frame_id = encoder;
  vel_encoder_msg.vector.x = vel_act1; // encoder left
  vel_encoder_msg.vector.y = pidTerm;  // pid rad/s
  vel_encoder_msg.vector.z = vel_req1;  // pid volts
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
>>>>>>> gunther_real
}