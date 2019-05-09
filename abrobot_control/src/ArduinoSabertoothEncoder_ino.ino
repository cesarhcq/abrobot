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


#define USBCON // RX and TX Arduino Leonardo - Sabertooth
#define USE_USBCON // ROS Arduino Leonardo
#define L 0.5 // distance between axes
#define R 0.0775 // wheel radius 

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <SabertoothSimplified.h>

SabertoothSimplified ST;

geometry_msgs::Twist vel_ref;
geometry_msgs::Point32 vel_kinematic_robo;
geometry_msgs::Point32 vel_encoder_robo;

//Left wheel
// int encoder0PinA_Left = 3;
// int encoder0PinB_Left = 4;
//Left wheel
int encoder0PinA_Left = 3;
int encoder0PinB_Left = 4;
int encoderPinALast_Left = LOW;
int encoder0Pos_Left = 0;
float vel_Left = 0;

//Position encoder
int read_Left = LOW;
float Delta_t = 0;
float Sum_t = 0;
float Sum_vel = 0;
float PreviusMillis = 0;

//Controller variable
double epx_Left = 0;

//ROS Function - Angular and linear Velocity Desired
void velResp(const geometry_msgs::Twist& msg){
  
  //V - linear velocity disired
  vel_ref.linear.x = msg.linear.x;

  //W - angular velocity disired
  vel_ref.angular.z = msg.angular.z;
  
}

// *********************************************
//Left wheel control
void VlRosController() {
  
  double Vl = vel_ref.linear.x;

  float Vl_gain = round(127 * Vl / 0.6);

  //output
  ST.motor(1, 0);// Vl
  ST.motor(2, Vl_gain);// vl

  // Tangential velocity measured by encoder sensor
  read_Left = digitalRead(encoder0PinA_Left);
  if ((encoderPinALast_Left == LOW) && (read_Left == HIGH)) {
    // if (digitalRead(encoder0PinB_Left) == HIGH) {
      encoder0Pos_Left++;
      // Time between encoder signals
      Delta_t = (millis() - PreviusMillis) * 0.001;
      PreviusMillis = millis();

      //Linear speed with respect to Theta = 10 degrees (encoder sensitivity) of wheel displacement of R = 7.5 cm radius.
      float w = (10 * PI / 180) / (Delta_t);
      vel_Left = w*R;
      //Sum_vel = Sum_vel + vel;
    //}
  }
  encoderPinALast_Left = read_Left;
  //Average speed of a wheel Vl
  //double Media_Vl_encoder = Sum_vel / encoder0Pos_Left;
  double Media_Vl_encoder = vel_Left;

  //Publisher Encoder
  vel_encoder_robo.x = Media_Vl_encoder;

  //Vl controll
  double erro = Vl - Media_Vl_encoder;
  //Proportional gain
  double kp = 0.003;
  //Integrative Gain
  double ki = 0.0003;

    // if (abs(erro) > 0.02) {
  //   //PID control
  //   double u = erro * kp + (erro + epx_Left) * ki;
  //   //Integrator Cumulative Error
  //   epx_Left = epx_Left + erro;
  //   //Speed saturation conversion
  //   Vl_gain = round(-127 * u / 0.6);
  // }

  //output
  ST.motor(1, 0);// Vl
  ST.motor(2, Vl_gain);// vl

}

// *********************************************
// Robot Differential Drive Kinematic
void kinematic(){
  // left wheel
  vel_kinematic_robo.x = ( (2*vel_ref.linear.x) - (vel_ref.angular.z*L) )/(2*R);
  // Left wheel
  vel_kinematic_robo.y = ( (2*vel_ref.linear.x) + (vel_ref.angular.z*L) )/(2*R);
}

ros::NodeHandle  nh;
//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &velResp);
//Publisher
ros::Publisher pub_kinematic("/vel_kinematic", &vel_kinematic_robo);
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_robo);


void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  //Serial.begin(9600);
  delay(1000);
  pinMode (encoder0PinA_Left, INPUT);
  pinMode (encoder0PinB_Left, INPUT);
  pinMode (encoder0PinA_Left, INPUT);
  pinMode (encoder0PinB_Left, INPUT);
  
// ROS Initialization with Publishers and Subscribers 
  nh.initNode();
  nh.subscribe(sub_rasp);
  nh.advertise(pub_kinematic);
  nh.advertise(pub_encoder);

}

void loop()
{
  kinematic();
  VlRosController();
  pub_kinematic.publish(&vel_kinematic_robo);
  pub_encoder.publish(&vel_encoder_robo);
  nh.spinOnce();
  delay(1);
}