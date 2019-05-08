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
#define R 0.075 // wheel radius 

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <SabertoothSimplified.h>

SabertoothSimplified ST;





geometry_msgs::Twist vel_ref;
geometry_msgs::Point32 vel_kinematic_robo;

int val;
int encoder0PinA_Left = 3;
int encoder0PinB_Left = 4;
int encoder0PinA_Right = 5;
int encoder0PinB_Right = 6;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
float  vel = 0;
float  Delta_t = 0;
float Sum_t = 0;
float Sum_vel = 0;
float Media_vel = 0;
float  PreviusMillis = 0;
bool flag = false;

//Controller variable
double epx = 0;
int Converted_Gain = -40; // Start with this value for the wheel to start moving

void full_backward() {
  ST.motor(1, 127);
  ST.motor(2, -127);
}

void full_forward() {
  ST.motor(1, -127);
  ST.motor(2, 127);
}

void half_forward() {
  ST.motor(1, Converted_Gain); //Gain of the PI control
  ST.motor(2, -Converted_Gain);
}

void Stop() {
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void full_right() {
  ST.motor(1, 127);
  ST.motor(2, 127);
}

void half_right() {
  ST.motor(1, 43);
  ST.motor(2, 40);
}

void full_left() {
  ST.motor(1, -127);
  ST.motor(2, -127);
}

//ROS Function - Angular and linear Velocity Desired
void velResp(const geometry_msgs::Twist& msg){
  
  //V - linear velocity disired
  vel_ref.linear.x = msg.linear.x;

  //W - angular velocity disired
  vel_ref.angular.z = msg.angular.z;
  
}
// Robot Differential Drive Kinematic
void kinematic(){
  
  // left wheel
  vel_kinematic_robo.x = ( (2*vel_ref.linear.x) - (vel_ref.angular.z*L) )/(2*R);
  // right wheel
  vel_kinematic_robo.y = ( (2*vel_ref.linear.x) + (vel_ref.angular.z*L) )/(2*R);
}

ros::NodeHandle  nh;
//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &velResp);
//Publisher
ros::Publisher pub_kinematic("/vel_kinematic", &vel_kinematic_robo);
ros::Publisher pub_encoder("/vel_encoder", &vel_ref);


void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  //Serial.begin(9600);
  delay(1000);
  pinMode (encoder0PinA_Right, INPUT);
  pinMode (encoder0PinB_Right, INPUT);
  pinMode (encoder0PinA_Left, INPUT);
  pinMode (encoder0PinB_Left, INPUT);
  
// ROS Initialization with Publishers and Subscribers 

  nh.initNode();
  nh.subscribe(sub_rasp);
  nh.advertise(pub_kinematic);
  nh.advertise(pub_encoder);

}

//Front controlled speed with PI
void encoder_Forward() {
  n = digitalRead(encoder0PinA_Left);
  half_forward();
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB_Left) == LOW) {
      encoder0Pos++;
      // Time between encoder signals
      Delta_t = (millis() - PreviusMillis) * 0.001;
      PreviusMillis = millis();
      Sum_t = Sum_t + Delta_t;
      //Linear speed with respect to 10 degrees (encoder sensitivity) of wheel displacement of 7.5 cm radius.
      vel = ((10 * PI / 180) / Delta_t) * 0.0775;
      Sum_vel = Sum_vel + vel;

      //      Serial.print("Front wheel speed right ");
      //      Serial.print(encoder0Pos);
      //      Serial.print(": ");
      //      Serial.print(vel);
      //      Serial.println(" m/s");
    }
  }

  encoder0PinALast = n;
  //Average speed of a wheel
  Media_vel = Sum_vel / encoder0Pos;
  //Setting the linear velocity input
  //float ref_velo = 0.5;
  float ref_velo = vel_ref.linear.x;
  double erro = ref_velo - Media_vel;
  //Proportional gain
  double kp = 0.003;
  //Integrative Gain
  double ki = 0.0003;


  if (abs(erro) > 0.02) {
    //PID control
    double u = erro * kp + (erro + epx) * ki;
    //Integrator Cumulative Error
    epx = epx + erro;
    //Speed saturation conversion
    Converted_Gain = round(-127 * u / 0.6);
    //Serial.print("Velocity: ");
    //Serial.print(vel);
    //Serial.print(" Erro: ");
    //Serial.print(erro);
    //Serial.print(" Controlled Gain: ");
    //Serial.print(u);
    //Serial.print(" Gain Converted: ");
    //Serial.println(Converted_Gain);
  } else {
//    Serial.print("Controlled! ");
//    Serial.print("Media Velocity: ");
//    Serial.println(Media_vel);
  }

  //Setting the distance to be traveled (100 cm -> 1 meter)
  if (Media_vel * Sum_t >= 100) {
    Stop();
    delay(3000);
    Sum_vel = 0;
    encoder0Pos = 0;
    Sum_t = 0;
    PreviusMillis = millis();
    //Change the flag to enter the rotation routine
    flag = true;
  }
}

//Control of the right rotation distance without PI
void encoder_Right() {
  n = digitalRead(encoder0PinA_Left);
  half_right();
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB_Left) == LOW) {
      encoder0Pos++;
      Delta_t = (millis() - PreviusMillis) * 0.001;
      // Time between encoder signals
      PreviusMillis = millis();
      Sum_t = Sum_t + Delta_t;
      // Velocity with respect to 10 degrees (encoder sensitivity) of wheel displacement of 7.5 cm radius.
      vel = ((10 * PI / 180) / Delta_t) * 0.0775; 
      Sum_vel = Sum_vel + vel;

      //      Serial.print("Velocity Right Rotation Left Wheel ");
      //      Serial.print(encoder0Pos);
      //      Serial.print(": ");
      //      Serial.print(vel);
      //      Serial.println(" m/s");
    }
  }
  encoder0PinALast = n;
  Media_vel = Sum_vel / encoder0Pos;
  //Setting the distance to be traveled (perimeter -> 0.66 = 180 degress)
  if (Media_vel * Sum_t >= 0.66) {
    Stop();
    delay(3000);
    Sum_vel = 0;
    encoder0Pos = 0;
    Sum_t = 0;
    // Time between encoder signals
    PreviusMillis = millis();
    //Change the flag to enter the frontal routine
    flag = false;
  }
}

void loop()
{
  
  encoder_Forward();
  kinematic();
  pub_kinematic.publish(&vel_kinematic_robo);
  pub_encoder.publish(&vel_ref);
  nh.spinOnce();
  delay(1);
  
}