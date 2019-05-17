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
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <SabertoothSimplified.h>

SabertoothSimplified ST;

geometry_msgs::Twist vel_ref;
geometry_msgs::Point32 vel_encoder_robo;
geometry_msgs::Point32 vel_kinematic_robo;

//Right wheel
int encoder0PinA_Right = 5;
int encoder0PinB_Right = 6;
int encoderPinALast_Right= LOW;
int encoder0Pos_Right = 1;
float vel_Right = 0;

//Left wheel
int encoder0PinA_Left = 3;
int encoder0PinB_Left = 4;
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


double arredondar(double valor, int casas, int ceilOrFloor) {
    double arredondado = valor;
    arredondado *= (pow(10, casas));
    if (ceilOrFloor == 0) {
        arredondado = ceil(arredondado);           
    } else {
        arredondado = floor(arredondado);
    }
    arredondado /= (pow(10, casas));
    return arredondado;
}

//ROS Function - Angular and linear Velocity Desired
void velResp(const geometry_msgs::Twist& msg){
  
  //V - linear velocity disired
  vel_ref.linear.x = msg.linear.x;

  //W - angular velocity disired
  vel_ref.angular.z = msg.angular.z;
  
}

ros::NodeHandle  nh;
//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &velResp);
//Publisher
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_robo);

// Robot Differential Drive Kinematic
void kinematic(){

  vel_kinematic_robo.x = ( (2*vel_ref.linear.x) - (vel_ref.angular.z*L) )/2;  // Left wheel
  vel_kinematic_robo.y = ( (2*vel_ref.linear.x) + (vel_ref.angular.z*L) )/2;  // Right wheel

}

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

  //Publish velocity left
  float w_left_encoder = vel_Left;

  //Publisher Encoder
  vel_encoder_robo.x = w_left_encoder;

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

    //round u
    //u = arredondar(u,2,2);

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

    //Output Motor Left
    ST.motor(2, Vl_gain);// vl
  }
}

//Left wheel control
void RosController_Wheel_Right() {

  //Call reference speed from kinematic
  float w_right = vel_kinematic_robo.y;

  //Debug kinematic
  vel_encoder_robo.z = w_right;// - (w_right*0.004);

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

  //Publish velocity left
  float w_right_encoder = vel_Right;

  //Publisher Encoder
  vel_encoder_robo.y = w_right_encoder;

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

    //round u
    //u = arredondar(u,2,2);

    if(w_right == 0){
      //Reset commands
      Media_Vr_encoder = 0;
      encoder0Pos_Right = 1;
      Sum_vel_Right = 0;
      epx_Right = 0;
    }else{
      //Speed saturation conversion
      Vl_gain = round((127 * u)/0.6);
    }

    //Degug-ROS
    //vel_encoder_robo.y = u;

    //Output Motor Left
    ST.motor(1, -Vl_gain);// vr
  }
  
}

// *********************************************

void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  //Serial.begin(9600);
  delay(5000);
  pinMode (encoder0PinA_Right, INPUT);
  pinMode (encoder0PinB_Right, INPUT);
  pinMode (encoder0PinA_Left, INPUT);
  pinMode (encoder0PinB_Left, INPUT);
  
// ROS Initialization with Publishers and Subscribers 
  nh.initNode();
  nh.subscribe(sub_rasp);
  nh.advertise(pub_encoder);

}

void loop()
{
  kinematic();
  RosController_Wheel_Left();
  RosController_Wheel_Right();
  pub_encoder.publish(&vel_encoder_robo);
  nh.spinOnce();
  delay(1);
}