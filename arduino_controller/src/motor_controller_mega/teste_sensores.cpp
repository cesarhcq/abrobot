// We'll name the Sabertooth object ST.
// For how to configure the Sabertooth, see the DIP Switch Wizard for
// https://www.dimensionengineering.com/datasheets/Sabertooth2x60.pdf
// Be sure to select Simplified Serial Mode for use with this library.
// This sample uses a baud rate of 9600 (page 6 and 16 on Sabertotth2x60 manual).
//
// Connections to make (See Gunther_Assembly_Manual):
//   Arduino TX->1    ->  Sabertooth S1
//   Arduino Rx->19   ->  Raspberry Rx->10
//   Arduino Tx->18   ->  Raspberry Tx->08
//   Arduino VIN      ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//   Arduino GND      ->  Sabertooth 0V
//   Arduino PIN 2    ->  Encoder white cable LA
//   Arduino PIN 4    ->  Encoder white cable LB
//
//   Arduino PIN 3      ->  Encoder blue RA
//   Arduino PWM_R1(9)  ->  Motor pin Right
//   Arduino PWM_R2(11) ->  Motor pin Right
//   https://learn.parallax.com/tutorials/robot/arlo/arlo-robot-assembly-guide/section-1-motor-mount-and-wheel-kit-assembly/step-6
//
// If you want to use a pin other than TX->1, see the SoftwareSerial example.

#define LOOPTIME        200   // PID loop time(ms)

#define encoder0PinA_Left 2   // Encoder white cable LA
#define encoder0PinB_Left 4   // Encoder white cable LB
#define encoder0PinA_Right 3  // Encoder RA

#define PWM_R1 9              // Motor direction sinal pin Right
#define PWM_R2 11             // Motor direction sinal pin Right

//#define PWM_L1 5               // Motor direction sinal pin Left
//#define PWM_L2 7               // Motor direction sinal pin Left

#define MOTOR_LEFT 2          // Motor Left PID Controll
#define MOTOR_RIGHT 1         // Motor Right PID Controll

#include "robot_specs_mega.h"
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <math.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point32.h>
#include <SabertoothSimplified.h>

SabertoothSimplified ST;

char encoder[] = "/encoder";
char frameid[] ="/sonar_ranger";
char frameid2[] ="/sonar_ranger2";
unsigned long lastMilli = 0;       // loop timing
double vel_req1 = 0;
double vel_req2 = 0;
double vel_act1 = 0;
double vel_act2 = 0;
int PWM_val1 = 0;
int PWM_val2 = 0;

double pidOut = 0;

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

double diffEncoder_Left = 0;
double diffEncoder_Right = 0;

const int trigPin = 7; // ultra sensor 1 pins
const int echoPin= 5;

const int trigPin2 = 8; // ultra sensor 2 pins
const int echoPin2= 6;

int duration;
int duration2;

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
//geometry_msgs::Vector3 vel_encoder_msg;
geometry_msgs::Vector3Stamped vel_encoder_msg;
sensor_msgs::Range sonar_msg;
sensor_msgs::Range sonar_msg2;

ros::Publisher pub_sonar( "rangeSonar", &sonar_msg);
ros::Publisher pub_sonar2( "rangeSonar2", &sonar_msg2);

ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_msg);

// *********************************************

void setup()
{
  SabertoothTXPinSerial.begin(9600); // MEGA-> SABERTOOTH: 0 (RX), 1 (TX)
  Serial1.begin(57600);              // MEGA-> RASPBERRY: Serial1 PINS = 19 (RX), 18 (TX)
  
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

  nh.advertise(pub_sonar);
  nh.advertise(pub_sonar2);

  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);

  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);

  // Settings of Ultrasound
  sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg.header.frame_id =  frameid;
  sonar_msg.field_of_view = (10.0/180.0) * 3.14;
  sonar_msg.min_range = 0.0;
  sonar_msg.max_range = 10.0;

  // Settings of Ultrasound
  sonar_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg2.header.frame_id =  frameid2;
  sonar_msg2.field_of_view = (10.0/180.0) * 3.14;
  sonar_msg2.min_range = 0.0;
  sonar_msg2.max_range = 10.0;

  pinMode(PWM_R1, INPUT);
  pinMode(PWM_R2, INPUT);

  pinMode (encoder0PinA_Left, INPUT);
  pinMode (encoder0PinB_Left, INPUT);
  pinMode (encoder0PinA_Right, INPUT);

  digitalWrite(encoder0PinA_Left, HIGH);                // turn on pullup resistor
  digitalWrite(encoder0PinB_Left, HIGH); 
  digitalWrite(encoder0PinA_Right, HIGH);

  attachInterrupt(0, call_encoder_Left, RISING); // Int.1 encoder left pin 2 (Interrupction - Arduino Mega)
  attachInterrupt(1, call_encoder_Right, RISING); // Int.0 encoder right pin 3 (Interrupction - Arduino Mega)

}


void loop()
{

  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME){
    getMotorData(time-lastMilli);

    PWM_val1 = updatePid(MOTOR_LEFT, vel_req1, vel_act1);
    PWM_val2 = updatePid(MOTOR_RIGHT, vel_req2, vel_act2);

    //PWM_val1 = constrain( ((0.2*127)/0.8) , -127, 127 );
    //PWM_val2 = constrain( ((0.2*127)/0.8) , -127, 127 );

    //Output Motor Left
    ST.motor(MOTOR_LEFT, PWM_val1);// vl
    //Output Motor Right
    ST.motor(MOTOR_RIGHT, -PWM_val2);// vr

    //Call Ultrasounds
    sensor_ultra();
    sensor_ultra2();

    publishVEL(time-lastMilli);
    lastMilli = time;
  }

}

void sensor_ultra(){

  float sensoReading = 0; 
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
    
  duration = pulseIn(echoPin, HIGH);
  sensoReading = duration *0.342/2000;
  //sensoReading = getDistance;
  sonar_msg.range = sensoReading;
  //Serial.println(sensoReading);
  sonar_msg.header.stamp = nh.now();
  pub_sonar.publish(&sonar_msg);
    
}

void sensor_ultra2(){

  float sensoReading = 0; 
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  digitalWrite(trigPin2,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2,HIGH);
  delayMicroseconds(10);
    
  duration2 = pulseIn(echoPin2, HIGH);
  sensoReading = duration2 *0.342/2000;
  //sensoReading = getDistance;
  sonar_msg2.range = sensoReading;
  //Serial.println(sensoReading);
  sonar_msg2.header.stamp = nh.now();
  pub_sonar2.publish(&sonar_msg2);
    
}

void getMotorData(unsigned long time)  {
  double dt = time * 0.001;
  double w1 = (encoder_pulse_left * PI / 180);
  double w2 = (encoder_pulse_right * PI / 180);

  diffEncoder_Left = (encoder0Pos_Left-encoder0PosAnt_Left);
  diffEncoder_Right = (encoder0Pos_Right-encoder0PosAnt_Right);

  double vel_left = (double(diffEncoder_Left*w1*R)/double(dt));
  double vel_right = (double(diffEncoder_Right*w2*R)/double(dt));
  encoder0PosAnt_Left = encoder0Pos_Left;
  encoder0PosAnt_Right = encoder0Pos_Right;

  vel_act1 = filterLeft(vel_left);
  vel_act2 = filterRight(vel_right);
}

double filterLeft(double vel_left)  {

  //Mean of velocity in 10 interations
  cont_Left++;

  Sum_vel_Left = Sum_vel_Left + vel_left;
  double filter = Sum_vel_Left / cont_Left;

  if(cont_Left>encoder_filter){
    Sum_vel_Left = filter;
    cont_Left = 1;
  }

  if(diffEncoder_Left ==0) filter = 0;

  return filter;
}

double filterRight(double vel_right)  {

  //Mean of velocity in 10 interations
  cont_Right++;

  Sum_vel_Right = Sum_vel_Right + vel_right;
  double filter = Sum_vel_Right / cont_Right;

  if(cont_Right>encoder_filter){
    Sum_vel_Right = filter;
    cont_Right = 1;
  }

  if(diffEncoder_Right ==0) filter = 0;

  return filter;
}

// PID correction - Function
int updatePid(int idMotor, double referenceValue, double encoderValue) {
  float Kp = 1.5;  //2.0
  float Kd = 0.1;  //0.1
  float Ki = 0.5;  //0.5
  double pidTerm = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  // erro = kinetmatic - encoder
  double error = referenceValue-encoderValue;
  if (idMotor == 2) { //left
    
    pidTerm = Kp*error + Ki*int_error1 + Kd*(error-last_error1);
    int_error1 += error;
    last_error1 = error;
  }
  else if(idMotor == 1){ //right
    int_error2 += error;
    pidTerm = Kp*error + Ki*int_error2 + Kd*(error-last_error2);
    last_error2 = error;
  }else{
    // last_error1 = 0;
    // last_error2 = 0;
    // int_error1 = 0;
    // int_error2 = 0;
    pidTerm = 0;
  }

  double constrainMotor = abs(referenceValue)*2.0;

  new_pwm = constrain( ((pidTerm*127)/(0.8)) , -((constrainMotor*127)/(0.8)), ((constrainMotor*127)/(0.8)) );
  new_cmd = constrain( new_pwm , -127, 127 );

  return int(new_cmd);
}

void publishVEL(unsigned long time) {
  vel_encoder_msg.header.stamp = nh.now();
  vel_encoder_msg.header.frame_id = encoder;
  vel_encoder_msg.vector.x = vel_act1;  // encoder left
  vel_encoder_msg.vector.y = vel_act2;  // pid rad/s
  vel_encoder_msg.vector.z = double(time)/1000;  // reference wr
  // vel_encoder_msg.x = vel_act1;  // encoder left
  // vel_encoder_msg.y = vel_act2;  // pid rad/s
  // vel_encoder_msg.z = double(time)/1000;
  pub_encoder.publish(&vel_encoder_msg);
  nh.spinOnce();
}

void call_encoder_Left() {
  if (digitalRead(encoder0PinA_Left) == digitalRead(encoder0PinB_Left)) encoder0Pos_Left++;
  else encoder0Pos_Left--;
}
void call_encoder_Right() {
  int pwm_value_1 = pulseIn(PWM_R1, HIGH);
  int pwm_value_2 = pulseIn(PWM_R2, HIGH);
  if ((pwm_value_1 - pwm_value_2) > 0) encoder0Pos_Right++;
  else encoder0Pos_Right--;
}
