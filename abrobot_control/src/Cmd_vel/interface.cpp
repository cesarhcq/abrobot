/*
** kinematic test - interface ros and arduino
*/

#define USE_USBCON
#define L 0.5
#define R 0.2
#include <ros.h>
//#include <SabertoothSimplified.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

geometry_msgs::Twist vel_ref;
geometry_msgs::Point32 vel_kinematic_robo;

void velResp(const geometry_msgs::Twist& msg){
  //V - linear velocity disired
  vel_ref.linear.x = msg.linear.x;

  //W - angular velocity disired
  vel_ref.angular.z = msg.angular.z;
}

void kinematic(){
	// left wheel
 	vel_kinematic_robo.x = ( (2*vel_ref.linear.x) - (vel_ref.angular.z*L) )/(2*R);
 	// right wheel
	vel_kinematic_robo.y = ( (2*vel_ref.linear.x) + (vel_ref.angular.z*L) )/(2*R);
}

void encoder(){
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
  nh.initNode();
  nh.subscribe(sub_rasp);
  nh.advertise(pub_kinematic);
  nh.advertise(pub_encoder);
}

void loop()
{ 
  kinematic();
  pub_kinematic.publish(&vel_kinematic_robo);
  pub_encoder.publish(&vel_ref);
  nh.spinOnce();
  delay(1);
}