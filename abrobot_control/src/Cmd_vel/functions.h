/*
** kinematic test - interface ros and arduino
*/

#define USE_USBCON
#define L 0.5 // distance between axes
#define R 0.2 // wheel radius 
#include <ros.h>
//#include <SabertoothSimplified.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

geometry_msgs::Twist vel_ref;
geometry_msgs::Point32 vel_kinematic_robo;


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