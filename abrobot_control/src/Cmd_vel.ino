/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
geometry_msgs::Twist twist;
void messageCb( const geometry_msgs::Twist& msg){
   
  twist.linear.x = msg.linear.x;

}


ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);


ros::Publisher pub("/cmd_vel2", &twist);

void setup()
{ 

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{ 
  pub.publish(&twist);
  nh.spinOnce();
  delay(1);
}
