#include <ros/ros.h>
#include <ros/time.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>


double L = 0.5; // distance between axes
double R = 0.0775; // wheel radius 

double encoder_left = 0;
double encoder_right = 0;
ros::Time encoder_time;
bool init = false;

void handle_vel_encoder(const geometry_msgs::Vector3Stamped& encoder) {
  encoder_left = encoder.vector.x;
  encoder_right = encoder.vector.y;
  encoder_time = encoder.header.stamp;
}

// Robot Differential Drive Reverse Kinematic
void reverse_kinematics(double vx, double vth){
  vx = (encoder_left + encoder_right)/2;
  vth = (encoder_left - encoder_right)/L;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = nh.subscribe("/vel_encoder", 100, handle_vel_encoder);
  
  // // Crete tf - base link and Odometry
  tf::TransformBroadcaster baselink_broadcaster;
  baselink_broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
      ros::Time::now(),"base_link", "base_laser"));
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double vx = 0;
  double vth = 0;

  ros::Time current_time, last_time;
  ros::Rate r(10.0);

  while(nh.ok()){

    ros::spinOnce(); // check for incoming messages

    if(!init){

      reverse_kinematics(vx, vth);
      last_time = encoder_time;
      init = true;

    }else if(init){

      reverse_kinematics(vx, vth);
      current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      double delta_x = ( vx * cos(th) ) * dt;
      double delta_y = ( vx * sin(th) ) * dt;
      double delta_th = vth * dt;

      x += delta_x;
      y += delta_y;
      th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

        //send the transform
      odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

        //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

        //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.angular.z = vth;

        //publish the message
      odom_pub.publish(odom);
    }
      // update the time
    last_time = current_time;
    r.sleep();
  }
}