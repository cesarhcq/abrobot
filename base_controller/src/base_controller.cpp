#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
<<<<<<< HEAD
=======
#include <geometry_msgs/Vector3.h>
>>>>>>> gunther_real

double L = 0.5; // distance between axes
double R = 0.0775; // wheel radius 

double encoder_left = 0;
double encoder_right = 0;
<<<<<<< HEAD
double vx = 0;
double vth = 0;
=======
double encoder_dt = 0;
double gyro_x = 0.0;
double gyro_y = 0.0;
double gyro_z = 0.0;


double v_encoder = 0;
double dth_encoder = 0;

>>>>>>> gunther_real
ros::Time encoder_time;
bool init = false;

void handle_vel_encoder(const geometry_msgs::Vector3Stamped& encoder) {
<<<<<<< HEAD
  encoder_left = encoder.vector.y;
  encoder_right = encoder.vector.x;
  encoder_time = encoder.header.stamp;
=======
  encoder_time = encoder.header.stamp;
  encoder_left = encoder.vector.x;
  encoder_right = encoder.vector.y;
  encoder_dt = encoder.vector.z;
>>>>>>> gunther_real

  //ROS_INFO("encoder_left %lf - encoder_right %lf", encoder.vector.x, encoder.vector.y);
}

<<<<<<< HEAD
// Robot Differential Drive Reverse Kinematic
void reverse_kinematics(){
  vx = (encoder_left + encoder_right)/2;
  vth = (encoder_left - encoder_right)/L;

  ROS_INFO("kinematics - vx %lf - vth %lf", vx, vth);
=======
// Gyro Function from Smartphone
void handle_gyro( const geometry_msgs::Vector3& gyro) {
  gyro_x = gyro.x;
  gyro_y = gyro.y;
  gyro_z = gyro.z;
}

// Robot Differential Drive Reverse Kinematic
void reverse_kinematics(){
  v_encoder = (encoder_left + encoder_right)/2; // linear
  dth_encoder = (encoder_right - encoder_left)/L; // angular

  //ROS_INFO("reverse_kinematics - v_encoder %lf - dth_encoder %lf", v_encoder, dth_encoder);
>>>>>>> gunther_real
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle nh;
<<<<<<< HEAD
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = nh.subscribe("/vel_encoder", 100, handle_vel_encoder);
  
  // // Crete tf - base link and Odometry
  tf::TransformBroadcaster baselink_broadcaster;
  tf::TransformBroadcaster odom_broadcaster;

=======
  ros::NodeHandle nh_private_("~");
  ros::Subscriber gyro_sub = nh.subscribe("gyro", 50, handle_gyro);
  ros::Subscriber sub = nh.subscribe("/vel_encoder", 100, handle_vel_encoder);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  // Crete tf - base link and Odometry
  tf::TransformBroadcaster baselink_broadcaster;
  tf::TransformBroadcaster odom_broadcaster;

  double alpha = 0.0;
  bool use_imu = false;
  double vx = 0.0;
  double vth = 0.0;
  double dth = 0.0;

>>>>>>> gunther_real
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

<<<<<<< HEAD
=======
  nh_private_.getParam("alpha", alpha);
  nh_private_.getParam("use_imu", use_imu);

>>>>>>> gunther_real
  ros::Time current_time, last_time;
  ros::Rate r(10.0);

  while(nh.ok()){

<<<<<<< HEAD
    ros::spinOnce(); // check for incoming messages

    baselink_broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
=======
    ros::spinOnce(); //check for incoming messages

    //set tf base_link and laser 
    baselink_broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.16, 0.0, 0.13)),
>>>>>>> gunther_real
      ros::Time::now(),"base_link", "laser"));

    if(!init){

      reverse_kinematics();
      last_time = encoder_time;
      init = true;

    }else if(init){

<<<<<<< HEAD
      reverse_kinematics();
      current_time = encoder_time;

        //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      double delta_x = ( vx * cos(th) ) * dt;
      double delta_y = ( vx * sin(th) ) * dt;
      double delta_th = vth * dt;

=======
      reverse_kinematics(); // return v_encoder and dth_encoder
      current_time = encoder_time;
      //compute odometry in a typical way given the velocities of the robot
      //double dt = (current_time - last_time).toSec();
      double dt = encoder_dt;

      if(use_imu) {
        dth = alpha*dth_encoder + (1-alpha)*gyro_z;
      } else{
        dth = dth_encoder;
      }
    
      // delta position and delta orientation
      double delta_x = ( v_encoder * cos(th) ) * dt;
      double delta_y = ( v_encoder * sin(th) ) * dt;
      double delta_th = dth * dt;

      ROS_INFO("DEBUG - delta_x %lf - delta_y %lf - time: %lf", delta_x, delta_y, dt);

      // position and orientation
>>>>>>> gunther_real
      x += delta_x;
      y += delta_y;
      th += delta_th;

<<<<<<< HEAD
      //ROS_INFO("encoder_left %lf - encoder_right %lf - time: %lf", encoder_left, encoder_right, encoder_time.toSec());
      //ROS_INFO("DEBUG - vx %lf - vth %lf", vx, vth);
=======
      //ROS_INFO("encoder_left %lf - encoder_right %lf - time: %lf", encoder_left, encoder_right, dt);
      //ROS_INFO("DEBUG - v_encoder %lf - dth_encoder %lf", v_encoder, dth);
>>>>>>> gunther_real

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

<<<<<<< HEAD
        //first, we'll publish the transform over tf
=======
      //first, we'll publish the transform over tf
>>>>>>> gunther_real
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

<<<<<<< HEAD
        //send the transform
      odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
=======
      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
>>>>>>> gunther_real
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

<<<<<<< HEAD
        //set the position
=======
      //set the position
>>>>>>> gunther_real
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

<<<<<<< HEAD
        //set the velocity
=======
      vx = (dt == 0)?  0 : v_encoder;
      vth = (dt == 0)? 0 : dth;

      //set the velocity
>>>>>>> gunther_real
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.linear.z = 0;
<<<<<<< HEAD
      odom.twist.twist.angular.z = 0;
      odom.twist.twist.angular.z = 0;
      odom.twist.twist.angular.z = vth;

      if (encoder_left == 0 && encoder_right == 0){
        odom.pose.covariance[0] = 1e-9;
        odom.pose.covariance[7] = 1e-3;
        odom.pose.covariance[8] = 1e-9;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = 1e-9;
        odom.twist.covariance[0] = 1e-9;
        odom.twist.covariance[7] = 1e-3;
        odom.twist.covariance[8] = 1e-9;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = 1e-9;
      }
      else{
        odom.pose.covariance[0] = 1e-3;
        odom.pose.covariance[7] = 1e-3;
        odom.pose.covariance[8] = 0.0;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = 1e3;
        odom.twist.covariance[0] = 1e-3;
        odom.twist.covariance[7] = 1e-3;
        odom.twist.covariance[8] = 0.0;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = 1e3;
      }

        //publish the message
=======
      odom.twist.twist.angular.x = 0;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.z = vth;

      //set the covariance
      // if (encoder_left == 0 && encoder_right == 0){
      odom.pose.covariance[0] = 5.0;
      odom.pose.covariance[7] = 5.0;
      odom.pose.covariance[14] = 1e-3;
      odom.pose.covariance[21] = 0.1;
      odom.pose.covariance[28] = 0.1;
      odom.pose.covariance[35] = 0.1;
      odom.twist.covariance[0] = 1.0;
      odom.twist.covariance[7] = 1e6;
      odom.twist.covariance[14] = 1e6;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 0.5;
      // // }
      // // else{
      //   odom.pose.covariance[0] = 1.0;
      //   odom.pose.covariance[7] = 1.0;
      //   odom.pose.covariance[14] = 1e-3;
      //   odom.pose.covariance[21] = 0.1;
      //   odom.pose.covariance[28] = 0.1;
      //   odom.pose.covariance[35] = 0.1;
      //   odom.twist.covariance[0] = 0.5;
      //   odom.twist.covariance[7] = 1e6;
      //   odom.twist.covariance[14] = 1e6;
      //   odom.twist.covariance[21] = 1e6;
      //   odom.twist.covariance[28] = 1e6;
      //   odom.twist.covariance[35] = 0.1;
      // }

      //publish the message
>>>>>>> gunther_real
      odom_pub.publish(odom);
    }
      // update the time
    last_time = current_time;
    r.sleep();
  }
}