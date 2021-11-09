     //inspired by markwsilliman/turtlebot repository

#include <ros/ros.h>

     //The following line is where we import the ``MoveBaseAction`` library which is responsible for accepting goals from users and move the robot to the specified location in its world.
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

     //this line is where we create the client that will communicate with actions that adhere to the base station interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  int main(int argc, char** argv){
    ros::init(argc, argv, "map_navigation");
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

     //wait for the action server to come up and then start the process
     while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
     }

     //This is where you create the goal to send to move_base using move_base_msgs::MoveBaseGoal messages to tell the robot to move one meter forward in the coordinate frame.
    move_base_msgs::MoveBaseGoal goal;
    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;
    move_base_msgs::MoveBaseGoal goal3;
    move_base_msgs::MoveBaseGoal goal4;
    move_base_msgs::MoveBaseGoal goal5;
    move_base_msgs::MoveBaseGoal goal6;
    move_base_msgs::MoveBaseGoal goal7;

     //we'll send a goal to the robot to move 1 meter forward
     goal.target_pose.header.frame_id = "map"; // probably use frame_id = "odom" instead "map"
     goal.target_pose.header.stamp = ros::Time::now();

     goal.target_pose.pose.position.x = 2.85; // change all positions according to the topic /move_base_simple/goal
     goal.target_pose.pose.position.y = -1.40;
     goal.target_pose.pose.orientation.z = -0.74;
     goal.target_pose.pose.orientation.w = 0.67;

     goal1.target_pose.header.frame_id = "map";
     goal1.target_pose.header.stamp = ros::Time::now();

     goal1.target_pose.pose.position.x = 2.92;
     goal1.target_pose.pose.position.y = -2.90;
     goal1.target_pose.pose.orientation.z = -0.68;
     goal1.target_pose.pose.orientation.w = 0.72;

     goal2.target_pose.header.frame_id = "map";
     goal2.target_pose.header.stamp = ros::Time::now();

     goal2.target_pose.pose.position.x = 1.39;
     goal2.target_pose.pose.position.y = -3.34;
     goal2.target_pose.pose.orientation.z = 0.98;
     goal2.target_pose.pose.orientation.w = -0.15;

     goal3.target_pose.header.frame_id = "map";
     goal3.target_pose.header.stamp = ros::Time::now();

     goal3.target_pose.pose.position.x = 0.32;
     goal3.target_pose.pose.position.y = -3.9;
     goal3.target_pose.pose.orientation.z = 0.92;
     goal3.target_pose.pose.orientation.w = -0.41;

     goal4.target_pose.header.frame_id = "map";
     goal4.target_pose.header.stamp = ros::Time::now();

     goal4.target_pose.pose.position.x = 2.5;
     goal4.target_pose.pose.position.y = -2.6;
     goal4.target_pose.pose.orientation.z = 0.44;
     goal4.target_pose.pose.orientation.w = 0.89;

     goal5.target_pose.header.frame_id = "map";
     goal5.target_pose.header.stamp = ros::Time::now();

     goal5.target_pose.pose.position.x = 2.69;
     goal5.target_pose.pose.position.y = -0.86;
     goal5.target_pose.pose.orientation.z = -0.17;
     goal5.target_pose.pose.orientation.w = 0.98;


     goal6.target_pose.header.frame_id = "map";
     goal6.target_pose.header.stamp = ros::Time::now();

     goal6.target_pose.pose.position.x = 1.95;
     goal6.target_pose.pose.position.y = 0.26;
     goal6.target_pose.pose.orientation.z = 0.92;
     goal6.target_pose.pose.orientation.w = 0.39;

     goal7.target_pose.header.frame_id = "map";
     goal7.target_pose.header.stamp = ros::Time::now();

     goal7.target_pose.pose.position.x = 3.24;
     goal7.target_pose.pose.position.y = -4.77;
     goal7.target_pose.pose.orientation.z = -0.67;
     goal7.target_pose.pose.orientation.w = 0.74;




     ROS_INFO("Sending goal");

     //this command sends the goal to the move_base node to be processed
  ac.sendGoal(goal);
    
     //After finalizing everything you have to wait for the goal to finish processing
     ac.waitForResult();

     //here we check for the goal if it succeded or failed and send a message according to the goal status.
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robo chegou no waypoint 1");
    ac.sendGoal(goal1);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robo chegou no waypoint 2");
      ac.sendGoal(goal2);
      ac.waitForResult();
    }else{
      ROS_INFO("Falhou waypoint 2");
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robo chegou no waypoint 3");
      ac.sendGoal(goal3);
      ac.waitForResult();
    }else{
      ROS_INFO("Falhou waypoint 4");
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robo chegou no waypoint 4");
      ac.sendGoal(goal4);
      ac.waitForResult();
    }else{
      ROS_INFO("Falhou waypoint 4");
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robo chegou no waypoint 5");
      ac.sendGoal(goal5);
      ac.waitForResult();
    }else{
      ROS_INFO("Falhou waypoint 5");
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robo chegou no waypoint 6");
      ac.sendGoal(goal6);
      ac.waitForResult();
    }else{
      ROS_INFO("Falhou waypoint 6");
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robo chegou no waypoint 7");
      ac.sendGoal(goal7);
      ac.waitForResult();
    }else{
      ROS_INFO("Falhou waypoint 7");
    }


    ROS_INFO("Robo completou a missao");
  }


  return 0;
}