# GuntherBOT - Autonomous Mobile Robot with Interface Web (Bootstrap 4 + ROS)

GuntherBOT is an autonomous mobile robot developed by [Acta Visio](http://www.acta-visio.com) for warehouse indoor environments. GuntherBOT is based on Arlo Robot as mobile platform to perform different tasks in an indoor environments. GuntherBOT was developed with Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/), Arduino Leonardo and Raspberry Pi 3B. 

Bootstrap 4 framework allows you to create modern, great looking websites really quickly and without a big effort. It is currently one of the most popular front-end libraries for a reason. It is also open source [Bootstrap 4 + ROS](https://medium.com/husarion-blog/bootstrap-4-ros-creating-a-web-ui-for-your-robot-9a77a8e373f9) and available under developer friendly MIT license. Building websites which not only look good but also scale well on different devices such as your laptop or a mobile phone is not a big deal with Bootstrap.

In this README We will show how to use that framework together with ROS (Robot Operating System) — the most popular robotic middleware — to create awesome, intuitive web user interfaces for robots.

Version | ROS Distro | Operating System
------------ | ------------- | ------------
1.0 | Kinetic Kame | Ubuntu 16.04 LTS (Xenial)

## Install dependencies and follow the installation instructions.

- [x] ROS Kinetic-devel: [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [x] ROS Navigation Stack: [ROS-Planning](https://github.com/ros-planning/navigation).
- [x] Rosserial: [Package for Arduino - Real Robot](http://wiki.ros.org/rosserial).

## Requirements

1. Laptop / PC with installed Visual Studio Code (VSC) - [Install MS Visual Studio Code in Ubuntu 16.04/16.10](http://tipsonubuntu.com/2017/03/03/install-ms-visual-studio-code-ubuntu-16-0416-10/).

- [x] First, in VSC you need to install the following plugins:

- [x] Bootstrap 4, Font awesome 4, Font Awesome 5 Free & Pro snippets: this is the framework itself with couple of additional features which contain ready-to-use templates and other stuff useful while developing in bootstrap

- [x] Live Server: to see your web UI changes in realtime

- [o] Prettier Code formatter: to keep your source code clean

- [o] vscode-icons: to make your workspace look good

## Steps to create the GuntherBOT based on ROS and Gazebo Simulator

### The first step



## Steps to clone this repository

1. Create a simple ROS Workspace - if you don't have yet. Following the installation instructions.

```
mkdir -p ~/guntherBot_ws/src && cd ~/guntherBot_ws

catkin init

cd ~/guntherBot_ws/src/ 

git clone -b cesar-working https://github.com/cesarhcq/abrobot.git

cd ~/guntherBot_ws/

catkin_make

```

2. Start a simple simulation of the GuntherBOT mobile Robot.

```
cd ~/guntherBot_ws/

source devel/setup.bash

roslaunch abrobot_gazebo second.launch
```

![guntherBOT](https://user-images.githubusercontent.com/15223825/57947842-64adbf00-78b6-11e9-944c-1244ae82ffaa.jpg)

3. Add more objects for Mapping

If you intend to add more objects in the world, you'll need to save as new world file and modify the XML launch file. 

```
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- Initial parameters for position in gazebo world -->

  <arg name ="x" default="0"/>
  <arg name ="y" default="0"/>
  <arg name ="z" default="10.5"/>

  <arg name="world" default="empty"/> 
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>
  
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find abrobot_gazebo)/worlds/test4.world"/>
<!--     <arg name="world_name" value="$(find abrobot_gazebo)/worlds/putyourworldsavedhere.world"/> -->
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find abrobot_description)/urdf/robot2.xacro'"/>

  <node name="abrobot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
   args="-urdf -param robot_description -model abrobot -x $(arg x) -y $(arg y) -z $(arg z)" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>

</launch>
```

4. TODO

## Real GuntherBOT - Integration Raspberry Pi 3, Arduino and ROS

After simulation, you need to do the experiments in real world. It is very important to create a environment with objects and obstacles for mapping and localization (SLAM).

### Communication between Arduino and Raspberry Pi 3

#### Dependencies

- [x] Rosserial: [Package for Arduino - Real Robot](http://wiki.ros.org/rosserial).


1. Plug the Arduino USB cable in the Raspberry Pi 3. Now, open the Arduino IDE to verify what's the USB port connected. If you are not installed Arduino IDE in the Raspberry Pi 3, you can follow this instructions.

```
sudo apt-get update
```
```
sudo apt-get install arduino arduino-core
```
Now, install the ROSSERIAL

```
sudo apt-get install ros-kinetic-rosserial-arduino
```
```
sudo apt-get install ros-kinetic-rosserial
```

After IDE Arduino installed, you'll need to install the **ros_lib library**

The link between ROS and Arduino is through the ros_lib library. This library will be used as any other Arduino library. To install the ros_lib library, type the following commands in the Ubuntu terminal:

