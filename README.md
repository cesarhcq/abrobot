# GuntherBOT - Autonomous Mobile Robot

GuntherBOT is an autonomous mobile robot developed by [Acta Visio](http://www.acta-visio.com) for warehouse indoor environments. GuntherBOT is based on Arlo Robot as mobile platform to perform different tasks in an indoor environments. GuntherBOT was developed with Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/), Arduino Leonardo and Raspberry Pi 3B.

Version | ROS devel | Operating System
------------ | ------------- | ------------
1.0 | Kinetic Kame | Ubuntu 16.04 LTS (Xenial)

## Install dependencies and follow the installation instructions.

- [x] ROS Kinetic-devel: [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [x] ROS Navigation Stack: [ROS-Planning](https://github.com/ros-planning/navigation).
- [x] Rosserial: [Package for Arduino - Real Robot](http://wiki.ros.org/rosserial).


## Steps to create the GuntherBOT based on ROS and Gazebo Simulator

### The first step

 To create the GuntherBOT is based on Package the contains a C++ parser for the Unified Robot Description Format (URDF), which is an XML format for representing a robot model. The code API of the parser has been through our review process and will remain backwards compatible in future releases. (See more in: [URDF](http://wiki.ros.org/urdf)).

### The robot model is based on Arlo Platform with Differential drive. 

 All components were created in the XML file with joints, collisions, pose, inertia and visual. The script file can be found here: [GuntherBOT description](https://github.com/cesarhcq/abrobot/tree/cesar-working/abrobot_description). The files used in the GuntherBOT simulation are: [Robot Xacro](https://github.com/cesarhcq/abrobot/blob/cesar-working/abrobot_description/urdf/robot2.xacro) and [Robot Gazebo](https://github.com/cesarhcq/abrobot/blob/master/abrobot_description/urdf/robot2.gazebo). If you need more information about [Arlo Description Platform](https://github.com/chrisl8/ArloBot/tree/new-serial-interface/src/arlobot/arlobot_description), visit the [Arlo Bot](https://github.com/chrisl8/ArloBot).

 If you need to add more sensors in your Robot, follow this great tutorial provided by: [Gazebo Sensors](http://gazebosim.org/tutorials/?tut=add_laser). Please, do not forget to add the .dae or .stl extension of the sensors.

## Steps to clone this repository

1. Create a simple ROS Workspace - if you don't have yet. Following the installation instructions.

```
$ mkdir -p ~/guntherBot_ws/src && cd ~/guntherBot_ws

$ catkin init

$ cd ~/guntherBot_ws/src/ 

$ git clone -b cesar-working https://github.com/cesarhcq/abrobot.git

$ cd ~/guntherBot_ws/

$ catkin_make

```

2. Start a simple simulation of the GuntherBOT mobile Robot.

```
$ cd ~/guntherBot_ws/

$ source devel/setup.bash

$ roslaunch abrobot_gazebo second.launch
```

![guntherBOT](https://user-images.githubusercontent.com/15223825/57947842-64adbf00-78b6-11e9-944c-1244ae82ffaa.jpg)

3. Edit world or insert more objects for SLAM






