# GuntherBOT - Autonomous Mobile Robot

GuntherBOT is an autonomous mobile robot developed by [Acta Visio](http://www.acta-visio.com) for warehouse indoor environments. GuntherBOT is based on Arlo Robot as mobile platform to perform different tasks in an indoor environments. GuntherBOT was developed with Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/), Arduino Leonardo and Raspberry Pi 3B.

Version | ROS Distro | Operating System
------------ | ------------- | ------------
1.0 | Kinetic Kame | Ubuntu 16.04 LTS (Xenial)


## Steps to create the GuntherBOT based on ROS and Gazebo Simulator

### The first step to create the GuntherBOT is based on Package the contains a C++ parser for the Unified Robot Description Format (URDF), which is an XML format for representing a robot model. The code API of the parser has been through our review process and will remain backwards compatible in future releases. (See more in: [URDF](http://wiki.ros.org/urdf)).

### The robot model is based on Arlo Platform with Differential drive. All components were created in the XML file with joints, collisions, pose, inertia and visual. The script file can be found here: [GuntherBOT description](https://github.com/cesarhcq/abrobot/tree/cesar-working/abrobot_description). The files used in the GuntherBOT simulation are: [Robot Xacro](https://github.com/cesarhcq/abrobot/blob/cesar-working/abrobot_description/urdf/robot2.xacro) and [Robot Gazebo](https://github.com/cesarhcq/abrobot/blob/master/abrobot_description/urdf/robot2.gazebo).




