# GuntherBOT - Autonomous Mobile Robot

GuntherBOT is an autonomous mobile robot developed by [Acta Visio](http://www.acta-visio.com) for warehouse indoor environments. GuntherBOT is based on Arlo Robot as mobile platform to perform different tasks in an indoor environments. GuntherBOT was developed with Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/), Arduino Leonardo and Raspberry Pi 3B.

Version | ROS Distro | Operating System
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

```
cd <sketchbook>/libraries
```
```
rosrun rosserial_arduino make_libraries.py .
```

Close the Arduino IDE and open again. Go to **sketchbook** in the Arduino IDE, and you will see the *ROS_LIB*

Verify the *serial_port* connected. In our case is:

> /dev/ttyACM0


### How to run rplidar ros package

Check the authority of rplidar's serial-port:
```
ls -l /dev |grep ttyUSB
```
Add the authority of write: (such as /dev/ttyUSB0)
```
sudo chmod 666 /dev/ttyUSB0
```
There're two ways to run rplidar ros package

### Simultaneous Localization and Mapping (SLAM) - ROS Navigation Stack - Real Application

#### Mapping

1. Navigate around the environment using Teleop Keyboard.

2. Saving the Map

```
rosrun map_server map_saver -f ~/guntherbot/src/abrobot/abrobot_navigation/maps/test_map
```

3. Loading the map

```
roslaunch abrobot_navigation amcl_demo.launch
```

4. Rviz visualization

```
roslaunch mybot_description mybot_rviz_amcl.launch
```

5. Run tests on Arduino

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

```
rosrun base_controller base_controller
```

6. Run gmaping and run tests AMCL

```
cd ~/ABRobot 
```

```
source devel/setup.bash
```

```
roslaunch base_controller gmapping.launch
```

```
roslaunch abrobot_navigation amcl_demo.launch
```


### WiFi connection between Robot and PC

The GuntherBot has a WiFi access point ```ssid: ubiquityrobot```. 

Assuming that:

* Gunther (IP: 10.42.0.1)

* PC (IP: 10.42.0.98)

1. In the PC or Laptop, open a terminal and type the following command:

```ssh -X ubuntu@10.42.0.1```

```password:ubuntu```

2. In the same terminal of the SSH, type:

```export ROS_MASTER_URI=http://10.42.0.1:11311```

```export ROS_IP=10.42.0.1```

Now, you can execute the launch file with roscore information

3. Open a new terminal again in the PC or Laptop and type:

**Does not need to use ssh again!**

```export ROS_MASTER_URI=http://10.42.0.1:11311```

```export ROS_IP=10.42.0.98```

```rosrun rviz rviz```


