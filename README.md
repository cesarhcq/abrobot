# GuntherBOT - Autonomous Mobile Robot

GuntherBOT is an autonomous mobile robot developed by [Acta Visio](http://www.acta-visio.com) for warehouse indoor environments. GuntherBOT is based on Arlo Robot as mobile platform to perform different tasks in an indoor environments. GuntherBOT was developed with Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/), Arduino Leonardo and Raspberry Pi 3B.

Version | ROS Version | Operating System
------------ | ------------- | ------------
1.0 | Kinetic Kame | Ubuntu 16.04 (Xenial)


## Steps to create the GuntherBOT based on ROS and Gazebo Simulator


```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

```
rosrun base_controller base_controller
```

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```