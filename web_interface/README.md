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

## 1. Laptop / PC with installed Visual Studio Code (VSC)
[Install MS Visual Studio Code in Ubuntu 16.04/16.10](http://tipsonubuntu.com/2017/03/03/install-ms-visual-studio-code-ubuntu-16-0416-10/).

First, in VSC you need to install the following plugins:

- **Bootstrap 4, Font awesome 4, Font Awesome 5 Free & Pro snippets:** this is the framework itself with couple of additional features which contain ready-to-use templates and other stuff useful while developing in bootstrap

- **Live Server:** to see your web UI changes in realtime

- **Prettier Code formatter:** to keep your source code clean

- **vscode-icons:** to make your workspace look good

## 2. Physical robot or gazebo simulation model.

On top of standard ROS, you will need to install couple additional packages. To do that, in your robot Linux terminal run the following commands:

```
sudo apt update
```

and

```
sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx
```

