#ifndef ROBOT_SPECS_H
#define ROBOT_SPECS_H

#define USBCON // RX and TX Arduino Leonardo - Sabertooth
#define USE_USBCON // ROS Arduino Leonardo
#define encoder_pulse 10 //Linear speed with respect to Theta = 10 degrees (encoder sensitivity) of wheel displacement of R = 7.5 cm radius.
#define L 0.5 // distance between axes m
#define R 0.0775 // wheel radius m
#define pi              3.1415926
#define two_pi          6.2831853
#define MAX_VEL         127

#endif