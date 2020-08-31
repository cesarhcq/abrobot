#!/usr/bin/env python
#http://astrobeano.blogspot.com/2014/01/gy-80-orientation-sensor-on-raspberry-pi.html
#https://github.com/adafruit/Adafruit_Python_ADXL345
#https://github.com/adafruit/Adafruit_Python_ADXL345
#https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x/blob/master/examples/adxl34x_freefall_detection_test.py

import time
import board
import busio
import adafruit_adxl34x

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import sqrt,atan2,cos,sin,pi

def imu_publisher(i2c):
    debug = True
    pub_freq = 10

    gyro_pub = rospy.Publisher('gyro', Vector3, queue_size=50)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=50)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(pub_freq)

    if rospy.has_param('~debug'):
        debug = rospy.get_param('~debug')

    accelerometer = adafruit_adxl34x.ADXL345(i2c)
    accelerometer.enable_freefall_detection()
    # alternatively you can specify attributes when you enable freefall detection for more control:
    # accelerometer.enable_freefall_detection(threshold=10,time=25)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rospy.loginfo("waiting for device...")

    while not rospy.is_shutdown():
        # rospy.loginfo(accelerometer.acceleration)
        # print("%f %f %f"%accelerometer.acceleration)
        # print("Dropped: %s"%accelerometer.events["freefall"])
        if(accelerometer.acceleration != 0):
            current_time = rospy.Time.now()
            print("%f %f %f"%accelerometer.acceleration)
            print("Dropped: %s"%accelerometer.events["freefall"])

            if debug:
                # rospy.loginfo('x %s y %s z %s', gyro_x, gyro_y, gyro_z)

            gyro_msg = Vector3()
            # gyro_msg.x = gyro_x
            # gyro_msg.y = gyro_y
            # gyro_msg.z = gyro_z
            gyro_pub.publish(gyro_msg)

            dt = current_time.to_sec() - last_time.to_sec()
            # theta += dt*gyro_z
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = '/base_link'
            # q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
            # imu_msg.orientation.x = q[0]
            # imu_msg.orientation.y = q[1]
            # imu_msg.orientation.z = q[2]
            # imu_msg.orientation.w = q[3]
            # imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
            # imu_msg.angular_velocity_covariance[0] = -1
            # imu_msg.linear_acceleration_covariance[0] = -1
            imu_pub.publish(imu_msg)

            last_time = current_time
            #rate.sleep()
        else:
            rospy.loginfo("received incomplete i2c packet from IMU")
            continue


if __name__ == '__main__':
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        imu_publisher(i2c)
    except rospy.ROSInterruptException:
        pass