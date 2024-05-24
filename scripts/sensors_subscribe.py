#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import LaserScan

# Callback function for wheel encoder positions
def wheel_pos_callback(data):
    # Log the received wheel encoder positions
    rospy.loginfo("Wheel Encoder Positions: %s", data.data)
    # data.data is a list of four values representing the positions of the four wheel encoders

# Callback function for arm positions
def arm_pos_callback(data):
    # Log the received arm positions
    rospy.loginfo("Arm Positions: %s", data.data)
    # data.data is a list of five values representing the positions of the five arm joints

# Callback function for finger positions
def finger_pos_callback(data):
    # Log the received finger positions
    rospy.loginfo("Finger Positions: %s", data.data)
    # data.data is a list of two values representing the positions of the two gripper fingers

# Callback function for IMU yaw data
def yaw_callback(data):
    # Log the received IMU yaw value
    rospy.loginfo("IMU Yaw: %f", data.data)
    # data.data is a single float value representing the yaw angle from the IMU

# Callback function for LIDAR data
def lidar_callback(data):
    # Log the received LIDAR data
    rospy.loginfo("LIDAR Data: min range: %f, max range: %f, ranges: %s",
                  data.range_min, data.range_max, data.ranges)
    # data.range_min is the minimum range value
    # data.range_max is the maximum range value
    # data.ranges is a list of distance measurements from the LIDAR sensor

def control_listener():
    # Initialize the ROS node with the name 'control'
    rospy.init_node('control', anonymous=True)

    # Subscribe to the wheel encoder positions topic
    rospy.Subscriber('wheel/getpos', Float64MultiArray, wheel_pos_callback)
    # This topic publishes the positions of the four wheel encoders as a Float64MultiArray

    # Subscribe to the arm positions topic
    rospy.Subscriber('arm/getpos', Float64MultiArray, arm_pos_callback)
    # This topic publishes the positions of the five arm joints as a Float64MultiArray

    # Subscribe to the finger positions topic
    rospy.Subscriber('finger/getpos', Float64MultiArray, finger_pos_callback)
    # This topic publishes the positions of the two gripper fingers as a Float64MultiArray

    # Subscribe to the IMU yaw topic
    rospy.Subscriber('imu/yaw', Float64, yaw_callback)
    # This topic publishes the yaw angle from the IMU as a Float64

    # Subscribe to the LIDAR data topic
    rospy.Subscriber('lidar/data', LaserScan, lidar_callback)
    # This topic publishes LIDAR data including min range, max range, and a list of distance measurements as a LaserScan message

    # Keep the node running to listen for incoming messages
    rospy.spin()

if __name__ == '__main__':
    try:
        control_listener()
    except rospy.ROSInterruptException:
        pass