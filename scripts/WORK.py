#!/usr/bin/env python

# Copyright mohis0 [iammohis@gmail.com].

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receiving motor commands (velocity).
"""
import sys
sys.path.append("/usr/local/webots/lib/controller/python")
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import LaserScan, Image
import numpy as np
from controller import Robot
from cv_bridge import CvBridge
import os
import math
import cv2

# Initialize constants and the robot
max_velocity = 2 * math.pi / 2
inf = float('+inf')

robot = Robot()
timeStep = 64  # Simulation time step

# Initialize base motors and encoders
wheel1 = robot.getDevice("wheel1")
wheel2 = robot.getDevice("wheel2")
wheel3 = robot.getDevice("wheel3")
wheel4 = robot.getDevice("wheel4")

encoder1 = robot.getDevice("wheel1sensor")
encoder2 = robot.getDevice("wheel2sensor")
encoder3 = robot.getDevice("wheel3sensor")
encoder4 = robot.getDevice("wheel4sensor")

encoder1.enable(timeStep)
encoder2.enable(timeStep)
encoder3.enable(timeStep)
encoder4.enable(timeStep)

# Set the wheel motors to velocity control mode
wheel1.setPosition(inf)
wheel2.setPosition(inf)
wheel3.setPosition(inf)
wheel4.setPosition(inf)

def go(speed1, speed2, speed3, speed4):
    # Set the velocity for each wheel motor
    wheel1.setVelocity(speed1)
    wheel2.setVelocity(speed2)
    wheel3.setVelocity(speed3)
    wheel4.setVelocity(speed4)

# Initialize arm motors
arm1 = robot.getDevice("arm1")
arm2 = robot.getDevice("arm2")
arm3 = robot.getDevice("arm3")
arm4 = robot.getDevice("arm4")
arm5 = robot.getDevice("arm5")

# Set maximum arm motor velocities
arm1.setVelocity(0.5)
arm2.setVelocity(0.5)
arm3.setVelocity(0.5)
arm4.setVelocity(0.5)
arm5.setVelocity(0.5)

# Get arm motor position limits
arm1MinPosition = arm1.getMinPosition()
arm1MaxPosition = arm1.getMaxPosition()
arm2MinPosition = arm2.getMinPosition()
arm2MaxPosition = arm2.getMaxPosition()
arm3MinPosition = arm3.getMinPosition()
arm3MaxPosition = arm3.getMaxPosition()
arm4MinPosition = arm4.getMinPosition()
arm4MaxPosition = arm4.getMaxPosition()
arm5MinPosition = arm5.getMinPosition()
arm5MaxPosition = arm5.getMaxPosition()

# Initialize arm position sensors
armsensor1 = robot.getDevice("arm1sensor")
armsensor2 = robot.getDevice("arm2sensor")
armsensor3 = robot.getDevice("arm3sensor")
armsensor4 = robot.getDevice("arm4sensor")
armsensor5 = robot.getDevice("arm5sensor")

armsensor1.enable(timeStep)
armsensor2.enable(timeStep)
armsensor3.enable(timeStep)
armsensor4.enable(timeStep)
armsensor5.enable(timeStep)

# Initialize gripper motors
finger1 = robot.getDevice("finger::left")
finger2 = robot.getDevice("finger::right")
finger1.setVelocity(0.03)
finger2.setVelocity(0.03)
fingerMinPosition = finger1.getMinPosition()
fingerMaxPosition = finger1.getMaxPosition()

fingersensor1 = robot.getDevice("finger::leftsensor")
fingersensor2 = robot.getDevice("finger::rightsensor")
fingersensor1.enable(timeStep)
fingersensor2.enable(timeStep)

# Initialize sensors
imu = robot.getDevice("IMU")
imu.enable(timeStep)

lidar = robot.getDevice("LIDAR")
lidar.enable(timeStep)

camera = robot.getDevice("Astra rgb")
camera.enable(timeStep)

camDepth = robot.getDevice("Astra depth")
camDepth.enable(timeStep)

print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
robot.step(timeStep)
rospy.init_node('listener', anonymous=True)

print('Subscribing to topics')
robot.step(timeStep)

def armPos(pos1=None, pos2=None, pos3=None, pos4=None, pos5=None):
    # Set positions for arm joints, ensuring they are within the allowed range
    if pos1 is not None:
        pos1 = max(min(pos1, arm1MaxPosition), arm1MinPosition)
        arm1.setPosition(pos1)
    if pos2 is not None:
        pos2 = max(min(pos2, arm2MaxPosition), arm2MinPosition)
        arm2.setPosition(pos2)
    if pos3 is not None:
        pos3 = max(min(pos3, arm3MaxPosition), arm3MinPosition)
        arm3.setPosition(pos3)
    if pos4 is not None:
        pos4 = max(min(pos4, arm4MaxPosition), arm4MinPosition)
        arm4.setPosition(pos4)
    if pos5 is not None:
        pos5 = max(min(pos5, arm5MaxPosition), arm5MinPosition)
        arm5.setPosition(pos5)

def fingerPos(pos1=None, pos2=None):
    # Set positions for gripper fingers, ensuring they are within the allowed range
    if pos1 is not None:
        pos1 = max(min(pos1, fingerMaxPosition), fingerMinPosition)
        finger1.setPosition(pos1) 
    if pos2 is not None:
        pos2 = max(min(pos2, fingerMaxPosition), fingerMinPosition)
        finger2.setPosition(pos2)

# Callback functions for subscribed topics
def speed_callback(data):
    # Log received velocity values and set the wheel speeds
    rospy.loginfo('Received velocity value: %s', str(data.data))
    speed1, speed2, speed3, speed4 = data.data
    go(speed1, speed2, speed3, speed4)
    
def arm_callback(data):
    # Log received arm position values and set the arm joint positions
    rospy.loginfo('Received arm pos value: %s', str(data.data))
    pos1, pos2, pos3, pos4, pos5 = data.data
    armPos(pos1, pos2, pos3, pos4, pos5)
    
def finger_callback(data):
    # Log received finger position values and set the finger positions
    rospy.loginfo('Received finger pos value: %s', str(data.data))
    pos1, pos2 = data.data
    fingerPos(pos1, pos2)
    
# Subscribing to control topics
rospy.Subscriber('wheel/setvel', Float64, speed_callback)
rospy.Subscriber('arm/setpos', Float64, arm_callback)
rospy.Subscriber('finger/setpos', Float64, finger_callback)

# Define publishers for sensor data
encoder_pub = rospy.Publisher('wheel/getpos', Float64MultiArray, queue_size=10)
arm_sensor_pub = rospy.Publisher('arm/getpos', Float64MultiArray, queue_size=10)
finger_sensor_pub = rospy.Publisher('finger/getpos', Float64MultiArray, queue_size=10)
yaw_pub = rospy.Publisher('imu/yaw', Float64, queue_size=10)
lidar_pub = rospy.Publisher('lidar/data', LaserScan, queue_size=10)
cam_pub = rospy.Publisher('camera/image', Image, queue_size=10)

def lidar_pub_data():
    # Publish LIDAR data
    topicdata = LaserScan()
    data = lidar.getRangeImage()
    datanp = np.array(data)
    topicdata.ranges = data
    topicdata.range_min = datanp.min()
    topicdata.range_max = datanp.max()
    lidar_pub.publish(topicdata)
    
def yaw_pub_data():
    # Publish IMU yaw data
    data = imu.getRollPitchYaw()
    yaw_pub.publish(data[2])
    
def encoders_pub_data():
    # Publish wheel encoder positions
    data = [encoder1.getValue(), encoder2.getValue(), encoder3.getValue(), encoder4.getValue()]
    topicdata = Float64MultiArray()
    topicdata.data = data
    encoder_pub.publish(topicdata)

def camera_pub_data():
    # Publish camera image with depth as alpha channel
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    depth_map = camDepth.getRangeImage()
    width = camDepth.getWidth()
    height = camDepth.getHeight()
    
    depth_array = np.array(depth_map, dtype=np.float64).reshape((height, width))
    min_distance = camDepth.getMinRange()
    max_distance = camDepth.getMaxRange()
    
    depth_array = (depth_array - min_distance) / (max_distance - min_distance)
    depth_array = np.clip(depth_array, 0.0, 1.0)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    image[:, :, 3] = (depth_array * 255).astype(np.uint8)
    
    image_message = CvBridge().cv2_to_imgmsg(image, encoding="bgra8")
    cam_pub.publish(image_message)

def arm_pos_pub_data():
    # Publish arm joint positions
    data = [armsensor1.getValue(), armsensor2.getValue(), armsensor3.getValue(), armsensor4.getValue(), armsensor5.getValue()]
    topicdata = Float64MultiArray()
    topicdata.data = data
    arm_sensor_pub.publish(topicdata)

def finger_pos_pub_data():
    # Publish finger positions
    data = [fingersensor1.getValue(), fingersensor2.getValue()]
    topicdata = Float64MultiArray()
    topicdata.data = data
    finger_sensor_pub.publish(topicdata)

while robot.step(timeStep) != -1 and not rospy.is_shutdown():
    # Publish sensor data continuously
    lidar_pub_data()
    yaw_pub_data()
    encoders_pub_data()
    arm_pos_pub_data()
    finger_pos_pub_data()
    camera_pub_data()


