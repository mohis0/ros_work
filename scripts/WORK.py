#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
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

max_velocity = 2*math.pi/2
inf = float('+inf')
velocity = []

robot = Robot()
# timeStep = int(robot.getBasicTimeStep())
# You can decrease the timeStep to 32, 16, 8, or 4 milliseconds.
# Be aware of the timeStep value: with a lower timeStep, your code needs to be more optimized 
# to ensure it completes one loop cycle within the timeStep limit.
timeStep = 64

# Initialize base motors.
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

# Activate controlling the motors setting the velocity.
# Otherwise by default the motor expects to be controlled in force or position,
# and setVelocity will set the maximum motor velocity instead of the target velocity.
wheel1.setPosition(inf)
wheel2.setPosition(inf)
wheel3.setPosition(inf)
wheel4.setPosition(inf)

def go(speed1, speed2, speed3, speed4):
    wheel1.setVelocity(speed1)
    wheel2.setVelocity(speed2)
    wheel3.setVelocity(speed3)
    wheel4.setVelocity(speed4)
    
# Ensure that the robot does not jump at the start point
go(0,0,0,0)

# Initialize arm motors.
arm1 = robot.getDevice("arm1")
arm2 = robot.getDevice("arm2")
arm3 = robot.getDevice("arm3")
arm4 = robot.getDevice("arm4")
arm5 = robot.getDevice("arm5")

# Set the maximum arm motors velocity.
arm1.setVelocity(0.5)
arm2.setVelocity(0.5)
arm3.setVelocity(0.5)
arm4.setVelocity(0.5)
arm5.setVelocity(0.5)

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


# Initialize arm position sensors.
# These sensors can be used to get the current joint position and monitor the joint movements.
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

# Initialize gripper motors.
finger1 = robot.getDevice("finger::left")
finger2 = robot.getDevice("finger::right")
# Set the maximum motor velocity.
finger1.setVelocity(0.03)
finger2.setVelocity(0.03)
# Read the minium and maximum position of the gripper motors.
fingerMinPosition = finger1.getMinPosition()
fingerMaxPosition = finger1.getMaxPosition()

fingersensor1 = robot.getDevice("finger::leftsensor")
fingersensor2 = robot.getDevice("finger::rightsensor")
fingersensor1.enable(timeStep)
fingersensor2.enable(timeStep)


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
    if pos1 != None:
        if pos1 > arm1MaxPosition:
            pos1 = arm1MaxPosition
        elif pos1 < arm1MinPosition:
            pos1 = arm1MinPosition
        arm1.setPosition(pos1)
    if pos2 != None:
        if pos2 > arm2MaxPosition:
            pos2 = arm2MaxPosition
        elif pos2 < arm2MinPosition:
            pos2 = arm2MinPosition
        arm2.setPosition(pos2)
    if pos3 != None:
        if pos3 > arm3MaxPosition:
            pos3 = arm3MaxPosition
        elif pos3 < arm3MinPosition:
            pos3 = arm3MinPosition
        arm3.setPosition(pos3)
    if pos4 != None:
        if pos4 > arm4MaxPosition:
            pos4 = arm4MaxPosition
        elif pos4 < arm4MinPosition:
            pos4 = arm4MinPosition
        arm4.setPosition(pos4)
    if pos5 != None:
        if pos5 > arm5MaxPosition:
            pos5 = arm5MaxPosition
        elif pos5 < arm5MinPosition:
            pos5 = arm5MinPosition
        arm5.setPosition(pos5)

def fingerPos(pos1=None, pos2=None):
    if pos1 != None:
        if pos1 > fingerMaxPosition:
            pos1 = fingerMaxPosition
        elif pos1 < fingerMinPosition:
            pos1 = fingerMinPosition
        finger1.setPosition(pos1) 
        
    if pos2 != None:
        if pos2 > fingerMaxPosition:
            pos2 = fingerMaxPosition
        elif pos2 < fingerMinPosition:
            pos2 = fingerMinPosition
        finger2.setPosition(pos2)
        
# Controlling CallBacks
def speed_callback(data):
    message = 'Received velocity value: ' + str(data.data)
    print(message)
    speed1, speed2, speed3, speed4 = data.data
    go(speed1, speed2, speed3, speed4)
    
def arm_callback(data):
    message = 'Received arm pos value: ' + str(data.data)
    print(message)
    pos1, pos2, pos3, pos4, pos5 = data.data
    armPos(pos1, pos2, pos3, pos4, pos5)
    
def finger_callback(data):
    message = 'Received finger pos value: ' + str(data.data)
    print(message)
    pos1, pos2 = data.data
    fingerPos(pos1, pos2)
    
rospy.Subscriber('wheel/setvel', Float64, speed_callback)
rospy.Subscriber('arm/setpos', Float64, arm_callback)
rospy.Subscriber('finger/setpos', Float64, finger_callback)

# define publishers
encoder_pub = rospy.Publisher('wheel/getpos', Float64MultiArray, queue_size=10)
arm_sensor_pub = rospy.Publisher('arm/getpos', Float64MultiArray, queue_size=10)
finger_sensor_pub = rospy.Publisher('finger/getpos', Float64MultiArray, queue_size=10)
yaw_pub = rospy.Publisher('imu/yaw', Float64, queue_size=10)
lidar_pub = rospy.Publisher('lidar/data', LaserScan, queue_size=10)
cam_pub = rospy.Publisher('camera/image', Image, queue_size=10)

def lidar_pub_data():
    topicdata = LaserScan()
    data = lidar.getRangeImage()
    datanp = np.array(data)
    topicdata.ranges = data
    topicdata.range_min = datanp.min()
    topicdata.range_max = datanp.max()
    lidar_pub.publish(topicdata)
    
def yaw_pub_data():
    data = imu.getRollPitchYaw()
    yaw_pub.publish(data[2])
    
def encoders_pub_data():
    data = []
    data.append(encoder1.getValue())
    data.append(encoder2.getValue())
    data.append(encoder3.getValue())
    data.append(encoder4.getValue())
    topicdata = Float64MultiArray()
    topicdata.data = data
    encoder_pub.publish(topicdata)

def camera_pub_data():
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    # uncomment to show RGB image
    # cv2.imshow("frame", image)
    # cv2.waitKey(1)
    
    depth_map = camDepth.getRangeImage()
    # Get the dimensions of the depth map
    width = camDepth.getWidth()
    height = camDepth.getHeight()

    # Convert the depth map to a NumPy array and reshape it
    depth_array = np.array(depth_map, dtype=np.float64)
    # Reshape the array to match the sensor's dimensions (width, height, channels)
    depth_array = depth_array.reshape((height, width))
    min_distance = camDepth.getMinRange()
    max_distance = camDepth.getMaxRange()

    # Clip the values to be within the min and max range
    combined_depth = np.clip(depth_array, min_distance, max_distance)
    normalized_combined_depth = 255 * (combined_depth - min_distance) / (max_distance - min_distance)
    normalized_combined_depth = normalized_combined_depth.astype(np.uint8)

    # Display the depth gray image
    # gray_image = cv2.cvtColor(normalized_combined_depth, cv2.COLOR_GRAY2BGR)
    # cv2.imshow('Combined RangeFinder Depth Map', gray_image)
    # cv2.waitKey(1)
    
    # BGRA image (Combining BGR and Depth Data)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    image[:,:,3] = normalized_combined_depth
    
    cv_bridge = CvBridge()
    image_msg = cv_bridge.cv2_to_imgmsg(image, encoding="bgra8")
    data = Image()
    data.data = image_msg.data
    data.encoding = "bgra8"
    data.height = height
    data.width = width
    cam_pub.publish(data)
    
def arm_pub_data():
    data = []
    data.append(armsensor1.getValue())
    data.append(armsensor2.getValue())
    data.append(armsensor3.getValue())
    data.append(armsensor4.getValue())
    data.append(armsensor5.getValue())
    topicdata = Float64MultiArray()
    topicdata.data = data
    arm_sensor_pub.publish(topicdata)

def finger_pub_data():
    data = []
    data.append(fingersensor1.getValue())
    data.append(fingersensor2.getValue())
    topicdata = Float64MultiArray()
    topicdata.data = data
    finger_sensor_pub.publish(topicdata)
    
def sensors():
    lidar_pub_data()
    encoders_pub_data()
    yaw_pub_data()
    camera_pub_data()
    arm_pub_data()
    finger_pub_data()
 
print('Running the control loop')

while robot.step(timeStep) != -1 and not rospy.is_shutdown():

    sensors()
    
    go(max_velocity, -max_velocity, max_velocity, -max_velocity)
    
    armPos(0, 0, -2.6, 0, 0)
    
    #TODO work just for finger2!!!
    fingerPos(0.015, 0.0)
    
cv2.destroyAllWindows()
