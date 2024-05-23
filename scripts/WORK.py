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

def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data

max_velocity = 2*math.pi/20
inf = float('+inf')
velocity = []

robot = Robot()
timeStep = int(robot.getBasicTimeStep())
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

wheel1.setPosition(inf)
wheel2.setPosition(inf)
wheel3.setPosition(inf)
wheel4.setPosition(inf)

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

print('Subscribing to "motor" topic')
robot.step(timeStep)
rospy.Subscriber('motor', Float64, callback)

# define publishers
encoder_pub = rospy.Publisher('encoder', Float64MultiArray, queue_size=10)
yaw_pub = rospy.Publisher('yaw', Float64, queue_size=10)
lidar_pub = rospy.Publisher('lidar', LaserScan, queue_size=10)
cam_pub = rospy.Publisher('camera', Image, queue_size=10)

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
    cv2.imshow("frame", image)
    cv2.waitKey(1)
    
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
    depth_array[depth_array == inf] = max_distance

    # Clip the values to be within the min and max range
    combined_depth = np.clip(depth_array, min_distance, max_distance)
    normalized_combined_depth = 255 * (combined_depth - min_distance) / (max_distance - min_distance)
    normalized_combined_depth = normalized_combined_depth.astype(np.uint8)
    gray_image = cv2.cvtColor(normalized_combined_depth, cv2.COLOR_GRAY2BGR)

    # Display the image
    cv2.imshow('Combined RangeFinder Depth Map', gray_image)
    cv2.waitKey(1)
    
    # BGRA image (Combining BGR and Depth Data)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    image[:,:,3] = normalized_combined_depth
    
    cv_bridge = CvBridge()
    image_msg = cv_bridge.cv2_to_imgmsg(image, encoding="bgra8")
    data = Image()
    data.data = image_msg
    data.encoding = "bgra8"
    data.height = height
    data.width = width
    cam_pub.publish(data)
    
    
def sensors():
    lidar_pub_data()
    encoders_pub_data()
    yaw_pub_data()
    camera_pub_data()

def go(speed1, speed2, speed3, speed4):
    wheel1.setVelocity(speed1)
    wheel2.setVelocity(speed2)
    wheel3.setVelocity(speed3)
    wheel4.setVelocity(speed4)


print('Running the control loop')

while robot.step(timeStep) != -1 and not rospy.is_shutdown():

    sensors()
    
    go(max_velocity, -max_velocity, -max_velocity, max_velocity)
    
    
cv2.destroyAllWindows()
