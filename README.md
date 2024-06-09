# ros_work
** ROS based @WORK simulator in webots **
## Installation

1. Install [Webots](https://cyberbotics.com/) (**R2023b**)
2. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
3. Install the `webots_ros` package:

    ```sh
    sudo apt-get install ros-noetic-webots-ros
    ```

## Setup

1. Add the following note to your `.zshrc` or `.bashrc` file:

    ```sh
    export WEBOTS_HOME=/usr/local/webots
    ```

2. Make a workspace and source it in your `.bashrc` or `.zshrc` file:

    ```sh
    source /home/username/catkin_ws/devel/setup.zsh
    ```

3. Go to the `src` folder and clone the project:

    ```sh
    git clone https://github.com/mohis0/ros_work.git
    ```

4. Go to your workspace **root** directory: `path/to/catkin_ws`
5. Check dependencies:

    ```sh
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro noetic
    ```

6. Build your workspace:

    ```sh
    catkin_make
    ```

7. Make sure you are connected to the internet (with **VPN**).
8. Launch your simulator:

    ```sh
    roslaunch ros_work @WORK.launch
    ```

## Info

### Topics

1. **`wheel/getpos`**
   - **Type**: `Float64MultiArray`
   - **Data**: An array of four floats representing the positions of the four wheel encoders.

2. **`arm/getpos`**
   - **Type**: `Float64MultiArray`
   - **Data**: An array of five floats representing the positions of the five arm joints.

3. **`finger/getpos`**
   - **Type**: `Float64MultiArray`
   - **Data**: An array of two floats representing the positions of the two gripper fingers.

4. **`imu/yaw`**
   - **Type**: `Float64`
   - **Data**: A single float representing the yaw value from the IMU.

5. **`lidar/data`**
   - **Type**: `LaserScan`
   - **Data**: A `LaserScan` message containing the range data from the LIDAR.

6. **`camera/image`**
   - **Type**: `Image`
   - **Data**: An `Image` message with a `bgra8` encoding, containing the RGB image and depth data in the alpha channel.

### Image Subscribe Topic

- **Topic**: `/camera/image`
- **Description**: This topic provides images captured from the camera along with depth information. The images are published with a `bgra8` encoding, where the RGB channels contain the visual data and the alpha channel contains depth information. This can be used for visual processing and depth perception tasks in your ROS nodes.

### Sensors Subscribe Topic

- **Node Name**: `sensors_subscribe`
- **Subscribed Topics**:
  - **`wheel/getpos`**: Subscribes to the wheel encoder positions.
  - **`arm/getpos`**: Subscribes to the arm joint positions.
  - **`finger/getpos`**: Subscribes to the gripper finger positions.
  - **`imu/yaw`**: Subscribes to the IMU yaw data.
  - **`lidar/data`**: Subscribes to the LIDAR scan data.

- **Description**: The `sensors_subscribe` node subscribes to various sensor data topics to receive information about the robot's state and environment. This includes wheel encoder positions, arm joint positions, gripper finger positions, IMU yaw data, and LIDAR scan data. The received data can be used for monitoring and controlling the robot's movements and interactions with its environment.

### Control the Robot

To control the robot, you can publish messages to specific topics that the robot subscribes to. Here are the topics you can use to control different aspects of the robot:

- **Wheel Velocity Control**
    - **Topic**: `wheel/setvel`
    - **Message Type**: `Float64MultiArray`
    - **Description**: Sets the velocities of the wheels. The message should be a list of four velocity values for each wheel.

- **Arm Position Control**
    - **Topic**: `arm/setpos`
    - **Message Type**: `Float64MultiArray`
    - **Description**: Sets the positions of the arm joints. The message should be a list of five position values for each arm joint.

- **Gripper Position Control**
    - **Topic**: `finger/setpos`
    - **Message Type**: `Float64MultiArray`
    - **Description**: Sets the positions of the gripper fingers. The message should be a list of two position values for each finger.

#### Example of Publishing Commands

Here is an example of how to publish commands to control the robot using the rostopic command-line tool:

1. **Set Wheel Velocities**:

    ```sh
    rostopic pub /wheel/setvel std_msgs/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5]"
    ```

2. **Set Arm Positions**:

    ```sh
    rostopic pub /arm/setpos std_msgs/Float64MultiArray "data: [0.0, 0.5, 1.0, -0.5, 0.0]"
    ```

3. **Set Gripper Positions**:

    ```sh
    rostopic pub /finger/setpos std_msgs/Float64MultiArray "data: [0.01, 0.01]"
    ```
#### Using Python for Control

You can also use a Python script to publish commands. Below is an example script:


    
    import rospy
    from std_msgs.msg import Float64MultiArray

    def publish_commands():
        rospy.init_node('robot_controller', anonymous=True)
        
        wheel_pub = rospy.Publisher('wheel/setvel', Float64MultiArray, queue_size=10)
        arm_pub = rospy.Publisher('arm/setpos', Float64MultiArray, queue_size=10)
        finger_pub = rospy.Publisher('finger/setpos', Float64MultiArray, queue_size=10)
        
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            wheel_msg = Float64MultiArray(data=[0.5, 0.5, 0.5, 0.5])
            arm_msg = Float64MultiArray(data=[0.0, 0.5, 1.0, -0.5, 0.0])
            finger_msg = Float64MultiArray(data=[0.01, 0.01])
            
            wheel_pub.publish(wheel_msg)
            arm_pub.publish(arm_msg)
            finger_pub.publish(finger_msg)
            
            rate.sleep()

    if __name__ == '__main__':
        try:
            publish_commands()
        except rospy.ROSInterruptException:
            pass

This script initializes the ROS node, creates publishers for each control topic, and sends commands at a rate of 10 Hz. Modify the values in the publish calls to suit your specific control needs.   

