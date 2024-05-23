# ros_work

## Installation

1. Install [Webots](https://cyberbotics.com/)
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

**Enjoy :)**
