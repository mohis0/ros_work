cmake_minimum_required(VERSION 2.8.3)
project(ros_work)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation tf moveit_ros_planning_interface)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
  # add_message_files(
  #   FILES
  # )

## Generate services in the 'srv' folder
  # add_service_files(
  #   FILES
  # )

## Generate added messages and services with any dependencies listed here
  # generate_messages(
  #   DEPENDENCIES
  #   std_msgs
  #   sensor_msgs
  # )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES webots_ros
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#instructions for panoramic_view_recorder node

# add_executable(panoramic_view_recorder src/panoramic_view_recorder.cpp)

# add_dependencies(panoramic_view_recorder webots_ros_generate_messages_cpp)

# target_link_libraries(panoramic_view_recorder
# 	${catkin_LIBRARIES}
# )


#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/ros_controller.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/ros_python.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/webots_launcher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/WORK.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/image_subscribe.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/sensors_subscribe.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})