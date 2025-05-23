cmake_minimum_required(VERSION 3.0.2)
project(homography_ros)


find_package(catkin REQUIRED)

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED 
  rospy
  roscpp
  roslib      # evaluation of the ROS-package path 
  cv_bridge   # conversion between OpenCV and ROS-messages 
  sensor_msgs # Image ROS-message
  message_generation
  std_msgs
)


add_service_files(FILES ImageData.srv)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_navigation_demo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
nodes/homography.py
nodes/send_image_service.py
nodes/fake_publisher.py

DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)