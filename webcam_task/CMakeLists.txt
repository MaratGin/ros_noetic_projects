cmake_minimum_required(VERSION 3.0.2)
project(webcam_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## packages ##
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

add_message_files(
  FILES 
  Motion.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

## build ##
add_executable(webcam_node nodes/subscriber.cpp)

target_include_directories(webcam_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(webcam_node PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS})
