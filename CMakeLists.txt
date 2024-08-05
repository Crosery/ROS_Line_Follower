cmake_minimum_required(VERSION 3.0.2)
project(line_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_follower
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_follower_node src/line_follower_node.cpp)
target_link_libraries(line_follower_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)